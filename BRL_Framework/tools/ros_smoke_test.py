#!/usr/bin/env python3
"""Exercise real offline task planner on an isolated ROS master with fake I/O.

Run after sourcing this workspace's devel/setup.bash. The in-memory scene-state
node and offline task planner are real; sensors and motion action servers are
test doubles. No database or hardware nodes are started.
"""

from copy import deepcopy
import os
import json
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
import xmlrpc.client


def main():
    root = Path(__file__).resolve().parents[1]
    log_dir = Path(tempfile.mkdtemp(prefix="brl-framework-smoke-"))
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    os.environ.update(ROS_MASTER_URI=f"http://127.0.0.1:{port}",
                      ROS_IP="127.0.0.1", ROS_HOSTNAME="127.0.0.1",
                      ROS_HOME=str(log_dir), ROS_LOG_DIR=str(log_dir / "ros"))
    for name in tuple(os.environ):
        if name.startswith("NEO4J_"):
            del os.environ[name]
    processes, streams = [], []
    gui_app = gui_window = None
    gui_mode = "--gui" in sys.argv
    gui_pause_gate = threading.Event()
    cancellation_phase = threading.Event()

    def start(command, name):
        stream = (log_dir / f"{name}.log").open("w")
        streams.append(stream)
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        processes.append(process)
        return process

    def wait_until(predicate, timeout=15):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if gui_app is not None:
                gui_app.processEvents()
            if predicate():
                return
            time.sleep(0.05)
        raise AssertionError("Timed out; logs: " + str(log_dir))

    try:
        start(["roscore", "-p", str(port)], "master")

        def master_ready():
            try:
                return xmlrpc.client.ServerProxy(os.environ["ROS_MASTER_URI"]).getPid("smoke")[0] == 1
            except OSError:
                return False

        wait_until(master_ready)
        import actionlib
        import rospy
        from brl_interfaces.msg import (
            SceneGraphMsg, Environment, Location, Robot, Object,
            PrepareNavigationAction, PrepareNavigationResult,
            PickTomatoAction, PickTomatoResult, ScanTomatoAction, ScanTomatoResult,
            PlaceTomatoAction, PlaceTomatoResult,
        )
        from brl_interfaces.srv import (
            UpdateEnvironment, UpdateEnvironmentResponse, UpdateRobot, UpdateRobotResponse,
            UpdateObject, UpdateObjectResponse, TriggerSensorScan, TriggerSensorScanResponse,
            GenerateTaskPlan,
        )
        from std_srvs.srv import Trigger
        from std_msgs.msg import String

        rospy.init_node("framework_smoke_io", disable_signals=True)
        lock = threading.RLock()
        scene = None
        objects, commands, scanned_classes = {}, [], []
        pick_positions, selections = [], []
        scanned_names = set()
        names = {1: "tomato1", 2: "tomato2"}
        statuses, executed_actions = [], []
        status_sub = rospy.Subscriber("/pomdp_planner/status", String, lambda msg: statuses.append(msg.data))
        action_sub = rospy.Subscriber("/pomdp_planner/current_action", String,
                                      lambda msg: executed_actions.append(msg.data))
        targets_sub = rospy.Subscriber("/brl_robot/selected_targets", String, lambda msg: selections.append(json.loads(msg.data)))

        def scene_callback(message):
            nonlocal scene
            with lock:
                scene = message
                objects.clear()
                objects.update((obj.name, obj) for obj in message.objects)
                for obj in message.objects:
                    if obj.status == "scanned" and obj.name not in scanned_names:
                        scanned_names.add(obj.name)
                        scanned_classes.append(dict(p.split(":", 1) for p in obj.properties)["tomato_class"])

        scene_sub = rospy.Subscriber("/scene_graph", SceneGraphMsg, scene_callback, queue_size=1)
        state_command = [sys.executable, str(root / "src/knowledge_interface/scripts/knowledge_graph_node.py")]
        state_node = start(state_command, "state")
        rospy.wait_for_service("/knowledge_graph_node/robot/update", timeout=15)
        rospy.wait_for_service("/knowledge_graph_node/object/update", timeout=15)
        wait_until(lambda: scene is not None)
        config = json.loads((root / "src/knowledge_interface/config/real_farm_config.json").read_text())
        assert scene.environment.name == config["environment"]["name"]
        assert not scene.robots and not scene.objects
        positions = {loc.name: [loc.position.x, loc.position.y, loc.position.theta] for loc in scene.locations}
        assert positions == {loc["name"]: loc["position"] for loc in config["locations"]}

        robot_client = rospy.ServiceProxy("/knowledge_graph_node/robot/update", UpdateRobot)
        object_client = rospy.ServiceProxy("/knowledge_graph_node/object/update", UpdateObject)
        env_client = rospy.ServiceProxy("/knowledge_graph_node/environment/update", UpdateEnvironment)
        assert not robot_client(robot_data=Robot()).success
        assert not object_client(object_data=Object()).success
        assert not env_client(status="").success
        assert env_client(status="task_planning").success
        wait_until(lambda: scene.environment.status == "task_planning")
        robot = Robot(name="brl_robot", type="harvester", status="active",
                      capabilities=["NavigateTo", "Harvest"])
        robot.position.x, robot.position.y, robot.position.theta = positions["Stem_01"]
        assert robot_client(robot_data=robot).success
        wait_until(lambda: bool(scene.robots))
        assert scene.robots[0].current_location_name == "Stem_01"

        planner = start([sys.executable, str(root / "src/task_planner/scripts/task_planner_node.py")], "planner")
        rospy.wait_for_service("/generate_task_plan", timeout=15)
        rospy.wait_for_service("/pomdp_planner/start", timeout=15)
        time.sleep(0.2)
        generate = rospy.ServiceProxy("/generate_task_plan", GenerateTaskPlan)
        # No camera topics, detector node, or trigger_sensor_scan service exist.
        # The production task planner must create the two log-derived targets.
        def end_scan(request):
            with lock:
                obj = deepcopy(objects[request.target_object])
            obj.properties = ["tomato_class:" + ("0" if obj.name == names[1] else "2"), "confidence:1.0"]
            obj.current_location_name = ""
            obj.held_by_robot_name = "brl_robot"
            response = object_client(object_data=obj)
            return TriggerSensorScanResponse(success=response.success, message=response.message)

        services = [
            rospy.Service("/brl_robot/start_single_tomato_scan", TriggerSensorScan,
                          lambda request: TriggerSensorScanResponse(success=True, message="test scan start")),
            rospy.Service("/brl_robot/end_single_tomato_scan", TriggerSensorScan, end_scan),
        ]
        servers = []
        for name, action_type, result_type in [
            ("prepare_navigation", PrepareNavigationAction, PrepareNavigationResult),
            ("pick_tomato", PickTomatoAction, PickTomatoResult),
            ("scan_tomato", ScanTomatoAction, ScanTomatoResult),
            ("place_tomato", PlaceTomatoAction, PlaceTomatoResult),
        ]:
            def callback(goal, name=name, result_type=result_type):
                with lock:
                    commands.append((name, getattr(goal, "tomato_name", ""), getattr(goal, "tomato_class", "")))
                    if name == "pick_tomato":
                        pick_positions.append((goal.tomato_position.x, goal.tomato_position.y, goal.tomato_position.z))
                server = next(server for server_name, server in servers if server_name == name)
                # Keep the fake current action active until the Qt pause service
                # is acknowledged; service process startup has variable latency.
                if gui_mode and name == "prepare_navigation" and len(commands) == 1:
                    gui_pause_gate.wait(timeout=10)
                elif cancellation_phase.is_set() and name == "prepare_navigation":
                    deadline = time.monotonic() + 10
                    while not server.is_preempt_requested() and time.monotonic() < deadline:
                        time.sleep(0.02)
                else:
                    time.sleep(0.2)
                server = next(server for server_name, server in servers if server_name == name)
                if server.is_preempt_requested():
                    server.set_preempted(result_type(success=False, message="test cancellation"))
                else:
                    server.set_succeeded(result_type(success=True, message="test action complete"))
            server = actionlib.SimpleActionServer("/brl_robot/" + name, action_type, callback, auto_start=False)
            servers.append((name, server))
            server.start()

        if gui_mode:
            os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
            import importlib.util
            from PyQt5.QtWidgets import QApplication
            spec = importlib.util.spec_from_file_location("brl_framework_gui", root / "run_master_gui.py")
            gui = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(gui)
            gui_app = QApplication(sys.argv[:1])
            gui_window = gui.MasterWindow(gui.parse_args(["--log-dir", str(log_dir / "gui")]))
            gui_window.show()
            wait_until(lambda: gui_window.connected and gui_window.load_button.isEnabled())
            gui_window.load_button.click()
            wait_until(lambda: not gui_window.control_busy)
            assert "불러오기 완료" in gui_window.status_label.text(), gui_window.control_log.toPlainText()
        else:
            response = generate(instruction="T1")
            assert response.success, response.message
        time.sleep(0.25)
        assert not commands, "Loading an offline plan sent action goals"
        if gui_mode:
            gui_window.start_button.click()
            wait_until(lambda: bool(commands) and gui_window.pause_button.isEnabled())
            gui_window.pause_button.click()
            wait_until(lambda: gui_window.pause_requested)
            gui_pause_gate.set()
        else:
            assert rospy.ServiceProxy("/pomdp_planner/start", Trigger)().success
            wait_until(lambda: bool(commands))
            assert rospy.ServiceProxy("/pomdp_planner/pause", Trigger)().success
        time.sleep(0.5)
        assert len(commands) == 1, commands
        if gui_mode:
            gui_window.resume_button.click()
        else:
            assert rospy.ServiceProxy("/pomdp_planner/resume", Trigger)().success
        wait_until(lambda: statuses and statuses[-1] in {"completed", "failed"}, timeout=40)
        assert statuses[-1] == "completed", statuses
        assert planner.poll() is None, "Offline planner exited"
        assert [name for name, _, _ in commands] == [
            "prepare_navigation", "pick_tomato", "scan_tomato", "place_tomato",
            "pick_tomato", "scan_tomato", "place_tomato", "prepare_navigation",
        ], commands
        assert commands[3][2] == "0", commands
        assert commands[6][2] == "2", commands
        assert commands[1][1] == names[1] and commands[4][1] == names[2], commands
        assert [name for name in executed_actions if name.startswith(("pick(", "scan("))] == [
            "pick(brl_robot, tomato1, stem_01)", "scan(brl_robot, tomato1)",
            "pick(brl_robot, tomato2, stem_01)", "scan(brl_robot, tomato2)",
        ], executed_actions
        assert pick_positions == [(0.216, -0.620, 1.043), (0.408, -0.639, 1.078)], pick_positions
        assert any([target["name"] for target in selected] == [names[1], names[2]]
                   and [target["destination"] for target in selected] == ["place", "discard"]
                   for selected in selections), selections
        assert scanned_classes == ["0", "2"], scanned_classes
        assert objects[names[1]].status == "in_basket", objects
        assert names[2] not in objects, objects
        assert scene.robots[0].current_location_name == "Stem_01"
        assert (scene.robots[0].position.x, scene.robots[0].position.y, scene.robots[0].position.theta) == tuple(positions["Stem_01"])
        print("PASS: single default T1 plan executes prepare_nav and separate pick/scan for place/discard; base position stays unchanged")
        # Cancellation still applies to an explicit prepare_nav arm action.
        cancel_plan = log_dir / "explicit_prepare_nav.yaml"
        cancel_plan.write_text("offline_actions:\n  - index: 1\n    name: prepare_nav(brl_robot)\n")
        before_stop = len(commands)
        cancellation_phase.set()
        if gui_mode:
            wait_until(lambda: not gui_window.control_busy and gui_window.runtime_status == "completed")
            gui_window.plan_path.setText(str(cancel_plan))
            gui_window.start_button.click()
            wait_until(lambda: len(commands) > before_stop and gui_window.stop_button.isEnabled())
            gui_window.stop_button.click()
        else:
            assert generate(instruction=str(cancel_plan)).success
            assert rospy.ServiceProxy("/task_planner/start", Trigger)().success
            wait_until(lambda: len(commands) > before_stop)
            assert rospy.ServiceProxy("/task_planner/stop", Trigger)().success
        wait_until(lambda: statuses[-1] == "stopped")
        time.sleep(0.3)
        assert len(commands) == before_stop + 1, commands
        assert not generate(instruction="/nonexistent/brl_plan.yaml").success
        # Repeating detect restores the exact same two values even after place
        # and discard, with no live detection service or camera available.
        cancellation_phase.clear()
        detect_plan = log_dir / "cached_detect.yaml"
        detect_plan.write_text("offline_actions:\n  - index: 1\n    name: detect(brl_robot, stem_01)\n")
        before_commands = len(commands)
        for _ in range(2):
            assert generate(instruction=str(detect_plan)).success
            before_status = len(statuses)
            assert rospy.ServiceProxy("/task_planner/start", Trigger)().success
            wait_until(lambda: "running" in statuses[before_status:] and statuses[-1] == "completed")
            wait_until(lambda: set(objects) == {"tomato1", "tomato2"} and all(obj.status == "detected" for obj in objects.values()))
            assert [(objects[name].position.x, objects[name].position.y, objects[name].position.z)
                    for name in ("tomato1", "tomato2")] == [(0.216, -0.620, 1.043), (0.408, -0.639, 1.078)]
        assert len(commands) == before_commands, "Detect itself sent a motion goal"
        print("PASS: cached detect always provides both fixed logged XYZ without a camera/detector; repeated detect restores the same values and emits no motion")
        state = rospy.get_master().getSystemState()[2][2]
        assert not any(name.startswith("/cf/") for name, _ in state)
        assert not any(name == "/brl_robot/trigger_sensor_scan" for name, _ in state)
        topics = rospy.get_published_topics()
        assert not any("move_base" in name or "cmd_vel" in name or name.endswith("gripper/cmd") for name, _ in topics)
        if gui_mode:
            wait_until(lambda: gui_window.runtime_status == "completed")
            assert all(card.process is None for card in gui_window.cards)
            gui_window.close()
            wait_until(lambda: gui_window.close_allowed)
            assert planner.poll() is None and state_node.poll() is None
            print("PASS: actual GUI buttons and ROS telemetry; closing GUI preserves external nodes")
        # Process lifetime is the only storage: restarting loads static JSON only.
        state_node.send_signal(signal.SIGINT)
        assert state_node.wait(timeout=5) == 0
        state_node = start(state_command, "state_restart")
        wait_until(lambda: scene is not None and not scene.robots and not scene.objects)
        assert scene.environment.name == config["environment"]["name"]
        assert scene.environment.status == config["environment"].get("status", "active")
        assert state_node.poll() is None
        # Detect-only works without robot/AMCL registration.
        wait_until(lambda: not scene.robots)
        assert generate(instruction=str(detect_plan)).success
        before_status = len(statuses)
        before_commands = len(commands)
        assert rospy.ServiceProxy("/task_planner/start", Trigger)().success
        wait_until(lambda: "running" in statuses[before_status:] and statuses[-1] == "completed")
        wait_until(lambda: len(scene.objects) == 2)
        assert not scene.robots and len(commands) == before_commands
        print("PASS: isolated detect test works without a camera, detector, robot metadata, or any motion goal")
        print("PASS: real in-memory state node without DB credentials; fixed logged detections without cameras, original farm coordinates, ROS updates, place/discard, pause/resume, cancellation, and reset on restart")
        print("Action sequence:", commands)
        print("Logs:", log_dir)
        rospy.signal_shutdown("smoke test finished")
    except Exception:
        print("Smoke test failed; logs:", log_dir, file=sys.stderr)
        raise
    finally:
        gui_pause_gate.set()
        if "rospy" in sys.modules:
            sys.modules["rospy"].signal_shutdown("smoke test cleanup")
        if gui_window is not None and not gui_window.close_allowed:
            gui_window.close()
            try:
                wait_until(lambda: gui_window.close_allowed, timeout=10)
            except AssertionError:
                pass
        for process in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
        for stream in streams:
            stream.close()


if __name__ == "__main__":
    main()
