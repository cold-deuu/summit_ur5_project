"""03 offline replay orchestration connected to unchanged 01 sensing/motion."""

import copy
import json
from pathlib import Path
import threading

import rospy
import rospkg
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from brl_interfaces.msg import (
    SceneGraphMsg, PrepareNavigationAction, PickTomatoAction,
    ScanTomatoAction, PlaceTomatoAction, Object as ObjectMsg,
)
from brl_interfaces.srv import (
    GenerateTaskPlan, GenerateTaskPlanResponse, UpdateObject, UpdateObjectRequest,
    TriggerSensorScan,
)
from common_models.converters import RosConverter
from .offline_plan import OfflinePlanner, parse_action, execution_action
from .ros_actions import OfflineActionExecutor
from .cached_detection import load_cached_detections


class TaskPlannerNode(OfflineActionExecutor):
    def __init__(self):
        rospy.init_node("task_planner_node", anonymous=False)
        self.robot_name = rospy.get_param("~robot_name", "brl_robot")
        self.detect_settle_time = float(rospy.get_param("~detect_settle_time", 0.5))
        self.action_server_wait_timeout = float(rospy.get_param("~action_server_wait_timeout", 30.0))
        package_path = Path(rospkg.RosPack().get_path("task_planner"))
        self.plan_directory = package_path / "templates/tomato/offline_plans"
        self.detection_config = rospy.get_param("~detection_config", str(package_path / "config/log_detections.json"))
        load_cached_detections(self.detection_config)
        self.default_plan = rospy.get_param("~offline_plan", "") or "T1"
        self.pomdp_planner = self.load_plan(self.default_plan)  # 03 adapter attribute; offline only.
        self.latest_scene = None
        self.holdings = None
        self.tomato_name_mapper = {}
        self.lock = threading.Lock()
        self.scene_condition = threading.Condition()
        self.scene_version = 0
        self.running = self.stop_requested = self.pause_requested = False
        self.worker = None
        self.action_client_specs = {
            "prepare_nav": (f"/{self.robot_name}/prepare_navigation", PrepareNavigationAction),
            "pick": (f"/{self.robot_name}/pick_tomato", PickTomatoAction),
            "scan": (f"/{self.robot_name}/scan_tomato", ScanTomatoAction),
            "place": (f"/{self.robot_name}/place_tomato", PlaceTomatoAction),
        }
        self.action_clients = {"detect": None}
        self.update_object_client = rospy.ServiceProxy("/knowledge_graph_node/object/update", UpdateObject)
        self.start_scan_client = self._scan_service("start")
        self.end_scan_client = self._scan_service("end")
        self.scene_sub = rospy.Subscriber("/scene_graph", SceneGraphMsg, self.scene_graph_callback, queue_size=1)
        self.status_pub = rospy.Publisher("/pomdp_planner/status", String, queue_size=1, latch=True)
        self.current_action_pub = rospy.Publisher("/pomdp_planner/current_action", String, queue_size=1, latch=True)
        self.targets_pub = rospy.Publisher(f"/{self.robot_name}/selected_targets", String, queue_size=1, latch=True)
        self.status_pub.publish(String(data="idle"))
        self.current_action_pub.publish(String(data=""))
        self.targets_pub.publish(String(data="[]"))
        self.services = [rospy.Service("/generate_task_plan", GenerateTaskPlan, self.prepare_callback)]
        # Preserve 03's existing ROS control endpoints; no POMDP search runs.
        for prefix in ("/pomdp_planner", "/task_planner"):
            for name in ("start", "stop", "pause", "resume"):
                self.services.append(rospy.Service(prefix + "/" + name, Trigger, getattr(self, name + "_callback")))
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Offline replay ready: %s (01 sensing/motion)", self.pomdp_planner.path)

    def load_plan(self, selector):
        path = Path(selector).expanduser()
        if not path.suffix:
            path = path.with_suffix(".yaml")
        if not path.is_absolute():
            path = self.plan_directory / path
        planner = OfflinePlanner(path)
        for action in planner.env.actions:
            if parse_action(action.name).arguments[0] != self.robot_name:
                raise ValueError(f"Plan robot must match {self.robot_name}: {action.name}")
        return planner

    def prepare_callback(self, request):
        with self.lock:
            if self.running:
                return GenerateTaskPlanResponse(success=False, message="planner already running")
            try:
                self.pomdp_planner = self.load_plan(request.instruction.strip() or self.default_plan)
                return GenerateTaskPlanResponse(success=True, message="Offline plan loaded: " + str(self.pomdp_planner.path))
            except (OSError, ValueError) as error:
                return GenerateTaskPlanResponse(success=False, message=str(error))

    def start_callback(self, request):
        with self.lock:
            if self.running:
                return TriggerResponse(success=False, message="planner already running")
            requires_robot = any(execution_action(action.name).kind != "detect"
                                 for action in self.pomdp_planner.env.actions)
            if self.latest_scene is None or (requires_robot and self.latest_scene.get_robot_by_name(self.robot_name) is None):
                return TriggerResponse(success=False, message="Waiting for robot in /scene_graph")
            self.pomdp_planner.step = 0
            self.tomato_name_mapper = {}
            self.targets_pub.publish(String(data="[]"))
            self.running = True
            self.stop_requested = self.pause_requested = False
            self.status_pub.publish(String(data="running"))
            self.worker = threading.Thread(target=self.run_planning_loop, daemon=True)
            self.worker.start()
        return TriggerResponse(success=True, message="planner started")

    def run_planning_loop(self):
        status = "stopped"
        try:
            while self.wait_while_paused():
                action = self.pomdp_planner.select_action(None)
                if action is None:
                    status = "completed"
                    break
                action = execution_action(action.name)
                self.current_action_pub.publish(String(data=action.name))
                rospy.loginfo("[Planner] Step %d | Action: %s", self.pomdp_planner.step, action.name)
                success = self.execute_action(action)
                if self.should_stop():
                    break
                if not success:
                    status = "failed"
                    rospy.logerr("Offline physical action failed: %s", action.name)
                    break
        except Exception as error:
            status = "stopped" if self.should_stop() else "failed"
            rospy.logerr("Offline replay stopped: %s", error)
        finally:
            self.current_action_pub.publish(String(data=""))
            with self.lock:
                self.running = False
                self.status_pub.publish(String(data=status))

    def scene_graph_callback(self, message):
        with self.scene_condition:
            self.latest_scene = RosConverter.msg_to_scene_graph(message)
            self.scene_version += 1
            self.scene_condition.notify_all()

    def refresh_scene(self):
        # 01 services return success/message, without 03 symbolic observations.
        # Read a fresh published scene after a service, preserving sensor classes.
        with self.scene_condition:
            version = self.scene_version
            if not self.scene_condition.wait_for(lambda: self.scene_version > version, timeout=3.0):
                raise RuntimeError("Timed out waiting for a fresh /scene_graph")
            return copy.deepcopy(self.latest_scene)

    def _scan_service(self, mode):
        name = f"/{self.robot_name}/{mode}_single_tomato_scan"
        proxy = rospy.ServiceProxy(name, TriggerSensorScan)
        def call(request):
            rospy.wait_for_service(name, timeout=2.0)
            response = proxy(request)
            if not response.success:
                raise RuntimeError(f"{name}: {response.message}")
            if mode == "end":
                self.refresh_scene()
            return response
        return call

    def resolve_action(self, action):
        parsed = execution_action(action.name)
        args = list(parsed.arguments)
        if parsed.kind in {"pick", "scan", "place", "discard"}:
            args[1] = self.tomato_name_mapper.get(args[1], args[1])
        if parsed.kind in {"detect", "pick"}:
            index = 1 if parsed.kind == "detect" else 2
            for location in self.latest_scene.get_all_locations():
                if location.name.lower() == args[index].lower():
                    args[index] = location.name
                    break
        return parse_action(parsed.kind + "(" + ", ".join(args) + ")")

    def cached_detect(self, location):
        config = load_cached_detections(self.detection_config)
        rospy.wait_for_service("/knowledge_graph_node/object/update", timeout=2.0)
        for index, target in enumerate(config["targets"], 1):
            obj = ObjectMsg()
            obj.name = target["name"]
            obj.type = "tomato"
            obj.status = "detected"
            obj.current_location_name = location
            obj.position.x, obj.position.y, obj.position.z = target["position"]
            obj.index = "%d/2" % index
            obj.properties = ["tomato_class:0", "data_source:fixed_log_coordinates",
                              "source_object:" + target["source_object"]]
            response = self.update_object_client(UpdateObjectRequest(object_data=obj))
            if not response.success:
                raise RuntimeError("Cached detect update failed: " + response.message)
        aliases, destinations = [], {}
        for entry in self.pomdp_planner.offline_plan[self.pomdp_planner.step:]:
            step = parse_action(entry["name"])
            if step.kind == "detect":
                break
            if step.kind == "pick" and step.arguments[2].lower() == location.lower():
                if step.arguments[1] not in aliases:
                    aliases.append(step.arguments[1])
            elif step.kind in {"place", "discard"}:
                destinations[step.arguments[1]] = step.kind
        if len(aliases) > 2:
            raise ValueError("Cached detect provides exactly two target positions")
        for alias, target in zip(aliases, config["targets"]):
            self.tomato_name_mapper[alias] = target["name"]
        def persisted():
            if self.latest_scene is None:
                return False
            for target in config["targets"]:
                obj = self.latest_scene.get_object_by_name(target["name"])
                if obj is None or obj.status != "detected" or (obj.position.x, obj.position.y, obj.position.z) != tuple(target["position"]):
                    return False
            return True
        with self.scene_condition:
            if not self.scene_condition.wait_for(persisted, timeout=3.0):
                raise RuntimeError("Fixed detection coordinates have not reached /scene_graph")
        selection = []
        for index, target in enumerate(config["targets"]):
            alias = aliases[index] if index < len(aliases) else target["name"]
            x, y, z = target["position"]
            destination = destinations.get(alias, target["destination"])
            selection.append({"alias": alias, "name": target["name"], "x": x, "y": y, "z": z,
                              "confidence": None, "destination": destination, "mode": "fixed_log_coordinates"})
            rospy.loginfo("[CachedDetect] %s | ripe(class=0) | xyz=(%.3f, %.3f, %.3f) | %s | fixed log coordinates",
                          alias, x, y, z, destination)
        self.targets_pub.publish(String(data=json.dumps(selection)))
        return True

    def execute_action(self, source_action):
        action = self.resolve_action(source_action)
        if self.should_stop():
            return False
        keys = {"prepare_nav": ["prepare_nav"],
                "pick": ["pick"],
                "scan": ["scan"], "place": ["place"], "discard": ["place"]}.get(action.kind, [])
        for key in keys:
            if not self._wait_for_action_server(key):
                return False
        if action.kind == "detect":
            if self.detect_settle_time > 0.0:
                rospy.sleep(self.detect_settle_time)
            return self.cached_detect(action.arguments[1])
        if self.should_stop():
            return False
        success, _, _ = getattr(self, "_execute_" + action.kind)(action)
        if not success or self.should_stop():
            return False
        if action.kind in {"pick", "scan", "place", "discard"}:
            return self.sync_object_status(action)
        return True

    def sync_object_status(self, action):
        scene = self.refresh_scene()
        name = action.arguments[1]
        tomato = scene.get_object_by_name(name)
        if tomato is None:
            raise RuntimeError(f"Object missing from Scene Graph: {name}")
        updated = copy.deepcopy(tomato)
        updated.status = {"pick": "picked", "scan": "scanned",
                          "place": "in_basket", "discard": "deleted"}[action.kind]
        message = RosConverter.object_to_msg(updated, "", self.robot_name)
        rospy.wait_for_service("/knowledge_graph_node/object/update", timeout=2.0)
        response = self.update_object_client(UpdateObjectRequest(object_data=message))
        if not response.success:
            return False
        self.refresh_scene()
        return True

    def shutdown(self):
        with self.lock:
            self.stop_requested = True
            self.pause_requested = False
        for client in self.action_clients.values():
            if client is not None:
                client.cancel_all_goals()
