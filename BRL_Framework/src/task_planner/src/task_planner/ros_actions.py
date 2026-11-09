"""Physical action adapters extracted from 03's offline ROS execution path.

01 sensor services do not return symbolic facts; optional facts are read with
getattr. Pick, scan, place, and discard goals otherwise follow the 03 code.
"""
import time
import actionlib
import rospy
from brl_interfaces.msg import (PickTomatoGoal, ScanTomatoGoal, PlaceTomatoGoal, PrepareNavigationGoal)
from brl_interfaces.srv import TriggerSensorScanRequest
from common_models.converters import RosConverter
from std_srvs.srv import TriggerResponse


class OfflineActionExecutor:
    def scene_graph_callback(self, msg):
        try:
            self.latest_scene = RosConverter.msg_to_scene_graph(msg)
        except Exception as e:
            rospy.logerr(f"Error in scene_graph_callback: {e}")

    def _wait_for_action_server(self, action_client_key):
        """Tolerate delayed actionlib discovery under heavy ROS traffic."""
        deadline = time.monotonic() + self.action_server_wait_timeout
        attempt = 0

        while not rospy.is_shutdown() and not self.should_stop():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return False

            attempt += 1
            service_name, action_type = self.action_client_specs[action_client_key]
            client = actionlib.SimpleActionClient(service_name, action_type)
            self.action_clients[action_client_key] = client
            wait_seconds = min(5.0, remaining)
            if client.wait_for_server(rospy.Duration(wait_seconds)):
                if action_client_key == "place":
                    self.action_clients["discard"] = client
                rospy.loginfo(
                    "[Planner] Action server connected: %s (attempt %d)",
                    action_client_key,
                    attempt,
                )
                return True

            rospy.logwarn(
                "[Planner] Incomplete action connection; recreating client: %s "
                "(attempt %d, %.1fs remaining)",
                action_client_key,
                attempt,
                max(0.0, deadline - time.monotonic()),
            )

        return False

    def _execute_pick(self, action):
        """
        pick(robot,tomato,stem)
        """
        client = self.action_clients["pick"]
        inner = action.name[len("pick("):-1]
        parts = [p.strip() for p in inner.split(",")]
        tomato_name = parts[1]
        
        location_name = parts[2].title()

        if self.latest_scene is None:
            rospy.logwarn("[Planner] latest_scene is None")
            return False, [], []
        
        tomato = self.latest_scene.get_object_by_name(tomato_name)
        if tomato is None:
            rospy.logwarn(f"[Planner] Tomato '{tomato_name}' not found in scene")
            return False, [], []

        tomato_candidate = self.latest_scene.get_objects_at_location(location_name)
        if tomato not in tomato_candidate:
            if self.pomdp_planner.is_online:
                rospy.logwarn(f"[Planner] There is no tomato at {location_name}")
                return False, [], []
            rospy.logwarn(
                "[Planner][Offline] %s is not assigned to %s in Scene Graph; "
                "continuing with its stored 3D position",
                tomato_name,
                location_name,
            )

        rospy.loginfo(
            "[Planner] Pick target | tomato=%s | stem=%s | position=(x=%.4f, y=%.4f, z=%.4f)",
            tomato.name,
            location_name,
            tomato.position.x,
            tomato.position.y,
            tomato.position.z,
        )

        goal = PickTomatoGoal()
        goal.robot_name = self.robot_name
        goal.tomato_name = tomato.name
        goal.tomato_position.x = tomato.position.x
        goal.tomato_position.y = tomato.position.y
        goal.tomato_position.z = tomato.position.z

        client.send_goal(goal)
        client.wait_for_result()

        state = client.get_state()

        if state == actionlib.GoalStatus.SUCCEEDED:
            holding_fact = f"holding({self.robot_name},{tomato.name})"
            handempty_fact = f"handempty({self.robot_name})"
            rospy.loginfo(
                "[Planner] pick succeeded; forcing observation: %s",
                holding_fact,
            )
            self.holdings = tomato
            add_list = [
                holding_fact,
                f"holded({tomato.name},{self.robot_name})",
            ]
            del_list = [
                handempty_fact,
                f"at({tomato.name},{location_name})",
            ]
            return True, del_list, add_list

        rospy.logwarn(f"[Planner] pick failed, state={state}")
        return False, [], []

    def _execute_scan(self, action):
        client = self.action_clients["scan"]

        inner = action.name[len("scan("):-1]
        parts = [p.strip() for p in inner.split(",")]
        tomato_name = parts[1]
        
        robot = self.latest_scene.get_robot_by_name(self.robot_name)
        tomato = robot.get_gripper_object()
        
        request = TriggerSensorScanRequest()
        request.robot_name = self.robot_name
        request.target_location = ""
        request.target_object = tomato_name

        goal = ScanTomatoGoal()
        goal.robot_name = self.robot_name

        if self.start_scan_client is not None:
            self.start_scan_client(request)
            rospy.loginfo("[Planner] Start single tomato scan...")

        client.send_goal(goal)
        rospy.loginfo("[Planner] Running scanning action...")
        client.wait_for_result()

        scan_observation = None
        if self.end_scan_client is not None:
            scan_observation = self.end_scan_client(request)
            rospy.loginfo("[Planner] End single tomato scan...")

        state = client.get_state()
        result = client.get_result()

        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("[Planner] scan success")
            add_list = [f"scanned({tomato_name})"]
            if scan_observation is not None and scan_observation.success:
                add_list.extend(list(getattr(scan_observation, "facts", [])))
            del_list = []
            return True, del_list, add_list

        rospy.logwarn(f"[Planner] scan failed, state={state}")
        return False, [], []

    def _execute_place(self, action):
        client = self.action_clients["place"]

        inner = action.name[len("place("):-1]
        parts = [p.strip() for p in inner.split(",")]
        tomato_name = parts[1]

        if self.latest_scene is None:
            rospy.logwarn("[Planner] latest_scene is None")
            return False, [], []

        # robot = self.latest_scene.get_robot_by_name(self.robot_name)
        # tomato = robot.get_gripper_object()

        goal = PlaceTomatoGoal()
        goal.robot_name = self.robot_name
        goal.tomato_name = tomato_name
        goal.tomato_class = str(0)

        client.send_goal(goal)
        client.wait_for_result()

        state = client.get_state()
        result = client.get_result()

        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("[Planner] place success")
            add_list = [f"handempty({self.robot_name})",f"loaded({tomato_name},{self.robot_name})"]
            del_list = [f"holding({self.robot_name},{tomato_name})",f"holded({tomato_name},{self.robot_name})"]
            return True, del_list, add_list

        rospy.logwarn(f"[Planner] place failed, state={state}")
        return False, [], []

    def _execute_discard(self, action):
        client = self.action_clients["discard"]

        inner = action.name[len("discard("):-1]
        parts = [p.strip() for p in inner.split(",")]
        tomato_name = parts[1]

        if self.latest_scene is None:
            rospy.logwarn("[Planner] latest_scene is None")
            return False, [], []

        # robot = self.latest_scene.get_robot_by_name(self.robot_name)
        # tomato = robot.get_gripper_object()

        goal = PlaceTomatoGoal()
        goal.robot_name = self.robot_name
        goal.tomato_name = tomato_name
        goal.tomato_class = str(2)

        client.send_goal(goal)
        client.wait_for_result()

        state = client.get_state()
        result = client.get_result()

        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("[Planner] discard success")
            add_list = [f"handempty({self.robot_name})",f"discarded({tomato_name})"]
            del_list = [f"holding({self.robot_name},{tomato_name})",f"holded({tomato_name},{self.robot_name})"]
            return True, del_list, add_list

        rospy.logwarn(f"[Planner] discard failed, state={state}")
        return False, [], []

    def _execute_prepare_nav(self, action):
        client = self.action_clients["prepare_nav"]

        goal = PrepareNavigationGoal()
        goal.robot_name = self.robot_name

        client.send_goal(goal)
        client.wait_for_result()

        state = client.get_state()
        result = client.get_result()

        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("[Planner] prepare_nav success")
            return True, [], [f"navprepared({self.robot_name})"]

        rospy.logwarn(f"[Planner] prepare_nav failed, state={state}")
        return False, [], []

    def should_stop(self):
        with self.lock:
            return self.stop_requested

    def wait_while_paused(self):
        logged = False
        while not rospy.is_shutdown():
            with self.lock:
                if self.stop_requested:
                    return False
                paused = self.pause_requested
            if not paused:
                if logged:
                    rospy.loginfo("Tomato planning resumed")
                return True
            if not logged:
                rospy.loginfo("Tomato planning paused before next planning step")
                logged = True
            rospy.sleep(0.1)
        return False

    def stop_callback(self, _request):
        with self.lock:
            if not self.running:
                return TriggerResponse(success=False, message="planner is not running")
            self.stop_requested = True
            self.pause_requested = False
        for client in self.action_clients.values():
            if client is not None:
                client.cancel_all_goals()
        return TriggerResponse(success=True, message="planner stop requested")

    def pause_callback(self, _request):
        with self.lock:
            if not self.running:
                return TriggerResponse(success=False, message="planner is not running")
            self.pause_requested = True
        return TriggerResponse(success=True, message="planner pause requested")

    def resume_callback(self, _request):
        with self.lock:
            if not self.running:
                return TriggerResponse(success=False, message="planner is not running")
            self.pause_requested = False
        return TriggerResponse(success=True, message="planner resumed")

