#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
MotionPlanner - MoveIt based motion planning logic

MoveIt related logic encapsulation:
- Scene setup (basket, collision objects)
- Trajectory generation (linear interpolation)
- Joint waypoint execution

Python 2/3 Compatible
"""
from __future__ import print_function, division, absolute_import

import sys
import numpy as np
from copy import deepcopy

import moveit_commander
from moveit_msgs.msg import RobotTrajectory, CollisionObject, AttachedCollisionObject
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped
from rci_action_manager.msg import MoveitOptimalStatus  # [HQP] add

import threading  # [HQP] add
import rospy


class MotionPlanner:
    """MoveIt based motion planning class"""

    def __init__(self, group_name="manipulator"):
        """
        MoveIt initialization

        Args:
            group_name: MoveIt planning group name
        """
        moveit_commander.roscpp_initialize(sys.argv)

        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()
        self.move_group = moveit_commander.MoveGroupCommander(group_name, wait_for_servers=30.0)

        self.joint_names = self.move_group.get_active_joints()
        self.group_name = group_name
        self.time_step = 2.0
        self.retry_num = 30

        # [HQP] add: planned trajectory -> HQP (rci_action_manager moveit_optimal) instead of move_group.execute
        self.hqp_joint_names = rospy.get_param("~hqp_joint_names", [
            "ur5e_shoulder_pan_joint", "ur5e_shoulder_lift_joint", "ur5e_elbow_joint",
            "ur5e_wrist_1_joint", "ur5e_wrist_2_joint", "ur5e_wrist_3_joint"])
        self.hqp_start_retry = int(rospy.get_param("~hqp_start_retry", 3))
        # [BRL_lab] add: lab MoveIt config splits paths into more points; default False keeps BRL behavior
        self.ignore_max_points = bool(rospy.get_param("~ignore_max_points", False))
        # [BRL_lab] add: run visual-servoing Cartesian steps through HQP like every other motion
        # (default True; False = BRL move_group.execute, needs a move_group with execution)
        self.cartesian_via_hqp = bool(rospy.get_param("~cartesian_via_hqp", True))
        # [BRL_lab] add: normalize pose-goal quaternions (pick pose |q|=0.9998 -> IK never converges in the goal sampler)
        self.normalize_pose_goals = bool(rospy.get_param("~normalize_pose_goals", False))
        # [BRL_lab] add: visual servoing solved by HQP (brl_hqp_servo.py action) instead of the step VS
        self.hqp_visual_servoing = bool(rospy.get_param("~hqp_visual_servoing", False))
        self.hqp_status_lock = threading.Lock()
        self.hqp_status = None
        self.hqp_trajectory_pub = rospy.Publisher(
            rospy.get_param("~hqp_trajectory_topic", "/rci_action_manager/moveit_plan"),
            JointTrajectory, queue_size=1)
        rospy.Subscriber(
            rospy.get_param("~hqp_status_topic", "/moveit_optimal_controller/status"),
            MoveitOptimalStatus, self._hqp_status_cb, queue_size=1)

        rospy.loginfo("[MotionPlanner] Initialized with group '%s'" % group_name)

    def setup_scene(self, reference_frame="ur5e_base_link"):
        """
        MoveIt Scene setup (basket, collision objects)

        Args:
            reference_frame: Reference frame
        """
        self._add_basket_to_scene(reference_frame)
        self._attach_tip_boxes()
        rospy.loginfo("[MotionPlanner] Scene setup complete")

    def _add_basket_to_scene(self, reference_frame):
        """Add basket collision objects"""
        basket_length = 0.28
        basket_width = 0.36
        basket_height = 0.19
        wall_thickness = 0.01

        sub_basket_length = 0.14
        sub_basket_width = 0.01
        sub_basket_height = 0.08

        # Basket bottom
        basket_bottom_pose = PoseStamped()
        basket_bottom_pose.header.frame_id = reference_frame
        basket_bottom_pose.pose.position.x = -0.32
        basket_bottom_pose.pose.position.y = 0.0
        basket_bottom_pose.pose.position.z = 0.05
        basket_bottom_pose.pose.orientation.w = 1.0
        self.scene.add_box("basket_bottom", basket_bottom_pose,
                           size=(basket_length, basket_width + 0.2, wall_thickness))

        # Right wall
        front_wall_pose = PoseStamped()
        front_wall_pose.header.frame_id = reference_frame
        front_wall_pose.pose.position.x = -0.32
        front_wall_pose.pose.position.y = -basket_width / 2 + wall_thickness / 2 - 0.1
        front_wall_pose.pose.position.z = basket_height / 2
        front_wall_pose.pose.orientation.w = 1.0
        self.scene.add_box("basket_right_wall", front_wall_pose,
                           size=(basket_length, wall_thickness, basket_height))

        # Left wall
        back_wall_pose = PoseStamped()
        back_wall_pose.header.frame_id = reference_frame
        back_wall_pose.pose.position.x = -0.32
        back_wall_pose.pose.position.y = basket_width / 2 - wall_thickness / 2 + 0.1
        back_wall_pose.pose.position.z = basket_height / 2
        back_wall_pose.pose.orientation.w = 1.0
        self.scene.add_box("basket_left_wall", back_wall_pose,
                           size=(basket_length, wall_thickness, basket_height))

        # Back wall
        left_wall_pose = PoseStamped()
        left_wall_pose.header.frame_id = reference_frame
        left_wall_pose.pose.position.x = -basket_length / 2 + wall_thickness / 2 - 0.32
        left_wall_pose.pose.position.y = 0.0
        left_wall_pose.pose.position.z = basket_height / 2
        left_wall_pose.pose.orientation.w = 1.0
        self.scene.add_box("basket_back_wall", left_wall_pose,
                           size=(wall_thickness, basket_width + 0.2, basket_height))

        # Sub basket left
        sub_left_pose = PoseStamped()
        sub_left_pose.header.frame_id = reference_frame
        sub_left_pose.pose.position.x = -0.32
        sub_left_pose.pose.position.y = basket_width / 2 - wall_thickness / 2 + 0.1
        sub_left_pose.pose.position.z = basket_height + sub_basket_height / 2
        sub_left_pose.pose.orientation.w = 1.0
        self.scene.add_box("sub_basket_left", sub_left_pose,
                           size=(sub_basket_length, sub_basket_width, sub_basket_height))

        # Sub basket right
        sub_right_pose = PoseStamped()
        sub_right_pose.header.frame_id = reference_frame
        sub_right_pose.pose.position.x = -0.32
        sub_right_pose.pose.position.y = -basket_width / 2 + wall_thickness / 2 - 0.1
        sub_right_pose.pose.position.z = basket_height + sub_basket_height / 2
        sub_right_pose.pose.orientation.w = 1.0
        self.scene.add_box("sub_basket_right", sub_right_pose,
                           size=(sub_basket_length, sub_basket_width, sub_basket_height))

    def _attach_tip_boxes(self):
        """Attach gripper tip collision boxes"""
        rospy.sleep(1.0)  # Wait for PlanningScene

        # Tip 1
        self._attach_tip_box(
            object_id="tip1_box",
            link_name="gripper_finger1_finger_tip_link"
        )

        # Tip 2
        self._attach_tip_box(
            object_id="tip2_box",
            link_name="gripper_finger2_finger_tip_link"
        )

    def _attach_tip_box(self, object_id, link_name):
        """Attach individual tip box"""
        box_size = [0.1, 0.025, 0.06]

        pose = PoseStamped()
        pose.header.frame_id = link_name
        pose.pose.position.x = 0.05 + box_size[0] / 2
        pose.pose.position.y = -0.015 - 0.025 / 2
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = box_size

        collision_object = CollisionObject()
        collision_object.id = object_id
        collision_object.header.frame_id = link_name
        collision_object.primitives = [primitive]
        collision_object.primitive_poses = [pose.pose]
        collision_object.operation = CollisionObject.ADD

        attached_object = AttachedCollisionObject()
        attached_object.link_name = link_name
        attached_object.object = collision_object
        attached_object.touch_links = self.robot.get_link_names(group=self.group_name)

        self.scene.attach_object(attached_object)
        rospy.loginfo("[MotionPlanner] Attached %s to %s" % (object_id, link_name))

    def generate_linear_trajectory(self, waypoints, steps=10):
        """
        Generate trajectory using linear interpolation

        Args:
            waypoints: List of joint positions (each waypoint is a list of joint angles)
            steps: Number of interpolation steps between waypoints

        Returns:
            Generated RobotTrajectory
        """
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names

        time_from_start = 0.0
        time_scaling_factor = 1.0

        for i in range(len(waypoints) - 1):
            start_wp = np.array(waypoints[i])
            end_wp = np.array(waypoints[i + 1])

            for j in range(steps):
                alpha = float(j) / float(steps)
                interpolated_wp = (1 - alpha) * start_wp + alpha * end_wp

                point = JointTrajectoryPoint()
                point.positions = interpolated_wp.tolist()
                point.time_from_start = rospy.Duration(time_from_start)
                trajectory.points.append(point)
                time_from_start += (self.time_step / steps) * time_scaling_factor

        # 마지막 waypoint 추가
        final_point = JointTrajectoryPoint()
        final_point.positions = waypoints[-1]
        final_point.time_from_start = rospy.Duration(time_from_start)
        trajectory.points.append(final_point)

        robot_trajectory = RobotTrajectory()
        robot_trajectory.joint_trajectory = trajectory
        return robot_trajectory

    def plan_to_pose(self, target_pose):
        """
        Generate plan to pose target

        Args:
            target_pose: Target pose

        Returns:
            (plan, success) tuple
        """
        if self.normalize_pose_goals:  # [BRL_lab] add: same rotation, unit length
            q = target_pose.pose.orientation if hasattr(target_pose, "pose") else target_pose.orientation
            n = np.sqrt(q.x ** 2 + q.y ** 2 + q.z ** 2 + q.w ** 2)
            if n > 0:
                q.x, q.y, q.z, q.w = q.x / n, q.y / n, q.z / n, q.w / n
        self.move_group.set_pose_target(target_pose)
        plan = self.move_group.plan()
        if isinstance(plan, tuple):  # Noetic add: plan() -> (success, trajectory, planning_time, error_code)
            plan = plan[1]

        success = bool(plan.joint_trajectory.points)
        return plan, success

    def plan_to_joint_values(self, joint_values):
        """
        Generate plan to joint values

        Args:
            joint_values: Target joint angles list

        Returns:
            (plan, success) tuple
        """
        self.move_group.set_joint_value_target(joint_values)
        plan = self.move_group.plan()
        if isinstance(plan, tuple):  # Noetic add: plan() -> (success, trajectory, planning_time, error_code)
            plan = plan[1]

        success = bool(plan.joint_trajectory.points)
        return plan, success

    def _plan_until_found(self, planner_function, planner_args=(),
                          planner_kwargs=None, max_points=20,
                          preempt_check_cb=None, max_attempts=10):
        """Match tele-op planning retries without repeating execution."""
        if planner_kwargs is None:
            planner_kwargs = {}
        attempt = 0
        while not rospy.is_shutdown() and attempt < max_attempts:
            if preempt_check_cb and preempt_check_cb():
                return None, False
            attempt += 1
            self.move_group.set_start_state_to_current_state()
            plan, success = planner_function(*planner_args, **planner_kwargs)
            if success:
                try:
                    point_count = len(plan.joint_trajectory.points)
                except Exception:
                    point_count = 0
                if point_count <= max_points or self.ignore_max_points:  # [BRL_lab] add: or ignore_max_points
                    return plan, True
                rospy.logwarn(
                    "[MotionPlanner] Plan has %d points (limit=%d); replanning"
                    % (point_count, max_points)
                )
            else:
                rospy.logwarn(
                    "[MotionPlanner] No motion plan found; retrying (attempt %d)"
                    % attempt
                )
            rospy.sleep(0.1)
        if not rospy.is_shutdown():
            rospy.logerr(
                "[MotionPlanner] No acceptable motion plan after %d attempts"
                % max_attempts
            )
        return None, False

    def execute(self, plan, wait=True):
        """
        Execute plan (synchronous)

        Args:
            plan: Plan to execute (RobotTrajectory or plan object)
            wait: Whether to wait for execution completion

        Returns:
            Execution success
        """
        try:
            success = self.move_group.execute(plan, wait=wait)
            return success
        except Exception as e:
            rospy.logerr("[MotionPlanner] Execution failed: %s" % str(e))
            return False

    # [HQP] Original MoveIt execution (move_group.execute -> robot directly) - commented out, replaced below
    # def execute_with_preempt(self, plan, preempt_check_cb=None, check_interval=0.05):
    #     """
    #     Execute plan with preempt capability (async + polling)
    #
    #     Args:
    #         plan: Plan to execute (RobotTrajectory or plan object)
    #         preempt_check_cb: Preempt check callback () -> bool
    #         check_interval: Preempt check interval (seconds)
    #
    #     Returns:
    #         (success, preempted) tuple
    #         - success: Execution success
    #         - preempted: Whether preempted
    #     """
    #     try:
    #         # Extract trajectory info
    #         if hasattr(plan, 'joint_trajectory'):
    #             trajectory = plan.joint_trajectory
    #         else:
    #             trajectory = plan
    #
    #         if not trajectory.points:
    #             rospy.logwarn("[MotionPlanner] Empty trajectory")
    #             return False, False
    #
    #         # Calculate expected duration
    #         expected_duration = trajectory.points[-1].time_from_start.to_sec()
    #         rospy.loginfo("[MotionPlanner] Trajectory has %d points, expected_duration=%.2fs" %
    #                      (len(trajectory.points), expected_duration))
    #
    #         # Target joint values (last point)
    #         target_joints = list(trajectory.points[-1].positions)
    #         rospy.loginfo("[MotionPlanner] Target joints: %s" % str(target_joints))
    #
    #         # Start async execution
    #         rospy.loginfo("[MotionPlanner] Calling execute(wait=False)...")
    #         self.move_group.execute(plan, wait=False)
    #         rospy.loginfo("[MotionPlanner] execute() returned, starting poll loop")
    #
    #         start_time = rospy.Time.now()
    #         timeout = expected_duration + 10.0  # Add margin
    #
    #         while not rospy.is_shutdown():
    #             # 1. Preempt check
    #             if preempt_check_cb and preempt_check_cb():
    #                 rospy.loginfo("[MotionPlanner] Preempt requested, stopping execution")
    #                 self.stop()
    #                 return False, True  # preempted
    #
    #             # 2. Timeout check
    #             elapsed = (rospy.Time.now() - start_time).to_sec()
    #             if elapsed > timeout:
    #                 rospy.logwarn("[MotionPlanner] Execution timeout")
    #                 self.stop()
    #                 return False, False
    #
    #             # 3. Completion check (target reached)
    #             current_joints = self.move_group.get_current_joint_values()
    #             if self._is_at_target(current_joints, target_joints):
    #                 rospy.loginfo("[MotionPlanner] Target reached")
    #                 return True, False
    #
    #             rospy.sleep(check_interval)
    #
    #         # ROS shutdown
    #         return False, False
    #
    #     except Exception as e:
    #         rospy.logerr("[MotionPlanner] execute_with_preempt failed: %s" % str(e))
    #         self.stop()
    #         return False, False

    def execute_with_preempt(self, plan, preempt_check_cb=None, check_interval=0.05):
        """
        [HQP] Send planned trajectory to HQP (always-running low-level controller)

        Same signature/return as the original: (success, preempted)
        If admittance moved the robot after planning (HQP START_MISMATCH),
        replan from the measured state to the same final joint goal and resend.
        """
        if hasattr(plan, 'joint_trajectory'):
            trajectory = plan.joint_trajectory
        else:
            trajectory = plan

        if not trajectory.points:
            rospy.logwarn("[MotionPlanner] Empty trajectory")
            return False, False

        for attempt in range(self.hqp_start_retry + 1):
            success, preempted, reason = self._hqp_send_and_wait(
                trajectory, preempt_check_cb, check_interval)
            if success or preempted or "first point differs" not in reason:
                return success, preempted

            rospy.logwarn("[MotionPlanner][HQP] Robot moved since planning; replanning from current state (%d/%d)" %
                          (attempt + 1, self.hqp_start_retry))
            replan, replan_success = self.plan_to_joint_values(list(trajectory.points[-1].positions))
            if not replan_success:
                return False, False
            trajectory = replan.joint_trajectory

        return False, False

    def _hqp_send_and_wait(self, trajectory, preempt_check_cb=None, check_interval=0.05):
        """[HQP] Publish one trajectory to HQP and wait for its result -> (success, preempted, reason)"""
        try:
            rospy.loginfo("[MotionPlanner][HQP] Trajectory has %d points, planned_duration=%.2fs" %
                          (len(trajectory.points), trajectory.points[-1].time_from_start.to_sec()))

            # HQP status is needed to identify the result of our trajectory
            deadline = rospy.Time.now() + rospy.Duration(2.0)
            while not rospy.is_shutdown() and self._get_hqp_status() is None:
                if rospy.Time.now() > deadline:
                    rospy.logerr("[MotionPlanner][HQP] No HQP status; is moveit_optimal controller running?")
                    return False, False, "no HQP status"
                rospy.sleep(check_interval)
            status = self._get_hqp_status()
            last_id = max(status.trajectory_id, status.result_trajectory_id)

            msg = deepcopy(trajectory)
            msg.header.stamp = rospy.Time(0)  # start at next HQP control tick
            self.hqp_trajectory_pub.publish(msg)

            # HQP may retime the path (geometric tracking), so completion/abort is decided by HQP.
            # Here only check that HQP accepts the trajectory and keeps publishing status.
            start_time = rospy.Time.now()
            accept_timeout = 2.0
            status_timeout = 1.0

            while not rospy.is_shutdown():
                # 1. Preempt check
                if preempt_check_cb and preempt_check_cb():
                    rospy.loginfo("[MotionPlanner][HQP] Preempt requested, holding current position")
                    self._hqp_hold()
                    return False, True, "preempted"

                # 2. HQP result of our trajectory (HQP assigns ids after last_id)
                status = self._get_hqp_status()
                if status.result_trajectory_id > last_id and status.last_result != "NONE":
                    if status.last_result == "SUCCEEDED":
                        rospy.loginfo("[MotionPlanner][HQP] Target reached (rms=%.4f, max=%.4f rad)" %
                                      (status.tracking_rms, status.tracking_max))
                        return True, False, status.reason
                    rospy.logwarn("[MotionPlanner][HQP] Trajectory %s: %s" %
                                  (status.last_result, status.reason))
                    return False, False, status.reason

                # 3. HQP acceptance / liveness check
                now = rospy.Time.now()
                if status.trajectory_id <= last_id and (now - start_time).to_sec() > accept_timeout:
                    rospy.logwarn("[MotionPlanner][HQP] Trajectory not accepted by HQP")
                    return False, False, "not accepted"
                if (now - self.hqp_status_stamp).to_sec() > status_timeout:
                    rospy.logwarn("[MotionPlanner][HQP] HQP status stale, holding")
                    self._hqp_hold()
                    return False, False, "status stale"

                rospy.sleep(check_interval)

            # ROS shutdown
            return False, False, "shutdown"

        except Exception as e:
            rospy.logerr("[MotionPlanner][HQP] send failed: %s" % str(e))
            return False, False, str(e)

    def _hqp_status_cb(self, msg):
        """[HQP] Store latest HQP status"""
        with self.hqp_status_lock:
            self.hqp_status = msg
            self.hqp_status_stamp = rospy.Time.now()

    def _get_hqp_status(self):
        with self.hqp_status_lock:
            return self.hqp_status

    def _hqp_hold(self):
        """[HQP] Replace active trajectory with a hold at the measured position (HQP blends to rest)"""
        status = self._get_hqp_status()
        if status is None:
            return
        trajectory = JointTrajectory()
        trajectory.joint_names = list(self.hqp_joint_names)
        for t in (0.0, 0.5):
            point = JointTrajectoryPoint()
            point.positions = list(status.actual)
            point.velocities = [0.0] * len(status.actual)
            point.accelerations = [0.0] * len(status.actual)
            point.time_from_start = rospy.Duration(t)
            trajectory.points.append(point)
        self.hqp_trajectory_pub.publish(trajectory)

    def _is_at_target(self, current, target, tolerance=0.01):
        """
        Check if current joint position reached target

        Args:
            current: Current joint values
            target: Target joint values
            tolerance: Tolerance (radian)

        Returns:
            Whether reached
        """
        if len(current) != len(target):
            return False

        for c, t in zip(current, target):
            if abs(c - t) > tolerance:
                return False

        return True

    # [HQP] Original MoveIt execution (move_group.execute -> robot directly) - commented out, replaced below
    # def _execute_scan_synchronously(self, plan, preempt_check_cb=None):
    #     """Execute a scan trajectory synchronously, matching teleop behavior."""
    #     if preempt_check_cb and preempt_check_cb():
    #         self.stop()
    #         return False, True
    #
    #     try:
    #         success = bool(self.move_group.execute(plan, wait=True))
    #         self.move_group.stop()
    #         self.move_group.clear_pose_targets()
    #
    #         if preempt_check_cb and preempt_check_cb():
    #             return False, True
    #         return success, False
    #     except Exception as error:
    #         rospy.logerr("[MotionPlanner] Scan execution failed: %s" % str(error))
    #         self.stop()
    #         return False, False

    def _execute_scan_synchronously(self, plan, preempt_check_cb=None):
        """[HQP] Execute a scan trajectory through HQP (same return as original)."""
        if preempt_check_cb and preempt_check_cb():
            return False, True
        return self.execute_with_preempt(plan, preempt_check_cb)

    def _run_hqp_visual_servoing(self, preempt_check_cb=None):  # [BRL_lab] add
        """HQP visual servoing action (servo + gripper close); returns (success, message)."""
        import actionlib
        from brl_interfaces.msg import VisualServoingActionAction, VisualServoingActionGoal
        client = actionlib.SimpleActionClient("/brl_robot/hqp_visual_servoing", VisualServoingActionAction)
        if not client.wait_for_server(rospy.Duration(5.0)):
            return False, "HQP visual servoing server not available"
        client.send_goal(VisualServoingActionGoal(on=True))
        while not client.wait_for_result(rospy.Duration(0.1)):
            if preempt_check_cb and preempt_check_cb():
                client.cancel_goal()
                client.wait_for_result(rospy.Duration(5.0))
                return False, "Preempted by client"
            if rospy.is_shutdown():
                return False, "shutdown"
        result = client.get_result()
        return (bool(result and result.success),
                "HQP visual servoing: %s" % (result.message if result else "no result"))

    def stop(self):
        """Stop current execution"""
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def get_current_joint_values(self):
        """Return current joint values"""
        return self.move_group.get_current_joint_values()

    def get_current_pose(self):
        """Return current end-effector pose"""
        return self.move_group.get_current_pose()

    def move_cartesian(self, x, y, z, roll=0, pitch=0, yaw=0, max_retries=3):
        """
        Cartesian incremental movement (for visual servoing)

        Args:
            x, y, z: Position delta (meters)
            roll, pitch, yaw: Rotation delta (radians)
            max_retries: Maximum retry attempts

        Returns:
            (success, message) tuple
        """
        from geometry_msgs.msg import Pose
        from tf.transformations import euler_from_quaternion, quaternion_from_euler

        for attempt in range(max_retries):
            waypoints = []

            # Get current pose
            current_pose = self.move_group.get_current_pose().pose
            if not current_pose:
                return False, "Failed to get current pose"

            waypoints.append(current_pose)

            # Calculate target pose
            target_pose = Pose()
            target_pose.position.x = current_pose.position.x + x
            target_pose.position.y = current_pose.position.y + y
            target_pose.position.z = current_pose.position.z + z

            # Handle rotation
            if roll != 0 or pitch != 0 or yaw != 0:
                quaternion = [
                    current_pose.orientation.x,
                    current_pose.orientation.y,
                    current_pose.orientation.z,
                    current_pose.orientation.w
                ]
                current_roll, current_pitch, current_yaw = euler_from_quaternion(quaternion)

                new_roll = current_roll + roll
                new_pitch = current_pitch + pitch
                new_yaw = current_yaw + yaw

                new_quaternion = quaternion_from_euler(new_roll, new_pitch, new_yaw)

                target_pose.orientation.x = new_quaternion[0]
                target_pose.orientation.y = new_quaternion[1]
                target_pose.orientation.z = new_quaternion[2]
                target_pose.orientation.w = new_quaternion[3]
            else:
                target_pose.orientation = current_pose.orientation

            waypoints.append(target_pose)

            # Compute Cartesian path
            try:
                (plan, fraction) = self.move_group.compute_cartesian_path(
                    waypoints,
                    0.005,  # eef_step (5mm resolution)
                    0.0     # jump_threshold
                )
            except TypeError:
                # [BRL_lab] add: MoveIt >= 1.1.14 Python API has no jump_threshold argument
                # (0.0 lands on avoid_collisions:bool and raises). Same call without it:
                # jump check off as before, avoid_collisions default True as before.
                (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.005)

            # Remove first point (current position)
            if plan.joint_trajectory.points:
                plan.joint_trajectory.points.pop(0)

            # Safety check - reject if too many points (wrong path)
            if len(plan.joint_trajectory.points) > 30:
                rospy.logwarn("[MotionPlanner] Cartesian path too long, retrying...")
                continue

            if fraction < 1.0:
                rospy.logwarn("[MotionPlanner] Cartesian path incomplete: %.2f" % fraction)

            # Execute
            if self.cartesian_via_hqp:  # [BRL_lab] add: same HQP path as execute_with_preempt
                success, _preempted, _reason = self._hqp_send_and_wait(plan.joint_trajectory)
            else:
                success = self.move_group.execute(plan, wait=True)
            if success:
                return True, "Cartesian movement completed"

        return False, "Cartesian movement failed after %d retries" % max_retries

    # ==================== High-Level Action Methods ====================

    def pick_tomato(self, tomato_position, vs_instance, vs_server, gripper_pub,
                    feedback_cb=None, preempt_check_cb=None):
        """
        Full tomato pick logic

        Args:
            tomato_position: Tomato position (x, y, z)
            vs_instance: VisualServoing instance
            vs_server: VisualServoing action server (for preempt check)
            gripper_pub: Gripper command publisher
            feedback_cb: Feedback callback (progress, status) -> None
            preempt_check_cb: Preempt check callback () -> bool

        Returns:
            (success, message) tuple
        """
        # 0. Move to base pick pose
        rospy.loginfo("[DEBUG][MotionPlanner] Moving to base-pick position")

        pre_pick_waypoints = [
            [90, -23, -125, -33, 87, -88], # changmin add
            # [90, -46, -100, -33, 87, -88], # changmin add
            # [87, -60, -87, -32, 90, -88] # changmin add
        ]

        for waypoint in pre_pick_waypoints:
            joint_values = [np.deg2rad(v) for v in waypoint]
            # plan, success = self.plan_to_joint_values(joint_values)
            plan, success = self.replanning_with_retry(
                planner_function=self.plan_to_joint_values,
                planner_args=(joint_values,),
                max_points=20,
            )
            

            if not success:
                return False, "Post-pick planning failed"

            exec_success, preempted = self.execute_with_preempt(plan, preempt_check_cb)
            if preempted:
                return False, "Preempted by client"
            if not exec_success:
                return False, "Post-pick movement failed"


        # 1. Move to pre-pick pose
        rospy.loginfo("[DEBUG][MotionPlanner] Moving to pre-pick position")
        target_pose = PoseStamped()
        target_pose.header.frame_id = "robot_base_footprint"
        target_pose.pose.position.x = tomato_position.x
        target_pose.pose.position.y = tomato_position.y + 0.15
        target_pose.pose.position.z = tomato_position.z
        target_pose.pose.orientation.x = -0.500
        target_pose.pose.orientation.y = 0.516
        target_pose.pose.orientation.z = -0.481
        target_pose.pose.orientation.w = -0.502

        # logging_target_pose
        # plan, success = self.plan_to_pose(target_pose)
        plan, success = self.replanning_with_retry(
                planner_function=self.plan_to_pose,
                planner_args=(target_pose,),
                max_points=18,
            )

        if not success:
            self.stop()
            return False, "Failed to plan to pre-pick pose"

        # Execute with preempt
        exec_success, preempted = self.execute_with_preempt(plan, preempt_check_cb)
        if preempted:
            return False, "Preempted by client"
        if not exec_success:
            self.stop()
            return False, "Failed to execute pre-pick movement"

        if feedback_cb:
            feedback_cb(30, "Pre-pick pose reached, starting visual servoing")



        # [BRL_lab] re-enabled; skipped when vs_instance is None (default = [HQP] disabled as before)
        if self.hqp_visual_servoing:  # [BRL_lab] add: HQP servo replaces the step VS below
            rospy.loginfo("[MotionPlanner] Start HQP visual servoing")
            success, message = self._run_hqp_visual_servoing(preempt_check_cb)
            rospy.loginfo("[MotionPlanner] %s" % message)
            if not success:
                return False, message
            vs_instance = None
        if vs_instance is not None:
            # 2. Run Visual Servoing (directly using instance)
            rospy.loginfo("[DEBUG][MotionPlanner] Start to run Visual Servoing")
            vs_instance.reset()

            rate = rospy.Rate(30)  # 30Hz
            while not rospy.is_shutdown() and not vs_instance.is_done():
                # Preempt check
                if preempt_check_cb and preempt_check_cb():
                    vs_instance.cleanup()
                    rospy.loginfo("[MotionPlanner] Visual servoing cancelled by preempt")
                    return False, "Preempted by client"

                # State machine step
                vs_instance.step()
                rate.sleep()

            vs_instance.cleanup()

            if not vs_instance.is_done():
                rospy.logerr("[MotionPlanner] Visual servoing failed")
                return False, "Visual servoing failed"

        if feedback_cb:
            feedback_cb(60, "Visual servoing succeeded, moving to post-pick")

        # 3. Post-pick movement
        rospy.loginfo("[MotionPlanner] Moving to post-pick position")

        post_pick_waypoints = [
            [98, -46, -100, -33, 87, -88], # changmin add
            [109, -38, -122, -22, 68, -90],
            [54, -90, -68, -111, 91, 7]
        ]

        for waypoint in post_pick_waypoints:
            joint_values = [np.deg2rad(v) for v in waypoint]
            # plan, success = self.plan_to_joint_values(joint_values)
            plan, success = self.replanning_with_retry(
                planner_function=self.plan_to_joint_values,
                planner_args=(joint_values,),
                max_points=25,
            )

            if not success:
                return False, "Post-pick planning failed"

            exec_success, preempted = self.execute_with_preempt(plan, preempt_check_cb)
            if preempted:
                return False, "Preempted by client"
            if not exec_success:
                return False, "Post-pick movement failed"

        if feedback_cb:
            feedback_cb(100, "Pick completed")

        self.stop()
        return True, "Pick completed successfully"

    def scan_tomato(self, feedback_cb=None, preempt_check_cb=None):
        """
        Tomato rottenness scan logic

        Args:
            feedback_cb: Feedback callback (progress, status) -> None
            preempt_check_cb: Preempt check callback () -> bool

        Returns:
            (success, message) tuple
        """
        # Scan waypoints (degree)
        scan_waypoints_deg = [
            [92, -96, -68, -153, 51, 61],
            [54, -90, -68, -111, 91, 7],
            [54, -93, -64, -120, 92, -173],
            [89, -94, -74, -150, 49, -121],
            [54, -93, -64, -120, 92, -173]
        ]
        waypoints = [[np.deg2rad(v) for v in pose] for pose in scan_waypoints_deg]

        # 첫 번째 waypoint로 이동
        # plan, success = self.plan_to_joint_values(waypoints[0])
        plan, success = self._plan_until_found(
            planner_function=self.plan_to_joint_values,
            planner_args=(waypoints[0],),
            max_points=30,
            preempt_check_cb=preempt_check_cb,
        )
        

        if not success:
            return False, "Initial waypoint planning failed"

        exec_success, preempted = self._execute_scan_synchronously(
            plan, preempt_check_cb
        )
        if preempted:
            return False, "Preempted by client"
        if not exec_success:
            return False, "Initial waypoint failed"

        if feedback_cb:
            feedback_cb(30, "Initial waypoint reached")

        # Linear trajectory 실행
        trajectory = self.generate_linear_trajectory(waypoints, steps=10)

        exec_success, preempted = self._execute_scan_synchronously(
            trajectory, preempt_check_cb
        )
        if preempted:
            return False, "Preempted by client"
        if not exec_success:
            return False, "Trajectory execution failed"

        if feedback_cb:
            feedback_cb(100, "Scan complete")

        return True, "Scan completed successfully"

    def place_tomato(self, tomato_class, gripper_pub,
                     feedback_cb=None, preempt_check_cb=None):
        """
        Tomato place logic

        Args:
            tomato_class: Tomato class ("0"=ripe, "1"=unripe, "2"=rotten)
            gripper_pub: Gripper command publisher
            feedback_cb: Feedback callback (progress, status) -> None
            preempt_check_cb: Preempt check callback () -> bool

        Returns:
            (success, message) tuple
        """
        from brl_interfaces.msg import GripperCmd

        # Select basket based on tomato class
        if tomato_class == "0":  # ripe
            joint_sequence_deg = [
                [24, -68, -90, -111, 91, -246],
                [24, -16, -120, -48, 89, -262], # changmin add way1
                [24, -16, -120, -48, 89, -95], # changmin add way1
                [90, -23, -125, -33, 87, -88], # final base pose
            ]
            drop_step = 0
        else:  # unripe or rotten
            joint_sequence_deg = [
                [202, -49, -139, -72, 93, -73],
                [205, -86, -158, -15, 92, -71],
                [202, -49, -139, -72, 93, -73],
                [90, -23, -125, -33, 87, -88], # final base pose
            ]
            drop_step = 1

        total_steps = len(joint_sequence_deg)

        for i, joint_angles in enumerate(joint_sequence_deg):
            joint_values = [np.deg2rad(v) for v in joint_angles]

            plan, success = self._plan_until_found(
                planner_function=self.plan_to_joint_values,
                planner_args=(joint_values,),
                max_points=25,
                preempt_check_cb=preempt_check_cb,
            )

            if not success:
                if preempt_check_cb and preempt_check_cb():
                    return False, "Preempted by client"
                return False, "Planning interrupted at step %d" % i

            exec_success, preempted = self.execute_with_preempt(plan, preempt_check_cb)
            if preempted:
                return False, "Preempted by client"
            if not exec_success:
                return False, "Movement failed at step %d" % i

            # Match the tele-op place sequence's class-specific drop step.
            if i == drop_step:
                msg = GripperCmd()
                msg.position = 0.08
                gripper_pub.publish(msg)
                rospy.sleep(1.5)

            if feedback_cb:
                progress = int((i + 1) * 100 / total_steps)
                feedback_cb(progress, "Step %d/%d done" % (i + 1, total_steps))

        return True, "Place completed"

    def prepare_navigation(self, feedback_cb=None, preempt_check_cb=None):
        """
        Prepare navigation (fold arm) logic

        Args:
            feedback_cb: Feedback callback (progress, status) -> None
            preempt_check_cb: Preempt check callback () -> bool

        Returns:
            (success, message) tuple
        """
        # Navigation-ready pose (radian)
        # joint_sequence_deg = [
        #     [24, -68, -90, -111, 91, -246],
        #     [25, -67, -131, -70, 91, -246],
        #     [24, -68, -90, -111, 91, -246],
        #     [24, -16, -120, -48, 89, -262], # changmin add way1
        #     [24, -16, -120, -48, 89, -95], # changmin add way1
        #     [98, -23, -125, -33, 87, -88], # final base pose
        # ]
        # total_steps = len(joint_sequence_deg)

        # for i, joint_angles in enumerate(joint_sequence_deg):
        #     joint_values = [np.deg2rad(v) for v in joint_angles]

        #     plan, success = self.plan_to_joint_values(joint_values)

        #     if not success:
        #         return False, "Planning failed at step %d" % i

        #     exec_success, preempted = self.execute_with_preempt(plan, preempt_check_cb)
        #     if preempted:
        #         return False, "Preempted by client"
        #     if not exec_success:
        #         return False, "Movement failed at step %d" % i

        #     if feedback_cb:
        #         progress = int((i + 1) * 100 / total_steps)
        #         feedback_cb(progress, "Step %d/%d done" % (i + 1, total_steps))

        joint_sequence_deg = [[90, -33, -140, -33, 87, -88],
                              [95, -18, -156, -65, 96, -88]]
        # joint_goal = [1.578, -0.237, -2.746, -1.171, 1.636, -1.461]


        for i, joint_angles in enumerate(joint_sequence_deg):
            joint_goal = [np.deg2rad(v) for v in joint_angles]
            # plan, success = self.plan_to_joint_values(joint_goal)
            # Changmin add
            plan, success = self.replanning_with_retry(
                planner_function=self.plan_to_joint_values,
                planner_args=(joint_goal,),
                max_points=25,
            )

            if feedback_cb:
                feedback_cb(0, "Planning trajectory")

            if not success:
                return False, "Planning failed"

            if feedback_cb:
                feedback_cb(50, "Executing trajectory")

            exec_success, preempted = self.execute_with_preempt(plan, preempt_check_cb)
            if preempted:
                return False, "Preempted by client"
            
            if not exec_success:
                return False, "Movement failed at step %d" % i

        self.stop()

        if feedback_cb:
            feedback_cb(100, "Prepare-navigation completed")

        return True, "Prepare-navigation completed"

    def _traj_len(self, plan):
        """Return number of trajectory points safely."""
        try:
            if hasattr(plan, "joint_trajectory"):
                return len(plan.joint_trajectory.points)
            else:
                return len(plan.points)
        except Exception:
            pass

        return 0
    
    # Changmin add
    def replanning_with_retry(self, planner_function, planner_args=(), planner_kwargs=None, max_points=20):
        """
        Docstring for replanning_with_retry
        
        :param self: Description
        :param planner_function: Description
        :param planner_args: Description
        :param planner_kwargs: Description
        
        plan, success = self.replanning_with_retry(
            planner_fn=self.plan_to_joint_values,
            planner_args=(target_pose,),
            max_points=120,
        )
        """
        
        if planner_kwargs is None:
            planner_kwargs = {}
        
        last_plan = None
        
        for attempt in range(1, self.retry_num):
            plan, success = planner_function(*planner_args, **planner_kwargs)
            last_plan = plan
            if not success:
                return last_plan, False

            npts = self._traj_len(plan)
    
            if npts > max_points and not self.ignore_max_points:  # [BRL_lab] add: and not ignore_max_points
                continue
            
            # ✅ acceptable plan
            # else:
                # print(f"[DUBUG][MotionPlanner] Plan Successed at retry: {attempt}")
            return plan, True
        
        return last_plan, False
