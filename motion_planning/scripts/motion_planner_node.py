#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Motion Planner Node (Merged) - ROS Action Servers for manipulation + Visual Servoing

Action Servers:
- /{robot_name}/pick_tomato          (PickTomatoAction)
- /{robot_name}/scan_tomato          (ScanTomatoAction)
- /{robot_name}/place_tomato         (PlaceTomatoAction)
- /{robot_name}/prepare_navigation   (PrepareNavigationAction)
- /{robot_name}/visual_servoing_action (VisualServoingActionAction)

Publishers:
- /{robot_name}/gripper/cmd (GripperCmd)

Subscribers:
- /{robot_name}/yolo_detection/camera2_best_target (Detection2D)

Python 2/3 Compatible
"""
from __future__ import print_function, division, absolute_import

import rospy
import actionlib
import time
import threading

from brl_interfaces.msg import GripperCmd, Detection2D
from robotiq_85_msgs.msg import GripperStat

from brl_interfaces.msg import (
    PrepareNavigationAction, PrepareNavigationFeedback, PrepareNavigationResult,
    PickTomatoAction, PickTomatoFeedback, PickTomatoResult,
    ScanTomatoAction, ScanTomatoFeedback, ScanTomatoResult,
    PlaceTomatoAction, PlaceTomatoFeedback, PlaceTomatoResult,
    # [BRL_lab] re-enabled import (VisualServoingAction.action added to brl_interfaces)
    VisualServoingActionAction, VisualServoingActionFeedback, VisualServoingActionResult
)

from motion_planning import MotionPlanner, VisualServoing


class MotionPlannerNode:
    """Motion Planner ROS node (merged with Visual Servoing)"""

    def __init__(self, robot_name, planner_class=MotionPlanner):
        """
        Initialize node

        Args:
            robot_name: Robot name (topic/service prefix)
        """
        rospy.init_node('motion_planner_node', anonymous=False)

        self.robot_name = robot_name
        rospy.loginfo("[MotionPlannerNode] Initializing for robot '%s'" % robot_name)

        # MotionPlanner initialization (shared)
        self.planner = planner_class()
        self.planner.setup_scene()

        # Publishers
        self.gripper_pub = rospy.Publisher(
            "/brl_robot/gripper/cmd", GripperCmd, queue_size=10)

        # Robotiq status reports the measured jaw opening in metres.
        self.gripper_status_topic = rospy.get_param(
            "~gripper_status_topic", "/gripper/stat")
        self.gripper_holding_position_threshold = float(rospy.get_param(
            "~gripper_holding_position_threshold", 0.020))
        self.gripper_status_timeout = float(rospy.get_param(
            "~gripper_status_timeout", 2.0))
        self.gripper_status_sample_count = int(rospy.get_param(
            "~gripper_status_sample_count", 20))
        self.gripper_status_lock = threading.Lock()
        self.gripper_status_sequence = 0
        self.latest_gripper_status = None
        self.gripper_status_history = []
        self.gripper_status_sub = rospy.Subscriber(
            self.gripper_status_topic, GripperStat, self.gripper_status_cb,
            queue_size=1)

        # [BRL_lab] Visual servoing behind ~enable_visual_servoing (default True = original BRL: step VS in every Pick)
        self.enable_visual_servoing = bool(rospy.get_param("~enable_visual_servoing", True))
        self.vs = None
        self.vs_server = None
        if self.enable_visual_servoing:
            # VisualServoing initialization (uses planner.move_cartesian directly)
            self.vs = VisualServoing(
                robot_name,
                self.gripper_pub,
                self.planner.move_cartesian  # Direct method reference
            )

            # Detection Subscriber (for visual servoing)
            rospy.Subscriber(
                "/brl_robot/yolo_detection/camera2_best_target",
                Detection2D,
                self.detection_callback)


        # ==================== Action Servers ====================
        # Manipulation Actions
        self.pick_server = actionlib.SimpleActionServer(
            "/brl_robot/pick_tomato", PickTomatoAction,
            self.pick_tomato_cb, False)
        self.pick_server.start()

        self.scan_server = actionlib.SimpleActionServer(
            "/brl_robot/scan_tomato", ScanTomatoAction,
            self.scan_tomato_cb, False)
        self.scan_server.start()

        self.place_server = actionlib.SimpleActionServer(
            "/brl_robot/place_tomato", PlaceTomatoAction,
            self.place_tomato_cb, False)
        self.place_server.start()

        self.prepare_nav_server = actionlib.SimpleActionServer(
            "/brl_robot/prepare_navigation", PrepareNavigationAction,
            self.prepare_navigation_cb, False)
        self.prepare_nav_server.start()

        # [BRL_lab] Visual Servoing Action (only with ~enable_visual_servoing)
        if self.enable_visual_servoing:
            self.vs_server = actionlib.SimpleActionServer(
                "/brl_robot/visual_servoing_action",
                VisualServoingActionAction,
                self.visual_servoing_cb,
                auto_start=False)
            self.vs_server.start()

        rospy.loginfo("[MotionPlannerNode] Ready - robot: %s" % robot_name)

    # ==================== Detection Callback ====================

    def detection_callback(self, msg):
        """
        Detection2D callback (for visual servoing)

        Args:
            msg: Detection2D message
        """
        if self.vs is not None:  # [BRL_lab] add guard
            self.vs.update_detection(msg)

    def gripper_status_cb(self, message):
        """Store the latest Robotiq status message."""
        with self.gripper_status_lock:
            self.gripper_status_sequence += 1
            self.latest_gripper_status = message
            self.gripper_status_history.append(
                (self.gripper_status_sequence, message))
            self.gripper_status_history = self.gripper_status_history[-100:]

    def gripper_status_marker(self):
        with self.gripper_status_lock:
            return self.gripper_status_sequence

    def wait_for_stopped_gripper_position_median(self, after_sequence):
        """Collect new stopped statuses and return their position median."""
        deadline = time.time() + self.gripper_status_timeout
        rate = rospy.Rate(100)
        positions = []
        last_status = None
        last_sequence = after_sequence
        while not rospy.is_shutdown() and time.time() < deadline:
            with self.gripper_status_lock:
                new_statuses = [
                    (sequence, status)
                    for sequence, status in self.gripper_status_history
                    if sequence > last_sequence
                ]
            for sequence, status in new_statuses:
                last_sequence = max(last_sequence, sequence)
                if status.is_ready and not status.is_moving:
                    positions.append(float(status.position))
                    last_status = status
                    if len(positions) >= self.gripper_status_sample_count:
                        ordered = sorted(positions)
                        middle = len(ordered) // 2
                        if len(ordered) % 2:
                            median_position = ordered[middle]
                        else:
                            median_position = (
                                ordered[middle - 1] + ordered[middle]) / 2.0
                        return median_position, last_status, len(positions)
            rate.sleep()
        return None, last_status, len(positions)

    # [BRL_lab] re-enabled (was "[HQP] Visual servoing temporarily disabled")
    # ==================== Visual Servoing Action ====================

    def visual_servoing_cb(self, goal):
        """Visual Servoing Action callback"""
        rospy.loginfo("[VisualServoing] Action started")

        # Reset state machine
        self.vs.reset()

        feedback = VisualServoingActionFeedback()
        result = VisualServoingActionResult()

        # Main loop
        rate = rospy.Rate(30)  # 30Hz
        max_step = 15

        # while not rospy.is_shutdown() and not self.vs.is_done():
        for step in range(max_step):
            if rospy.is_shutdown() and self.vs.is_done():
                break

            # Preempt check
            if self.vs_server.is_preempt_requested():
                rospy.loginfo("[VisualServoing] Preempted")
                self.vs.cleanup()
                self.vs_server.set_preempted()
                return

            # Publish feedback
            # [BRL_lab] action Feedback has distance_to_target (no "state" field); state is logged instead
            rospy.logdebug("[VisualServoing] state %s" % self.vs.state)
            self.vs_server.publish_feedback(feedback)

            # State machine step
            self.vs.step()

            time.sleep(0.75) 
            #rate.sleep()

        # Completion
        self.vs.cleanup()
        result.success = self.vs.is_done()

        if result.success:
            rospy.loginfo("[VisualServoing] Action succeeded")
            self.vs_server.set_succeeded(result)
        else:
            rospy.logwarn("[VisualServoing] Action failed")
            self.vs_server.set_aborted(result)

    # ==================== Manipulation Actions ====================

    def pick_tomato_cb(self, goal):
        """PickTomato Action callback"""
        rospy.loginfo("[PickTomato] Starting pick for tomato: %s" % goal.tomato_name)

        # Feedback callback
        def feedback_cb(progress, status):
            feedback = PickTomatoFeedback()
            feedback.progress = progress
            feedback.status = status
            self.pick_server.publish_feedback(feedback)

        # Preempt check callback
        def preempt_check_cb():
            return self.pick_server.is_preempt_requested()

        # Call MotionPlanner logic (uses internal visual servoing)
        success, message = self.planner.pick_tomato(
            tomato_position=goal.tomato_position,
            vs_instance=self.vs,  # [BRL_lab] None unless ~enable_visual_servoing
            vs_server=self.vs_server,
            gripper_pub=self.gripper_pub,
            feedback_cb=feedback_cb,
            preempt_check_cb=preempt_check_cb
        )

        # Return result
        result = PickTomatoResult()
        result.success = success
        result.message = message

        if message == "Preempted by client":
            self.pick_server.set_preempted(result)
        elif success:
            status_marker = self.gripper_status_marker()
            measured_position, status, sample_count = (
                self.wait_for_stopped_gripper_position_median(status_marker)
            )
            if measured_position is None:
                result.message = (
                    "%s; received %d/%d stopped gripper statuses on %s within %.1fs; "
                    "reporting handempty" % (
                        message, sample_count, self.gripper_status_sample_count,
                        self.gripper_status_topic,
                        self.gripper_status_timeout))
                observation = "handempty(%s)" % goal.robot_name
                rospy.logwarn("[PickTomato] %s", result.message)
            else:
                if measured_position < self.gripper_holding_position_threshold:
                    observation = "holding(%s,%s)" % (
                        goal.robot_name, goal.tomato_name)
                else:
                    observation = "handempty(%s)" % goal.robot_name
                result.message = (
                    "%s; median gripper position %.6f m from %d samples "
                    "(holding when < %.6f m)" % (
                        message, measured_position, sample_count,
                        self.gripper_holding_position_threshold))
            result.facts = [observation]
            result.confidences = [1.0]
            if measured_position is not None and status is not None:
                rospy.loginfo(
                    "[PickTomato] Observation: %s | median_position=%.6f m "
                    "samples=%d threshold=%.6f m obj_detected=%s current=%.3f",
                    observation, measured_position, sample_count,
                    self.gripper_holding_position_threshold,
                    status.obj_detected, float(status.current))
            else:
                rospy.logwarn(
                    "[PickTomato] Observation: %s | insufficient samples=%d/%d",
                    observation, sample_count, self.gripper_status_sample_count)
            self.pick_server.set_succeeded(result)
        else:
            self.pick_server.set_aborted(result)

    def scan_tomato_cb(self, goal):
        """ScanTomato Action callback"""
        rospy.loginfo("[ScanTomato] Starting scan for robot: %s" % goal.robot_name)

        # Feedback callback
        def feedback_cb(progress, status):
            feedback = ScanTomatoFeedback()
            feedback.progress = progress
            feedback.status = status
            self.scan_server.publish_feedback(feedback)

        # Preempt check callback
        def preempt_check_cb():
            return self.scan_server.is_preempt_requested()

        # Call MotionPlanner logic
        success, message = self.planner.scan_tomato(
            feedback_cb=feedback_cb,
            preempt_check_cb=preempt_check_cb
        )

        # Return result
        result = ScanTomatoResult()
        result.success = success
        result.message = message

        if message == "Preempted by client":
            self.scan_server.set_preempted(result)
        elif success:
            self.scan_server.set_succeeded(result)
        else:
            self.scan_server.set_aborted(result)

    def place_tomato_cb(self, goal):
        """PlaceTomato Action callback"""
        rospy.loginfo("[PlaceTomato] Placing tomato: %s (class: %s)" % (goal.tomato_name, goal.tomato_class))

        # Feedback callback
        def feedback_cb(progress, status):
            feedback = PlaceTomatoFeedback()
            feedback.progress = progress
            feedback.status = status
            self.place_server.publish_feedback(feedback)

        # Preempt check callback
        def preempt_check_cb():
            return self.place_server.is_preempt_requested()

        # Call MotionPlanner logic
        success, message = self.planner.place_tomato(
            tomato_class=goal.tomato_class,
            gripper_pub=self.gripper_pub,
            feedback_cb=feedback_cb,
            preempt_check_cb=preempt_check_cb
        )

        # Return result
        result = PlaceTomatoResult()
        result.success = success
        result.message = message

        if message == "Preempted by client":
            self.place_server.set_preempted(result)
        elif success:
            self.place_server.set_succeeded(result)
        else:
            self.place_server.set_aborted(result)

    def prepare_navigation_cb(self, goal):
        """PrepareNavigation Action callback"""
        rospy.loginfo("[PrepareNavigation] Starting for robot: %s" % goal.robot_name)

        # Feedback callback
        def feedback_cb(progress, status):
            feedback = PrepareNavigationFeedback()
            feedback.progress = progress
            feedback.status = status
            self.prepare_nav_server.publish_feedback(feedback)

        # Preempt check callback
        def preempt_check_cb():
            return self.prepare_nav_server.is_preempt_requested()

        # Call MotionPlanner logic
        success, message = self.planner.prepare_navigation(
            feedback_cb=feedback_cb,
            preempt_check_cb=preempt_check_cb
        )

        # Return result
        result = PrepareNavigationResult()
        result.success = success
        result.message = message

        if message == "Preempted by client":
            self.prepare_nav_server.set_preempted(result)
        elif success:
            self.prepare_nav_server.set_succeeded(result)
        else:
            self.prepare_nav_server.set_aborted(result)


def main():
    """Main function"""
    try:
        robot_name = rospy.get_param("~robot_name", "brl_robot")
        node = MotionPlannerNode(robot_name)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
