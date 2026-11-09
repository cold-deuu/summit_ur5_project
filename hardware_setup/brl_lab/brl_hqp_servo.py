#!/usr/bin/env python3
"""[BRL_lab] Visual servoing solved by HQP: wrist-camera tomato estimate -> streamed graps_link target.

Replaces BRL's step servo (visual_servoing.py: 2 cm MoveIt Cartesian steps toward a fixed pixel
window) when motion_planner.py runs with ~hqp_visual_servoing. Action server
/<robot>/hqp_visual_servoing (brl_interfaces VisualServoingAction.action, goal.on=True).

Per camera frame:
  1. Tomato point in the camera optical frame from the YOLO best target (bbox centre ray; a box cut
     by the image border is re-centred from its uncut edge and size, the fruit being round):
     depth image median in the bbox center (+ radius, surface -> center); else, if the bbox is
     not cut by the image border, the size prior Z = fx * diameter / bbox width; else only the
     ray (keep the current range, correct sideways). The D435 has no depth very close, so the
     last centimeters use the size prior / ray.
  2. To the base frame by TF at the image stamp (tool->camera = hand-eye; sim: brl_sim_wrist_camera.py).
  3. Filtered (outlier gate, max offset from the first estimate).
Target for graps_link, orientation fixed at the start pose (BRL pick orientation), approach axis
a = graps_link z:
  ALIGN    : tomato - standoff * a  (centre the gripper on the approach line, out of contact)
  APPROACH : tomato + grasp_offset * a
Streamed at ~rate to /rci_action_manager/servo_target; HQP (servo_enabled) moves graps_link by
damped differential IK under its speed/acceleration/joint limits. Done when graps_link is within
~final_tolerance: stop streaming (HQP brakes, holds, SERVO_ENDED), close the gripper.
Aborts (no gripper close): no detection, lost target before the approach, estimate jump,
HQP abort (status ABORTED for this stream), timeout, preempt.
"""
import threading

import actionlib
import numpy as np
import rospy
import tf2_ros
from brl_interfaces.msg import (Detection2D, GripperCmd, VisualServoingActionAction,
                                VisualServoingActionFeedback, VisualServoingActionResult)
from geometry_msgs.msg import PoseStamped
from rci_action_manager.msg import MoveitOptimalStatus
from sensor_msgs.msg import CameraInfo, Image
from tf.transformations import quaternion_from_matrix, quaternion_matrix


def matrix(transform):
    t, q = transform.transform.translation, transform.transform.rotation
    m = quaternion_matrix([q.x, q.y, q.z, q.w])
    m[:3, 3] = [t.x, t.y, t.z]
    return m


class HqpServo:
    def __init__(self):
        robot = rospy.get_param("~robot_name", "brl_robot")
        self.base_frame = rospy.get_param("~base_frame", "summit_xl_base_footprint")
        self.tool_frame = rospy.get_param("~tool_frame", "graps_link")
        self.diameter = float(rospy.get_param("~tomato_diameter", 0.066))
        self.standoff = float(rospy.get_param("~standoff", 0.10))
        self.grasp_offset = float(rospy.get_param("~grasp_offset", 0.0))
        self.align_tolerance = float(rospy.get_param("~align_tolerance", 0.008))
        self.final_tolerance = float(rospy.get_param("~final_tolerance", 0.005))
        self.rate_hz = float(rospy.get_param("~rate", 30.0))
        self.detection_timeout = float(rospy.get_param("~detection_timeout", 1.5))
        self.max_duration = float(rospy.get_param("~max_duration", 40.0))
        self.max_jump = float(rospy.get_param("~max_target_jump", 0.05))
        self.max_offset = float(rospy.get_param("~max_offset_from_first", 0.15))
        self.alpha = float(rospy.get_param("~filter_alpha", 0.4))
        self.depth_patch = float(rospy.get_param("~depth_patch", 0.3))
        self.grip_close = float(rospy.get_param("~gripper_close", 0.0))
        self.grip_open = float(rospy.get_param("~gripper_open", 0.087))
        self.grip_wait = float(rospy.get_param("~grip_wait", 2.0))

        self.edge_correction = True
        self.lock = threading.Lock()
        self.detection = None
        self.depth = None
        self.info = None
        self.status = None
        self.tf = tf2_ros.Buffer(rospy.Duration(10))
        tf2_ros.TransformListener(self.tf)
        rospy.Subscriber(rospy.get_param("~detection_topic", "/%s/yolo_detection/camera2_best_target" % robot),
                         Detection2D, self._set("detection"), queue_size=1)
        rospy.Subscriber(rospy.get_param("~depth_topic", "/%s/camera2/aligned_depth_to_color/image_raw" % robot),
                         Image, self._set("depth"), queue_size=1, buff_size=2 ** 24)
        rospy.Subscriber(rospy.get_param("~camera_info_topic", "/%s/camera2/color/camera_info" % robot),
                         CameraInfo, self._set("info"), queue_size=1)
        rospy.Subscriber(rospy.get_param("~status_topic", "/moveit_optimal_controller/status"),
                         MoveitOptimalStatus, self._set("status"), queue_size=1)
        self.target_pub = rospy.Publisher(rospy.get_param("~servo_topic", "/rci_action_manager/servo_target"),
                                          PoseStamped, queue_size=1)
        self.gripper_pub = rospy.Publisher(rospy.get_param("~gripper_topic", "/%s/gripper/cmd" % robot),
                                           GripperCmd, queue_size=10)
        self.server = actionlib.SimpleActionServer("/%s/hqp_visual_servoing" % robot, VisualServoingActionAction,
                                                   self.execute, False)
        self.server.start()
        rospy.loginfo("[hqp_servo] ready: /%s/hqp_visual_servoing (standoff %.3f m, %s in %s)",
                      robot, self.standoff, self.tool_frame, self.base_frame)

    def _set(self, name):
        def callback(msg):
            with self.lock:
                setattr(self, name, msg)
        return callback

    def _gripper(self, position):
        self.gripper_pub.publish(GripperCmd(position=position))

    def _tool(self):
        return matrix(self.tf.lookup_transform(self.base_frame, self.tool_frame, rospy.Time(0), rospy.Duration(0.2)))

    def _measure(self, det, depth, info, estimate):
        """Tomato centre in the base frame, or None. Returns (point, source)."""
        fx, fy, cx, cy = info.K[0], info.K[4], info.K[2], info.K[5]
        # A box cut by the image border has its centre shifted inward. The fruit is round:
        # rebuild the centre from the uncut edge and the uncut size (sim: the camera sits 8 cm
        # above the gripper, so near the tomato it is cut at the bottom of the image).
        x0, x1 = det.center_x - det.width / 2, det.center_x + det.width / 2
        y0, y1 = det.center_y - det.height / 2, det.center_y + det.height / 2
        cut_x0, cut_x1, cut_y0, cut_y1 = x0 <= 2, x1 >= info.width - 3, y0 <= 2, y1 >= info.height - 3
        u, v = det.center_x, det.center_y
        if not self.edge_correction:  # ~edge_correction false: plain box centre
            cut_x0 = cut_x1 = cut_y0 = cut_y1 = False
        if (cut_y0 or cut_y1) and not (cut_y0 and cut_y1) and not (cut_x0 or cut_x1):
            v = y0 + det.width / 2 if cut_y1 else y1 - det.width / 2
        if (cut_x0 or cut_x1) and not (cut_x0 and cut_x1) and not (cut_y0 or cut_y1):
            u = x0 + det.height / 2 if cut_x1 else x1 - det.height / 2
        ray = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
        try:
            cam = matrix(self.tf.lookup_transform(self.base_frame, det.header.frame_id, det.header.stamp,
                                                  rospy.Duration(0.1)))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
            return None, "tf"
        point_cam, source = None, None
        if depth is not None and depth.encoding == "16UC1" and abs((depth.header.stamp - det.header.stamp).to_sec()) < 0.1:
            image = np.frombuffer(depth.data, dtype=np.uint16).reshape(depth.height, depth.width)
            size = min(det.width, det.height) if (cut_x0 or cut_x1 or cut_y0 or cut_y1) else None
            hw = max(2, int((size or det.width) * self.depth_patch / 2))
            hh = max(2, int((size or det.height) * self.depth_patch / 2))
            pu, pv = int(round(u)), int(round(v))
            patch = image[max(0, pv - hh):max(0, pv + hh + 1), max(0, pu - hw):max(0, pu + hw + 1)]
            values = patch[patch > 0]
            if values.size >= 5:
                surface = float(np.median(values)) / 1000.0
                point_cam = ray * surface + ray / np.linalg.norm(ray) * (self.diameter / 2)
                source = "depth"
        if point_cam is None:
            # Size prior from an uncut dimension (width if left/right are inside, else height).
            uncut = det.width if not (cut_x0 or cut_x1) else (det.height if not (cut_y0 or cut_y1) else 0)
            if uncut > 5:
                point_cam = ray * fx * self.diameter / uncut
                source = "size"
        if point_cam is not None:
            return (cam @ np.r_[point_cam, 1.0])[:3], source
        if estimate is None:
            return None, "ray"
        origin = cam[:3, 3]
        direction = cam[:3, :3] @ (ray / np.linalg.norm(ray))
        return origin + direction * float((estimate - origin) @ direction), "ray"

    def _finish(self, success, message):
        # Stop streaming; HQP brakes and holds (SERVO_ENDED) or already aborted.
        deadline = rospy.Time.now() + rospy.Duration(3.0)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            with self.lock:
                status = self.status
            if status is not None and status.state != "SERVOING":
                break
            rospy.sleep(0.05)
        if success:
            self._gripper(self.grip_close)
            rospy.sleep(self.grip_wait)
        rospy.loginfo("[hqp_servo] %s: %s", "done" if success else "failed", message)
        result = VisualServoingActionResult(success=success, message=message)
        if not success and self.server.is_preempt_requested():
            self.server.set_preempted(result)
        elif success:
            self.server.set_succeeded(result)
        else:
            self.server.set_aborted(result)

    def execute(self, goal):
        if not goal.on:
            self.server.set_succeeded(VisualServoingActionResult(success=True, message="off"))
            return
        self.edge_correction = bool(rospy.get_param("~edge_correction", True))  # read per goal (rosparam set)
        self._gripper(self.grip_open)
        try:
            start_tool = self._tool()
        except Exception as error:  # noqa: BLE001 - report any TF failure as an abort
            self.server.set_aborted(VisualServoingActionResult(success=False, message="no %s pose: %s" % (self.tool_frame, error)))
            return
        approach = start_tool[:3, 2]  # graps_link z = approach direction of the pick orientation
        orientation = quaternion_from_matrix(start_tool)  # held fixed during the servo
        begin = rospy.Time.now()
        estimate, first, phase = None, None, "ALIGN"
        last_seen = begin
        published = False
        my_stream = None
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            if self.server.is_preempt_requested():
                return self._finish(False, "preempted")
            if (now - begin).to_sec() > self.max_duration:
                return self._finish(False, "timeout after %.0f s" % self.max_duration)
            with self.lock:
                det, depth, info, status = self.detection, self.depth, self.info, self.status
            if published and status is not None:
                if my_stream is None and status.state == "SERVOING":
                    my_stream = status.trajectory_id
                if my_stream is not None and status.result_trajectory_id == my_stream and status.last_result == "ABORTED":
                    return self._finish(False, "HQP aborted the servo: " + status.reason)
            fresh = (det is not None and det.valid and info is not None and
                     (now - det.header.stamp).to_sec() < self.detection_timeout)
            if fresh:
                point, source = self._measure(det, depth, info, estimate)
                if point is not None:
                    if first is None:
                        first = estimate = point
                        rospy.loginfo("[hqp_servo] tomato at (%.3f %.3f %.3f) from %s", *point, source)
                    elif np.linalg.norm(point - first) > self.max_offset:
                        return self._finish(False, "estimate moved %.3f m from the first" % np.linalg.norm(point - first))
                    elif np.linalg.norm(point - estimate) < self.max_jump:
                        estimate = (1 - self.alpha) * estimate + self.alpha * point
                    last_seen = now
            if estimate is None:
                if (now - begin).to_sec() > self.detection_timeout:
                    return self._finish(False, "no tomato detection")
                rate.sleep()
                continue
            tool = self._tool()
            if phase == "ALIGN":
                target = estimate - self.standoff * approach
                if (now - last_seen).to_sec() > self.detection_timeout:
                    return self._finish(False, "tomato lost before the approach")
                if np.linalg.norm(tool[:3, 3] - target) < self.align_tolerance:
                    phase = "APPROACH"
                    rospy.loginfo("[hqp_servo] aligned at standoff %.3f m, approaching", self.standoff)
            if phase == "APPROACH":
                target = estimate + self.grasp_offset * approach
            error = float(np.linalg.norm(tool[:3, 3] - target))
            if phase == "APPROACH" and error < self.final_tolerance:
                tomato_error = tool[:3, 3] - (estimate + self.grasp_offset * approach)
                return self._finish(True, "graps_link at the tomato estimate (error %.1f mm)" % (1000 * np.linalg.norm(tomato_error)))
            msg = PoseStamped()
            msg.header.stamp = now
            msg.header.frame_id = self.base_frame
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = target
            msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = orientation
            self.target_pub.publish(msg)
            published = True
            self.server.publish_feedback(VisualServoingActionFeedback(distance_to_target=error))
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("brl_hqp_servo")
    HqpServo()
    rospy.spin()
