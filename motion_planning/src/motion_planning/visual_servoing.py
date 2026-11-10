#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
VisualServoing - Detection2D 기반 Visual Servoing 로직

상태머신 기반 토마토 접근:
- State 0 (CENTER): 중앙 정렬 (토마토를 카메라 중앙으로 이동)
- State 2 (APPROACH): 접근 (전진, bbox 크기 확인)
- State 4 (GRIP): 그리퍼 닫기
- State 5 (WAIT): 대기
- State 6 (DONE): 완료

의존성 (Node에서 전달):
- gripper_pub: 그리퍼 명령 publisher
- move_cartesian_fn: Cartesian 이동 함수 (MotionPlanner에서 제공)

검출 입력:
- /{robot_name}/yolo_detection/camera2_best_target (Detection2D)
  brl_sensor/tomato_detector.py에서 발행

Python 2/3 호환
"""
from __future__ import print_function, division, absolute_import

import time
import rospy


MAX_ALIGNMENT_ATTEMPTS = 11
BBOX_TIMEOUT_SECONDS = 5.0
# Python 3에서는 monotonic clock, Python 2(ROS Melodic)에서는 wall clock을 사용한다.
_wall_time = getattr(time, "monotonic", time.time)


class VisualServoing:
    """Detection2D 기반 Visual Servoing 클래스"""

    # 상태 상수
    STATE_CENTER = 0
    STATE_APPROACH = 2
    STATE_GRIP = 4
    STATE_WAIT = 5
    STATE_DONE = 6

    def __init__(self, robot_name, gripper_pub, move_cartesian_fn):
        """
        Visual Servoing 초기화

        Args:
            robot_name: 로봇 이름 (토픽/서비스 prefix)
            gripper_pub: 그리퍼 명령 publisher
            move_cartesian_fn: Cartesian 이동 함수 (x, y, z, roll, pitch, yaw) -> (success, message)
        """
        self.robot_name = robot_name
        self.gripper_pub = gripper_pub
        self.move_cartesian_fn = move_cartesian_fn

        # State machine
        self.state = self.STATE_CENTER
        self.counter = 0
        self.alignment_attempt_count = 0
        self.forced_grip_due_alignment_limit = False
        self.forced_grip_due_bbox_timeout = False

        # Detection data (updated by Node)
        self.latest_detection = None
        self.detection_sequence = 0
        self.last_usable_detection_sequence = -1
        self.last_usable_bbox_wall_time = _wall_time()
        self.move_complete_time = None  # 이동 완료 시점 (동기화용)

        # VS parameters
        self.center_range_x = (400, 460)
        self.center_range_y = (200, 260)
        self.target_bbox_size = 350
        self.move_scale = 0.02
        self.max_alignment_attempts = max(1, int(MAX_ALIGNMENT_ATTEMPTS))
        self.bbox_timeout_seconds = float(BBOX_TIMEOUT_SECONDS)

        # Gripper positions
        self.gripper_open = 0.087
        self.gripper_close = 0.0

        # Initial gripper open
        self.move_gripper(self.gripper_open)
        time.sleep(1.0)

        rospy.loginfo("[VisualServoing] Initialized for robot '%s'" % robot_name)

    def reset(self):
        """상태머신 초기화"""
        self.state = self.STATE_CENTER
        self.counter = 0
        self.alignment_attempt_count = 0
        self.forced_grip_due_alignment_limit = False
        self.forced_grip_due_bbox_timeout = False
        self.latest_detection = None
        self.detection_sequence = 0
        self.last_usable_detection_sequence = -1
        self.last_usable_bbox_wall_time = _wall_time()
        self.move_complete_time = None
        self.move_gripper(self.gripper_open)

    def update_detection(self, detection):
        """
        검출 데이터 업데이트 (Node에서 호출)

        Args:
            detection: Detection2D 메시지
        """
        self.latest_detection = detection
        self.detection_sequence += 1

    def is_done(self):
        """완료 여부 확인"""
        return self.state == self.STATE_DONE

    # ==================== TCP 제어 ====================

    def move(self, direction):
        """
        TCP Cartesian 이동

        Args:
            direction: [x, y, z, rx, ry, rz] movement amount

        Returns:
            Success status
        """
        if self.move_cartesian_fn is None:
            rospy.logerr("[VisualServoing] Cartesian move function not available")
            return False

        try:
            x, y, z = direction[0], direction[1], direction[2]
            roll, pitch, yaw = direction[3], direction[4], direction[5]
            success, message = self.move_cartesian_fn(x, y, z, roll, pitch, yaw)
            # 이동 완료 시점 기록 (SAM3 동기화용)
            self.move_complete_time = rospy.Time.now()
            # 이동 이전 bbox는 더 이상 사용할 수 없으므로 실시간 타임아웃을 다시 시작한다.
            self.last_usable_bbox_wall_time = _wall_time()
            return success
        except Exception as e:
            rospy.logerr("[VisualServoing] Cartesian move failed: %s" % str(e))
            return False

    def move_gripper(self, position):
        """
        Gripper control

        Args:
            position: Gripper position (0.0 = closed, 0.087 = open)
        """
        if self.gripper_pub is None:
            rospy.logerr("[VisualServoing] Gripper publisher not available")
            return

        from brl_interfaces.msg import GripperCmd
        msg = GripperCmd()
        msg.position = position
        self.gripper_pub.publish(msg)

    # Convenience methods
    def move_forward(self, distance):
        self.move([0, -distance, 0, 0, 0, 0])

    def move_backward(self, distance):
        self.move([0, distance, 0, 0, 0, 0])

    def move_left(self, distance):
        self.move([distance, 0, 0, 0, 0, 0])

    def move_right(self, distance):
        self.move([-distance, 0, 0, 0, 0, 0])

    def move_up(self, distance):
        self.move([0, 0, -distance, 0, 0, 0])

    def move_down(self, distance):
        self.move([0, 0, distance, 0, 0, 0])

    # ==================== State Machine ====================

    def _wait_for_bbox_or_force_grip(self, reason):
        """Usable bbox를 기다리고, 실시간 5초 타임아웃 시 집기 fallback을 수행한다."""
        elapsed = _wall_time() - self.last_usable_bbox_wall_time
        if elapsed < self.bbox_timeout_seconds:
            rospy.logwarn_throttle(
                1.0,
                "[VisualServoing] Waiting for bbox (%s, %.1f/%.1fs)" %
                (reason, elapsed, self.bbox_timeout_seconds),
            )
            return False

        rospy.logwarn(
            "[VisualServoing] No usable bbox for %.1fs (%s); "
            "moving forward %.3fm before forcing gripper close",
            elapsed,
            reason,
            self.move_scale,
        )
        self.forced_grip_due_bbox_timeout = True
        # 기존 alignment-limit fallback(2 x move_scale)의 1/2만 전진한다.
        self.move_forward(self.move_scale)
        self.state = self.STATE_GRIP
        return True

    def step(self):
        """
        Execute one step of state machine

        Returns:
            Current state
        """
        # GRIP/WAIT는 bbox가 없어도 끝까지 진행해야 한다.
        if self.state == self.STATE_GRIP:
            self._handle_grip_state()
            return self.state

        if self.state == self.STATE_WAIT:
            self._handle_wait_state()
            return self.state

        if self.state == self.STATE_DONE:
            return self.state

        # Check detection data
        if self.latest_detection is None:
            self._wait_for_bbox_or_force_grip("no detection message")
            return self.state

        detection = self.latest_detection

        # 이동 완료 전 캡처된 이미지 무시 (SAM3 동기화)
        if self.move_complete_time is not None:
            if detection.header.stamp < self.move_complete_time:
                self._wait_for_bbox_or_force_grip("stale detection after move")
                return self.state

        rospy.logdebug("[VisualServoing] State: %d, Counter: %d" % (self.state, self.counter))

        # Check detection validity
        if not detection.valid:
            self.state = self.STATE_CENTER
            self.counter = 0
            self._wait_for_bbox_or_force_grip("invalid detection")
            return self.state

        # 새로 도착한, 이동 이후의 valid bbox만 타임아웃 시각을 갱신한다.
        if self.detection_sequence != self.last_usable_detection_sequence:
            self.last_usable_detection_sequence = self.detection_sequence
            self.last_usable_bbox_wall_time = _wall_time()

        # Extract detection data
        center = (detection.center_x, detection.center_y)
        bbox_width = detection.width

        rospy.logdebug("[VisualServoing] Detected at center=(%f, %f), width=%f" %
                       (center[0], center[1], bbox_width))

        # State handling
        if self.state == self.STATE_CENTER:
            self._handle_center_state(center)

        elif self.state == self.STATE_APPROACH:
            self._handle_approach_state(bbox_width)

        return self.state

    def _handle_center_state(self, center):
        """State 0: Center alignment"""
        scale = self.move_scale
        completed = True

        # X-axis alignment
        if center[0] < self.center_range_x[0]:
            self.move_left(scale)
            completed = False
        elif center[0] > self.center_range_x[1]:
            self.move_right(scale)
            completed = False

        # Y-axis alignment
        if center[1] < self.center_range_y[0]:
            self.move_down(scale)
            completed = False
        elif center[1] > self.center_range_y[1]:
            self.move_up(scale)
            completed = False

        if completed:
            self.state = self.STATE_APPROACH
            self.alignment_attempt_count = 0
        else:
            self.alignment_attempt_count += 1
            rospy.logwarn(
                "[VisualServoing] Center alignment correction %d/%d",
                self.alignment_attempt_count,
                self.max_alignment_attempts,
            )
            if self.alignment_attempt_count >= self.max_alignment_attempts:
                rospy.logwarn(
                    "[VisualServoing] Center alignment limit reached; "
                    "moving forward twice before forcing gripper close"
                )
                self.forced_grip_due_alignment_limit = True
                for forward_step in range(2):
                    rospy.logwarn(
                        "[VisualServoing] Forced forward move %d/2",
                        forward_step + 1,
                    )
                    self.move_forward(self.move_scale)
                self.state = self.STATE_GRIP

        self.move_gripper(self.gripper_open)

    def _handle_approach_state(self, bbox_width):
        """State 2: Approach"""
        if bbox_width > self.target_bbox_size:
            self.state = self.STATE_GRIP
            return

        self.move_forward(self.move_scale)
        self.counter += 1

        # Return to center alignment after certain count
        if self.counter > 2:
            self.state = self.STATE_CENTER
            self.counter = 0

    def _handle_grip_state(self):
        """State 4: Close gripper"""
        self.move_gripper(self.gripper_close)
        self.state = self.STATE_WAIT

    def _handle_wait_state(self):
        """State 5: Wait"""
        time.sleep(5)
        self.state = self.STATE_DONE

    def cleanup(self):
        """Resource cleanup"""
        pass  # cv2 removed
