#!/usr/bin/env python3
"""
Robot Tracker - AMCL 기반 로봇 위치 추적

AMCL pose를 받아서 Knowledge Base에 로봇 위치 업데이트
- AMCL pose 실시간 추적
- 로봇 자동 초기화 (최초 생성)
- Knowledge Base 서비스 연동
"""

import rospy
import time
from typing import Optional

from geometry_msgs.msg import PoseWithCovarianceStamped
from brl_interfaces.srv import UpdateRobot, UpdateRobotRequest, UpdateRobotResponse
from common_models import Robot
from common_models.converters import RosConverter


class RobotTracker:
    """AMCL 기반 로봇 위치 추적 및 Knowledge Base 업데이트"""

    def __init__(self, robot: Robot, robot_update_client):
        """
        Args:
            robot: Robot 인스턴스 (중앙에서 생성된 공유 인스턴스)
            robot_update_client: UpdateRobot 서비스 클라이언트 (의존성 주입)

        Note:
            위치 정보(current_location)는 Scene Graph 토픽에서 관리됨 (sensor_node.py)
            KB 등록은 첫 AMCL 메시지 수신 후에 수행됨 (AMCL 없으면 등록 안함)
        """
        self.robot = robot
        self.robot_update_client = robot_update_client

        # State variables
        self.last_update_time = 0
        self._registered_to_kb = False  # KB 등록 여부

        # Initialize AMCL subscriber
        self.init_navigation_subscriber()

        rospy.loginfo(f"[RobotTracker] Initialized for robot: {robot.name}")
        rospy.loginfo(f"[RobotTracker] Waiting for AMCL data before registering to KB...")

    def init_navigation_subscriber(self):
        """AMCL 토픽 구독 초기화"""
        amcl_topic = f"/{self.robot.name}/amcl_pose"

        try:
            rospy.loginfo(f"Subscribing to AMCL topic: {amcl_topic}")

            # 토픽 존재 확인
            import rostopic
            try:
                topic_type = rostopic.get_topic_type(amcl_topic, blocking=True)[0]
                if topic_type is None:
                    rospy.logerr(f"Topic {amcl_topic} does not exist")
                    rospy.logerr("Available AMCL topics:")
                    import subprocess
                    result = subprocess.run(['rostopic', 'list'], capture_output=True, text=True)
                    amcl_topics = [t for t in result.stdout.split('\n') if 'amcl_pose' in t]
                    for topic in amcl_topics:
                        rospy.logerr(f"  {topic}")
                    return
                else:
                    rospy.loginfo(f"Topic {amcl_topic} found with type: {topic_type}")
            except Exception as e:
                rospy.logwarn(f"Could not verify topic existence: {e}")

            self.pose_subscriber = rospy.Subscriber(
                amcl_topic,
                PoseWithCovarianceStamped,
                self.amcl_callback,
                queue_size=10
            )
            rospy.loginfo(f"Successfully subscribed to AMCL: {amcl_topic}")

        except Exception as e:
            rospy.logerr(f"Failed to subscribe to {amcl_topic}: {e}")

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        """AMCL 위치 콜백 - 로봇 위치 업데이트"""
        try:
            # Extract pose
            pose = msg.pose.pose
            position = pose.position
            orientation = pose.orientation

            # Convert quaternion to euler angle (yaw)
            from tf.transformations import euler_from_quaternion
            euler = euler_from_quaternion([
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w
            ])
            yaw = euler[2]

            # Update robot pose (공유 인스턴스 업데이트!)
            self.robot.position.x = position.x
            self.robot.position.y = position.y
            self.robot.position.theta = yaw

            # 첫 AMCL 수신 시 KB에 로봇 등록
            if not self._registered_to_kb:
                rospy.loginfo(f"First AMCL pose received for {self.robot.name}: "
                            f"({position.x:.2f}, {position.y:.2f}, {yaw:.2f})")
                self._register_robot_to_kb()
                self._registered_to_kb = True
                self.last_update_time = time.time()  # 중복 호출 방지
                return  # 등록 직후 바로 리턴

            # Update Knowledge Base with throttling
            current_time = time.time()
            update_interval = 0.5  # Default update interval
            if current_time - self.last_update_time >= update_interval:
                self.update_robot_position()
                self.last_update_time = current_time

        except Exception as e:
            rospy.logerr(f"Error in AMCL callback: {e}")

    def update_robot_position(self):
        """로봇 위치를 Knowledge Base에 업데이트"""
        try:
            # Pydantic Robot → ROS RobotMsg (RosConverter 사용!)
            robot_msg = RosConverter.robot_to_msg(
                self.robot,
                current_location_name=""  # KB가 position 기반으로 자동 계산
            )

            # Call service
            req = UpdateRobotRequest()
            req.robot_data = robot_msg

            response = self.robot_update_client(req)

            if response.success:
                # 위치 정보는 Scene Graph 토픽에서 관리됨 (Single Source of Truth)
                rospy.loginfo_throttle(
                    5,
                    f"Robot position updated: "
                    f"({self.robot.position.x:.2f}, {self.robot.position.y:.2f}, "
                    f"{self.robot.position.theta:.2f})"
                )
            else:
                rospy.logwarn_throttle(10, f"Failed to update robot position: {response.message}")

        except Exception as e:
            rospy.logerr(f"Error updating robot position: {e}")

    def _register_robot_to_kb(self):
        """Robot을 Knowledge Base에 등록 (첫 AMCL 수신 시 호출)"""
        try:
            rospy.loginfo(f"Registering robot '{self.robot.name}' to Knowledge Base...")

            # Pydantic Robot → ROS RobotMsg (RosConverter 사용!)
            robot_msg = RosConverter.robot_to_msg(
                self.robot,
                current_location_name=""  # KB가 position 기반으로 자동 할당
            )

            # Call UpdateRobot service
            req = UpdateRobotRequest()
            req.robot_data = robot_msg

            response = self.robot_update_client(req)

            if response.success:
                rospy.loginfo(f"Robot '{self.robot.name}' registered successfully")
                rospy.loginfo(f"Type: {self.robot.type}, Capabilities: {self.robot.capabilities}")
            else:
                rospy.logerr(f"Failed to register robot: {response.message}")

        except Exception as e:
            rospy.logerr(f"Error registering robot to KB: {e}")

    def get_robot_status(self) -> str:
        """로봇 상태 정보 반환"""
        return (f"Robot {self.robot.name} - "
               f"pose: ({self.robot.position.x:.2f}, {self.robot.position.y:.2f}, "
               f"{self.robot.position.theta:.2f}), status: {self.robot.status}")
