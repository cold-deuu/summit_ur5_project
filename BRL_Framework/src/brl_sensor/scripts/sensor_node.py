#!/usr/bin/env python3
"""
BRL Sensor Node - 로봇 위치 추적 및 토마토 검출

ROS 노드 진입점
- RobotTracker: AMCL 기반 로봇 위치 추적
- TomatoDetector: YOLO 기반 토마토 검출
"""

import rospy
import os
import yaml
import rospkg

from brl_sensor import RobotTracker, TomatoDetector
from brl_interfaces.srv import UpdateRobot, UpdateRobotRequest, UpdateObject, TriggerSensorScan, TriggerSensorScanResponse, PixelTo3D, PixelTo3DResponse
from brl_interfaces.msg import SceneGraphMsg
from common_models import Robot
from common_models.base import Pose2D
from common_models.converters import RosConverter


class BRLSensorNode:
    """BRL Sensor 통합 노드"""

    def __init__(self, robot_name: str):
        # Initialize ROS node
        rospy.init_node(f"brl_sensor_node_{robot_name}", anonymous=False)
        rospy.loginfo(f"BRL Sensor Node Started for robot: {robot_name}")

        self.robot_name = robot_name
        self.current_location = ""  # Scene Graph에서 업데이트됨

        # Load robot configuration
        robot_config = self.load_robot_config()

        # Create Robot instance (중앙에서 생성!)
        self.robot = Robot(
            name=robot_name,
            type=robot_config['type'],
            description=robot_config.get('description', ''),
            status=robot_config.get('initial_status', 'active'),
            position=Pose2D(x=0.0, y=0.0, theta=0.0),  # AMCL에서 업데이트됨
            capabilities=robot_config.get('capabilities', []),
            held_objects={},
            properties=robot_config.get('properties', {})
        )

        rospy.loginfo(f"Robot instance created: {self.robot.name} (type: {self.robot.type})")
        rospy.loginfo(f"Robot properties: {self.robot.properties}")

        # Initialize service clients (모든 ROS 서비스는 여기서!)
        robot_update_client = self.init_robot_service_client()
        object_update_client = self.init_object_service_client()

        # Initialize RobotTracker (Robot 인스턴스 주입!)
        # NOTE: KB 등록은 첫 AMCL 메시지 수신 후 RobotTracker에서 수행
        self.robot_tracker = RobotTracker(
            self.robot,
            robot_update_client
        )

        # Initialize TomatoDetector (Robot 인스턴스 주입!)
        self.tomato_detector = TomatoDetector(
            self.robot,
            object_update_client
        )

        rospy.loginfo("BRL Sensor Node initialized successfully")

        # Subscribe to Scene Graph (Robot 상태 동기화)
        # NOTE: tomato_detector 초기화 이후에 등록해야 callback에서 사용 가능
        rospy.Subscriber("/scene_graph", SceneGraphMsg, self.scene_graph_callback, queue_size=1)

        # Initialize services (제공하는 서비스는 여기서!)
        self.init_services()

        # Status timer
        rospy.Timer(rospy.Duration(10.0), self.status_timer_callback)

    def load_robot_config(self):
        """YAML 파일에서 robot_type 기반으로 로봇 설정 로딩"""
        robot_type = rospy.get_param("~robot_type", "harvester")

        try:
            # Get config file path using rospkg
            rospack = rospkg.RosPack()
            package_path = rospack.get_path('brl_sensor')
            config_file = os.path.join(package_path, 'config', 'robot_config.yaml')

            # Load YAML
            with open(config_file, 'r', encoding='utf-8') as file:
                config = yaml.safe_load(file)

            # Get robot type config
            robot_type_config = config.get('robot_types', {}).get(robot_type)
            if not robot_type_config:
                raise ValueError(f"Unknown robot_type: {robot_type}")

            # Merge with defaults
            defaults = config.get('default', {})
            return {**defaults, **robot_type_config}

        except Exception as e:
            rospy.logerr(f"Failed to load robot configuration: {e}")
            raise

    def scene_graph_callback(self, msg: SceneGraphMsg):
        """Scene Graph 구독 - Robot 상태 및 위치 동기화"""
        try:
            # ROS msg → Pydantic SceneGraph
            scene = RosConverter.msg_to_scene_graph(msg)

            # TomatoDetector에 SceneGraph 전달 (중복 체크용)
            self.tomato_detector.update_scene_graph(scene)

            # SceneGraph 헬퍼 메서드 활용
            location = scene.get_robot_location(self.robot.name)
            if location:
                updated_robot = location.robots[self.robot.name]

                # Position 제외하고 업데이트 (position은 AMCL이 관리)
                self.robot.status = updated_robot.status
                self.robot.held_objects = updated_robot.held_objects
                self.current_location = location.name  # 위치 정보 저장!

                rospy.logdebug(f"Robot {self.robot.name} synced: location={self.current_location}, status={self.robot.status}")
            else:
                rospy.logwarn_throttle(10, f"Robot {self.robot.name} not found in Scene Graph")

        except Exception as e:
            rospy.logerr(f"Error in scene_graph_callback: {e}")

    def init_robot_service_client(self):
        """Robot update 서비스 클라이언트 초기화"""
        try:
            service_name = "/knowledge_graph_node/robot/update"
            rospy.wait_for_service(service_name, timeout=5.0)
            return rospy.ServiceProxy(service_name, UpdateRobot)
        except rospy.ROSException as e:
            rospy.logerr(f"Failed to connect to robot update service: {e}")
            raise

    def init_object_service_client(self):
        """Object update 서비스 클라이언트 초기화"""
        try:
            service_name = "/knowledge_graph_node/object/update"
            rospy.wait_for_service(service_name, timeout=5.0)
            return rospy.ServiceProxy(service_name, UpdateObject)
        except rospy.ROSException as e:
            rospy.logerr(f"Failed to connect to object update service: {e}")
            raise

    def init_services(self):
        """제공하는 ROS 서비스 초기화"""
        try:
            rospy.Service(
                f"/{self.robot_name}/trigger_sensor_scan",
                TriggerSensorScan,
                self.tomato_detector.sensor_scan_callback
            )
            rospy.Service(
                f"/{self.robot_name}/start_single_tomato_scan",
                TriggerSensorScan,
                lambda req: self.tomato_detector.scan_single_tomato(req, mode="start")
            )
            rospy.Service(
                f"/{self.robot_name}/end_single_tomato_scan",
                TriggerSensorScan,
                lambda req: self.tomato_detector.scan_single_tomato(req, mode="end")
            )
            rospy.Service(
                f"/{self.robot_name}/pixel_to_3d",
                PixelTo3D,
                self.tomato_detector.pixel_to_3d_callback
            )
            rospy.loginfo("Sensor services registered (including pixel_to_3d)")
        except Exception as e:
            rospy.logerr(f"Failed to initialize services: {e}")
            raise

    def status_timer_callback(self, event):
        """주기적 상태 로깅 (AMCL 수신 후에만)"""
        try:
            # AMCL 데이터 수신 전에는 로깅하지 않음
            if not self.robot_tracker._registered_to_kb:
                rospy.loginfo_throttle(30, f"[{self.robot_name}] Waiting for AMCL data...")
                return

            status = self.robot_tracker.get_robot_status()
            rospy.loginfo_throttle(15, f"Status: {status}")
        except Exception as e:
            rospy.logerr(f"Error in status timer: {e}")


def main():
    """메인 함수"""
    try:
        robot_name = rospy.get_param("~robot_name", "brl_robot")

        sensor_node = BRLSensorNode(robot_name=robot_name)
        rospy.loginfo(f"BRL Sensor ready - tracking AMCL pose and updating Knowledge Base")

        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
