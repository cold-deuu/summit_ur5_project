#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import json
from pathlib import Path
import rospkg

from knowledge_interface.scene_graph import SceneManager
from common_models import Environment, Location, Pose2D
from common_models.converters import RosConverter

from brl_interfaces.srv import (
    UpdateRobot, UpdateRobotResponse,
    UpdateObject, UpdateObjectResponse,
    UpdateEnvironment, UpdateEnvironmentResponse
)
from brl_interfaces.msg import SceneGraphMsg

class KnowledgeGraphNode:    
    def __init__(self):
        rospy.init_node('knowledge_graph_node')
        
        rospy.loginfo("In-memory Scene Graph Node Initializing...")

        # --- Scene Graph System ---
        self.scene_manager = SceneManager()
        package_path = Path(rospkg.RosPack().get_path('knowledge_interface'))
        config_path = rospy.get_param('~environment_config', str(package_path / 'config/real_farm_config.json'))
        environment = self.load_environment_config(config_path)
        if not self.scene_manager.initialize(environment):
            raise RuntimeError("Scene Graph initialization failed")

        # --- Scene Graph Publisher ---
        self.scene_graph_pub = rospy.Publisher('/scene_graph', SceneGraphMsg, queue_size=1)

        # Scene Graph 주기적 발행을 위한 타이머 (5Hz)
        self.scene_graph_timer = rospy.Timer(rospy.Duration(0.2), self.scene_graph_timer_callback)

        # --- ROS Services ---
        self.init_services()

        self.publish_scene_graph()
        rospy.loginfo("In-memory Scene Graph ready: %s", config_path)

    @staticmethod
    def load_environment_config(config_path):
        """Load the unchanged 01 farm configuration directly into memory."""
        with Path(config_path).expanduser().open(encoding='utf-8') as stream:
            config = json.load(stream)
        env_config = config['environment']
        environment = Environment(
            name=env_config['name'],
            type=env_config['type'],
            description=env_config.get('description', ''),
            status=env_config.get('status', 'active'),
            properties=env_config.get('properties', {}),
        )
        for item in config['locations']:
            x, y, theta = item['position']
            location = Location(
                name=item['name'],
                type=item['type'],
                description=item.get('description', ''),
                status=item.get('status', 'available'),
                position=Pose2D(x=x, y=y, theta=theta),
                connected_locations=item.get('connected_to', []),
                environment_name=environment.name,
                properties=item.get('properties', {}),
            )
            environment.add_location(location)
        if not environment.locations:
            raise ValueError("Environment config must contain at least one location")
        return environment

    def init_services(self):
        """Initialize all ROS services."""
        # Environment update service
        rospy.Service('~environment/update', UpdateEnvironment, self.update_environment_callback)

        # Robot update service
        rospy.Service('~robot/update', UpdateRobot, self.update_robot_callback)

        # Object update service
        rospy.Service('~object/update', UpdateObject, self.update_object_callback)

    def update_environment_callback(self, req: UpdateEnvironment) -> UpdateEnvironmentResponse:
        """Environment status 업데이트"""
        try:
            new_status = req.status

            if not new_status:
                return UpdateEnvironmentResponse(success=False, message="Status is required")

            success = self.scene_manager.update_environment_status(new_status)

            if success:
                rospy.loginfo(f"Environment status updated to '{new_status}'")
                return UpdateEnvironmentResponse(
                    success=True,
                    message=f"Environment status updated to {new_status}"
                )
            else:
                return UpdateEnvironmentResponse(
                    success=False,
                    message=f"Failed to update environment status"
                )

        except Exception as e:
            rospy.logerr(f"Environment update failed: {e}")
            return UpdateEnvironmentResponse(success=False, message=str(e))

    def update_robot_callback(self, req: UpdateRobot) -> UpdateRobotResponse:
        """Robot 업데이트 - Converter 사용 + Scene Manager에 위임"""
        try:
            robot_msg = req.robot_data

            if not robot_msg.name:
                rospy.logwarn("Robot name is required for update.")
                return UpdateRobotResponse(success=False, message="Robot name is required")

            # ROS Message → Pydantic Robot (Converter 사용!)
            robot = RosConverter.msg_to_robot(robot_msg)

            # Scene Manager에 위임 (merge, location 자동 할당 모두 처리)
            success = self.scene_manager.update_robot(robot)

            if success:
                rospy.loginfo(f"Robot '{robot.name}' updated successfully")
                # Immediately publish scene_graph for BT sync
                self.publish_scene_graph()
                return UpdateRobotResponse(
                    success=True,
                    message=f"Robot {robot.name} updated successfully"
                )
            else:
                return UpdateRobotResponse(
                    success=False,
                    message=f"Failed to update robot {robot.name}"
                )

        except Exception as e:
            rospy.logerr(f"Robot update failed: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())
            return UpdateRobotResponse(success=False, message=str(e))

    def update_object_callback(self, req: UpdateObject) -> UpdateObjectResponse:
        """
        Object 업데이트 - Converter 사용 + Scene Manager에 위임

        Note:
            - current_location_name과 held_by_robot_name은 상호 배타적
            - location_name이 있으면: obj.position 사용 (센서/BT가 업데이트)
            - held_by가 있으면: obj.position 무시 (Robot 위치 사용)
        """
        try:
            object_msg = req.object_data

            if not object_msg.name:
                rospy.logwarn("Object name is required for update.")
                return UpdateObjectResponse(success=False, message="Object name is required")

            # Handle deletion request (status="deleted")
            if object_msg.status == "deleted":
                success = self.scene_manager.delete_object(object_msg.name)
                if success:
                    rospy.loginfo(f"Object '{object_msg.name}' deleted successfully")
                    # Immediately publish scene_graph for BT sync
                    self.publish_scene_graph()
                    return UpdateObjectResponse(
                        success=True,
                        message=f"Object {object_msg.name} deleted successfully"
                    )
                else:
                    return UpdateObjectResponse(
                        success=False,
                        message=f"Failed to delete object {object_msg.name}"
                    )

            # ROS Message → Pydantic Object (Converter 사용!)
            obj = RosConverter.msg_to_object(object_msg)
            held_by=object_msg.held_by_robot_name if object_msg.held_by_robot_name else ""
            location_name=object_msg.current_location_name if object_msg.current_location_name else ""
            rospy.loginfo(f"[KB] Index: {obj.index}")

            # Scene Manager에 위임 (location, held_by 메타데이터 전달)
            success = self.scene_manager.update_object(
                obj,
                location_name=location_name,
                held_by=held_by
            )

            if success:
                rospy.loginfo(f"[KB] Object '{obj.name}' with property {obj.properties['tomato_class']} updated successfully")
                # Immediately publish scene_graph for BT sync
                self.publish_scene_graph()
                return UpdateObjectResponse(
                    success=True,
                    message=f"Object {obj.name} updated successfully"
                )
            else:
                return UpdateObjectResponse(
                    success=False,
                    message=f"Failed to update object {obj.name}"
                )

        except Exception as e:
            rospy.logerr(f"Object update failed: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())
            return UpdateObjectResponse(success=False, message=str(e))
    

    def publish_scene_graph(self):
        """현재 Scene Graph를 토픽으로 발행 (RosConverter 사용)"""
        try:
            self.scene_manager.update_timestamp()
            scene_graph = self.scene_manager.get_scene_graph()
            if scene_graph:
                msg = RosConverter.scene_graph_to_msg(scene_graph)
                self.scene_graph_pub.publish(msg)
                rospy.logdebug(f"Scene Graph published: {msg.total_robots} robots, {msg.total_objects} objects")
            else:
                rospy.logdebug("No scene graph available to publish")
        # Exception
        except Exception as e:
            rospy.logerr(f"Failed to publish scene graph: {e}")

    def scene_graph_timer_callback(self, event):
        """주기적으로 Scene Graph 발행하는 타이머 콜백"""
        self.publish_scene_graph()


def main():
    """메인 함수"""
    try:
        node = KnowledgeGraphNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
