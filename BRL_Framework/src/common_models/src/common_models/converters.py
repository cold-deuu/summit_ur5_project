#!/usr/bin/env python3
"""
ROS Converters - Pydantic ↔ ROS msg 변환

변환 방향:
- Pydantic → ROS msg: 발행/서비스 응답용
- ROS msg → Pydantic: 구독/서비스 요청용

Pydantic 모델이 primary (타입 안전, 풍부한 기능)
ROS 메시지는 transport용 (flat, serializable)
"""

from typing import Dict, Any, List
import json

from .scene import Robot, Object, Location, Environment, SceneGraph
from .base import Position, Pose2D

try:
    from brl_interfaces.msg import (
        Robot as RobotMsg,
        Object as ObjectMsg,
        Location as LocationMsg,
        Environment as EnvironmentMsg,
        SceneGraphMsg
    )
    from geometry_msgs.msg import Pose2D as RosPose2D, Point
except ImportError:
    # Allow import without ROS for testing
    pass


class RosConverter:
    """Bidirectional converter between Pydantic models and ROS messages."""

    # ========================================================================
    # Pydantic → ROS msg (for publishing)
    # ========================================================================

    @staticmethod
    def environment_to_msg(env: Environment) -> EnvironmentMsg:
        """Convert Environment Pydantic model to ROS message."""
        msg = EnvironmentMsg()
        msg.name = env.name
        msg.type = env.type
        msg.description = env.description
        msg.status = env.status
        msg.properties = RosConverter._dict_to_properties(env.properties)
        return msg

    @staticmethod
    def location_to_msg(loc: Location, environment_name: str = "") -> LocationMsg:
        """
        Convert Location Pydantic model to ROS message.

        Args:
            loc: Location Pydantic model
            environment_name: Parent environment name (for hierarchical reference)
        """
        msg = LocationMsg()
        msg.name = loc.name
        msg.type = loc.type
        msg.description = loc.description
        msg.status = loc.status
        msg.position = RosPose2D(x=loc.position.x, y=loc.position.y, theta=loc.position.theta)
        msg.environment_name = environment_name
        msg.connected_locations = loc.connected_locations
        msg.properties = RosConverter._dict_to_properties(loc.properties)
        return msg

    @staticmethod
    def robot_to_msg(robot: Robot, current_location_name: str = "") -> RobotMsg:
        """
        Convert Robot Pydantic model to ROS message.

        Args:
            robot: Robot Pydantic model
            current_location_name: Parent location name (for hierarchical reference)
        """
        msg = RobotMsg()
        msg.name = robot.name
        msg.type = robot.type
        msg.description = robot.description
        msg.status = robot.status
        msg.position = RosPose2D(x=robot.position.x, y=robot.position.y, theta=robot.position.theta)
        msg.current_location_name = current_location_name
        msg.capabilities = robot.capabilities
        # held_objects: Dict[str, Object] → List[str] (names only)
        msg.held_objects = list(robot.held_objects.keys())
        msg.properties = RosConverter._dict_to_properties(robot.properties)
        return msg

    @staticmethod
    def object_to_msg(obj: Object, current_location_name: str = "", held_by_robot_name: str = "") -> ObjectMsg:
        """
        Convert Object Pydantic model to ROS message.

        Args:
            obj: Object Pydantic model
            current_location_name: Parent location name (if in a location)
            held_by_robot_name: Holder robot name (if held by a robot)

        Note: current_location_name and held_by_robot_name are mutually exclusive
        """
        msg = ObjectMsg()
        msg.name = obj.name
        msg.type = obj.type
        msg.description = obj.description
        msg.status = obj.status
        msg.position = Point(x=obj.position.x, y=obj.position.y, z=obj.position.z)
        msg.current_location_name = current_location_name
        msg.held_by_robot_name = held_by_robot_name
        msg.properties = RosConverter._dict_to_properties(obj.properties)
        msg.index = obj.index  # Optional field for tracking in BT (not part of core model)
        return msg

    @staticmethod
    def scene_graph_to_msg(scene: SceneGraph) -> SceneGraphMsg:
        """
        Convert SceneGraph Pydantic model to ROS message.

        Flattens the hierarchical structure into separate arrays,
        but preserves hierarchy information via reference fields.
        """
        msg = SceneGraphMsg()

        # Environment
        msg.environment = RosConverter.environment_to_msg(scene.environment)

        # Locations (flat list with environment_name reference)
        msg.locations = [
            RosConverter.location_to_msg(loc, environment_name=scene.environment.name)
            for loc in scene.environment.locations.values()
        ]

        # Robots (flat list with current_location_name reference)
        msg.robots = []
        for loc_name, loc in scene.environment.locations.items():
            for robot in loc.robots.values():
                robot_msg = RosConverter.robot_to_msg(robot, current_location_name=loc_name)
                msg.robots.append(robot_msg)

        # Objects (flat list with location/robot references)
        msg.objects = []
        for loc_name, loc in scene.environment.locations.items():
            # Objects in location
            for obj in loc.objects.values():
                obj_msg = RosConverter.object_to_msg(
                    obj,
                    current_location_name=loc_name,
                    held_by_robot_name=""
                )
                msg.objects.append(obj_msg)

            # Objects held by robots
            for robot in loc.robots.values():
                for obj in robot.held_objects.values():
                    obj_msg = RosConverter.object_to_msg(
                        obj,
                        current_location_name="",
                        held_by_robot_name=robot.name
                    )
                    msg.objects.append(obj_msg)

        # Metadata
        msg.timestamp = scene.timestamp
        msg.total_locations = len(msg.locations)
        msg.total_robots = len(msg.robots)
        msg.total_objects = len(msg.objects)

        return msg

    # ========================================================================
    # ROS msg → Pydantic (for subscribing)
    # ========================================================================

    @staticmethod
    def msg_to_environment(msg: EnvironmentMsg) -> Environment:
        """Convert Environment ROS message to Pydantic model."""
        return Environment(
            name=msg.name,
            type=msg.type,
            description=msg.description,
            status=msg.status,
            properties=RosConverter._properties_to_dict(msg.properties)
        )

    @staticmethod
    def msg_to_location(msg: LocationMsg) -> Location:
        """Convert Location ROS message to Pydantic model."""
        return Location(
            name=msg.name,
            type=msg.type,
            description=msg.description,
            status=msg.status,
            position=Pose2D(x=msg.position.x, y=msg.position.y, theta=msg.position.theta),
            connected_locations=list(msg.connected_locations),
            properties=RosConverter._properties_to_dict(msg.properties)
        )

    @staticmethod
    def msg_to_robot(msg: RobotMsg) -> Robot:
        """
        Convert Robot ROS message to Pydantic model.

        Note: held_objects will be empty Dict initially.
        Use SceneGraph to populate with actual Object instances.
        """
        return Robot(
            name=msg.name,
            type=msg.type,
            description=msg.description,
            status=msg.status,
            position=Pose2D(x=msg.position.x, y=msg.position.y, theta=msg.position.theta),
            capabilities=list(msg.capabilities),
            held_objects={},  # Will be populated when building SceneGraph
            properties=RosConverter._properties_to_dict(msg.properties)
        )

    @staticmethod
    def msg_to_object(msg: ObjectMsg) -> Object:
        """Convert Object ROS message to Pydantic model."""
        return Object(
            name=msg.name,
            type=msg.type,
            description=msg.description,
            status=msg.status,
            position=Position(x=msg.position.x, y=msg.position.y, z=msg.position.z),
            properties=RosConverter._properties_to_dict(msg.properties),
            index=msg.index
        )

    @staticmethod
    def msg_to_scene_graph(msg: SceneGraphMsg) -> SceneGraph:
        """
        Convert SceneGraphMsg to SceneGraph Pydantic model.

        Rebuilds the hierarchical structure from flat arrays using reference fields.

        Steps:
        1. Create environment
        2. Create all locations and add to environment
        3. Create all objects (map by name)
        4. Create all robots and assign to locations
        5. Assign objects to locations or robots based on reference fields
        """
        # Create environment
        environment = RosConverter.msg_to_environment(msg.environment)

        # Create all locations and add to environment
        location_map: Dict[str, Location] = {}
        for loc_msg in msg.locations:
            loc = RosConverter.msg_to_location(loc_msg)
            location_map[loc.name] = loc
            environment.add_location(loc)

        # Create all objects (indexed by name)
        all_objects: Dict[str, Object] = {}
        for obj_msg in msg.objects:
            obj = RosConverter.msg_to_object(obj_msg)
            all_objects[obj.name] = obj

        # Create all robots and assign to locations
        for robot_msg in msg.robots:
            robot = RosConverter.msg_to_robot(robot_msg)

            # Populate held_objects using reference field
            for obj_name in robot_msg.held_objects:
                if obj_name in all_objects:
                    robot.held_objects[obj_name] = all_objects[obj_name]

            # Assign robot to location using reference field
            if robot_msg.current_location_name in location_map:
                location_map[robot_msg.current_location_name].add_robot(robot)

        # Assign objects to locations or robots using reference fields
        for obj_msg in msg.objects:
            obj = all_objects[obj_msg.name]

            if obj_msg.held_by_robot_name:
                # Object is held by robot - already added to robot.held_objects above
                pass
            elif obj_msg.current_location_name and obj_msg.current_location_name in location_map:
                # Object is in a location
                location_map[obj_msg.current_location_name].add_object(obj)

        # Create scene graph
        scene_graph = SceneGraph(
            environment=environment,
            timestamp=msg.timestamp
        )

        return scene_graph

    # ========================================================================
    # Helper Methods - Properties Conversion
    # ========================================================================

    @staticmethod
    def _dict_to_properties(d: Dict[str, Any]) -> List[str]:
        """
        Convert Dict to string[] for ROS message properties.

        Format: "key:value"
        - Simple types: "battery:80", "status:active"
        - Complex types (list/dict): "bbox:[1,2,3,4]" (JSON encoded)

        Args:
            d: Dictionary to convert

        Returns:
            List of "key:value" strings
        """
        result = []
        for key, value in d.items():
            if isinstance(value, (list, dict)):
                # JSON encode complex types
                result.append(f"{key}:{json.dumps(value)}")
            elif isinstance(value, bool):
                # Boolean as lowercase string
                result.append(f"{key}:{str(value).lower()}")
            else:
                # Simple types as string
                result.append(f"{key}:{value}")
        return result

    @staticmethod
    def _properties_to_dict(props: List[str]) -> Dict[str, Any]:
        """
        Convert string[] from ROS message to Dict.

        Attempts to parse values intelligently:
        - JSON arrays/objects: Parsed to list/dict
        - Numbers: Converted to int/float
        - Booleans: Converted to bool
        - Everything else: Kept as string

        Args:
            props: List of "key:value" strings

        Returns:
            Dictionary with parsed values
        """
        result = {}
        for prop in props:
            if ':' not in prop:
                continue

            key, value = prop.split(':', 1)

            # Try JSON parsing (for lists/dicts)
            try:
                parsed = json.loads(value)
                result[key] = parsed
                continue
            except (json.JSONDecodeError, ValueError):
                pass

            # Try boolean
            if value.lower() in ('true', 'false'):
                result[key] = value.lower() == 'true'
                continue

            # Try number
            try:
                if '.' in value:
                    result[key] = float(value)
                else:
                    result[key] = int(value)
                continue
            except ValueError:
                pass

            # Keep as string
            result[key] = value

        return result
