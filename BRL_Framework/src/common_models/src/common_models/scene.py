#!/usr/bin/env python3
"""
Scene Models - 계층적 환경 모델

시스템 상태를 표현하는 통일된 계층 구조:
    Environment → Location → (Robot | Object)

공통 필드: name, type, description, status, properties
관계: 단방향 (부모 → 자식), SceneGraph에서 역방향 조회 제공
"""

from typing import Dict, List, Optional, Any, Union
from pydantic import BaseModel, Field
from .base import Position, Pose2D


# ============================================================================
# Object Model
# ============================================================================

class Object(BaseModel):
    """
    환경 내 모든 객체를 표현하는 모델

    위치 규칙:
    - Location에 있을 때: position이 실제 위치
    - Robot이 들고 있을 때: position은 outdated (Robot.position 사용)
    - 컨테이너 정보는 SceneGraph.get_object_container()로 조회
    """
    # Common fields
    name: str = Field(..., description="Unique object identifier")
    type: str = Field(..., description="Object type (e.g., tomato, basket, tool)")
    description: str = Field(default="", description="Object description")
    status: str = Field(default="available", description="Current status: available, held, placed")
    properties: Dict[str, Any] = Field(
        default_factory=dict,
        description="Extended properties (e.g., ripeness, quality, bounding_box)"
    )

    # Specific fields
    position: Position = Field(..., description="3D position (may be outdated if held by robot)")
    index: str = Field(default="", description="Optional index for tracking in BT (not part of core model)")


# ============================================================================
# Robot Model
# ============================================================================

class Robot(BaseModel):
    """
    로봇 모델 (Knowledge Graph + Task Planning 공용)

    주요 필드:
    - position: 2D 포즈 (x, y, theta)
    - capabilities: 수행 가능한 액션 목록
    - held_objects: 현재 들고 있는 객체들
    """
    # Common fields
    name: str = Field(..., description="Unique robot identifier")
    type: str = Field(default="harvester", description="Robot type")
    description: str = Field(default="", description="Robot description")
    status: str = Field(default="active", description="Robot status: active, idle, charging, error")
    properties: Dict[str, Any] = Field(
        default_factory=dict,
        description="Extended properties (e.g., battery, speed, max_load)"
    )

    # Specific fields
    position: Pose2D = Field(..., description="2D pose")
    capabilities: List[str] = Field(
        default_factory=list,
        description="List of high-level actions the robot can perform"
    )
    held_objects: Dict[str, Object] = Field(
        default_factory=dict,
        description="Objects currently held by this robot"
    )

    def can_perform(self, action: str) -> bool:
        """Checks if the robot can perform a given action."""
        return action in self.capabilities

    def pick_up_object(self, obj: Object):
        """Pick up an object."""
        self.held_objects[obj.name] = obj

    def drop_object(self, object_name: str) -> Optional[Object]:
        """Drop an object."""
        return self.held_objects.pop(object_name, None)

    def get_gripper_object(self) -> Optional[Object]:
        """
        Get object in gripper (not in basket).

        Gripper object: status != "in_basket"
        Basket objects: status == "in_basket"

        Returns:
            Object in gripper, or None if gripper is empty
        """
        for obj in self.held_objects.values():
            if obj.status != "in_basket":
                return obj
        return None

    def get_basket_objects(self) -> List[Object]:
        """
        Get all objects in basket.

        Returns:
            List of objects with status == "in_basket"
        """
        return [obj for obj in self.held_objects.values() if obj.status == "in_basket"]

    def is_gripper_empty(self) -> bool:
        """Check if gripper is holding an object."""
        return self.get_gripper_object() is None

    def get_basket_count(self) -> int:
        """Get count of objects in basket."""
        return len(self.get_basket_objects())


# ============================================================================
# Location Model
# ============================================================================

class Location(BaseModel):
    """
    환경 내 특정 위치를 표현하는 모델

    주요 필드:
    - position: 2D 포즈 (x, y, theta)
    - robots: 이 위치에 있는 로봇들
    - objects: 이 위치에 있는 객체들
    - connected_locations: 연결된 위치 이름 목록 (navigation용)
    """
    # Common fields
    name: str = Field(..., description="Unique location identifier")
    type: str = Field(..., description="Location type: stem, dock, charging_station")
    description: str = Field(default="", description="Location description")
    status: str = Field(default="available", description="Occupancy status: available, occupied, reserved")
    properties: Dict[str, Any] = Field(
        default_factory=dict,
        description="Extended properties (e.g., capacity, charging_available)"
    )

    # Specific fields
    position: Pose2D = Field(..., description="2D pose")
    environment_name: Optional[str] = Field(
        default=None,
        description="Parent environment name (for Neo4j reverse lookup)"
    )
    robots: Dict[str, Robot] = Field(
        default_factory=dict,
        description="Robots currently at this location"
    )
    objects: Dict[str, Object] = Field(
        default_factory=dict,
        description="Objects currently at this location"
    )
    connected_locations: List[str] = Field(
        default_factory=list,
        description="List of connected location names for navigation"
    )

    def add_robot(self, robot: Robot):
        """Adds a robot to this location."""
        self.robots[robot.name] = robot

    def remove_robot(self, robot_name: str) -> Optional[Robot]:
        """Removes a robot from this location."""
        robot = self.robots.pop(robot_name, None)
        return robot

    def add_object(self, obj: Object):
        """Adds an object to this location."""
        self.objects[obj.name] = obj

    def remove_object(self, object_name: str) -> Optional[Object]:
        """Removes an object from this location."""
        return self.objects.pop(object_name, None)


# ============================================================================
# Environment Model
# ============================================================================

class Environment(BaseModel):
    """
    Scene Graph의 루트. 전체 운영 영역을 표현

    주요 필드:
    - locations: 환경 내 모든 위치들
    - status: 시스템 상태 (idle, executing 등)

    Note: Environment는 position이 없음 (글로벌 좌표계 정의)
    """
    # Common fields
    name: str = Field(..., description="Unique environment identifier")
    type: str = Field(default="farm", description="Environment type")
    description: str = Field(default="", description="Environment description")
    status: str = Field(default="idle", description="System status: idle, planning, executing, paused, error")
    properties: Dict[str, Any] = Field(
        default_factory=dict,
        description="Extended properties (e.g., farm_size, weather)"
    )

    # Specific fields
    locations: Dict[str, Location] = Field(
        default_factory=dict,
        description="All locations within the environment"
    )

    def add_location(self, location: Location):
        """Adds a location to the environment."""
        self.locations[location.name] = location


# ============================================================================
# SceneGraph Model
# ============================================================================

class SceneGraph(BaseModel):
    """
    시스템 현재 상태의 완전한 계층적 표현. Task Planning의 주요 데이터 구조

    Helper 메서드:
    - get_robot_by_name(), get_object_by_name() - 엔티티 조회
    - get_object_container() - 객체 컨테이너 조회 (Location 또는 Robot)
    - get_all_robots(), get_all_objects() - 집계 조회
    """
    environment: Environment
    timestamp: float = Field(default=0.0, description="Timestamp of when the scene was generated")

    # ========================================================================
    # Helper Methods - Container/Location Lookups (Reverse Direction)
    # ========================================================================

    def get_robot_location(self, robot_name: str) -> Optional[Location]:
        """Find location containing the specified robot."""
        for location in self.environment.locations.values():
            if robot_name in location.robots:
                return location
        return None

    def get_object_container(self, object_name: str) -> Optional[Union[Location, Robot]]:
        """
        Find where the object is located.

        Returns:
            Location: if object is in a location
            Robot: if object is held by a robot
            None: if object not found

        Usage - Getting Object's Actual Position:
            container = scene_graph.get_object_container("tomato_01")
            if isinstance(container, Location):
                # Object in location → use Object.position
                obj = container.objects["tomato_01"]
                actual_position = obj.position  # 3D position
                print(f"At location {container.name}: {actual_position}")
            elif isinstance(container, Robot):
                # Object held by robot → use Robot.position (Object.position outdated)
                actual_position = container.position  # 2D position
                print(f"Held by robot {container.name}: {actual_position}")
            else:
                print("Object not found")
        """
        for location in self.environment.locations.values():
            # Check if in this location
            if object_name in location.objects:
                return location
            # Check if held by any robot at this location
            for robot in location.robots.values():
                if object_name in robot.held_objects:
                    return robot
        return None

    # ========================================================================
    # Helper Methods - Entity Lookups
    # ========================================================================

    def get_robot_by_name(self, name: str) -> Optional[Robot]:
        """Find and return a robot by its unique name."""
        for location in self.environment.locations.values():
            if name in location.robots:
                return location.robots[name]
        return None

    def get_object_by_name(self, name: str) -> Optional[Object]:
        """Find and return an object by its unique name (in locations or held by robots)."""
        # Search in locations
        for location in self.environment.locations.values():
            if name in location.objects:
                return location.objects[name]
            # Search in robots at this location
            for robot in location.robots.values():
                if name in robot.held_objects:
                    return robot.held_objects[name]
        return None

    def get_location_by_name(self, name: str) -> Optional[Location]:
        """Find and return a location by its unique name."""
        return self.environment.locations.get(name)

    # ========================================================================
    # Helper Methods - Aggregate Queries
    # ========================================================================

    def get_all_robots(self) -> List[Robot]:
        """Returns a list of all robots in the entire scene."""
        return [
            robot
            for location in self.environment.locations.values()
            for robot in location.robots.values()
        ]

    def get_all_objects(self) -> List[Object]:
        """Returns a list of all objects in the entire scene (from locations and held by robots)."""
        objects = []
        for location in self.environment.locations.values():
            # Objects in location
            objects.extend(location.objects.values())
            # Objects held by robots at this location
            for robot in location.robots.values():
                objects.extend(robot.held_objects.values())
        return objects

    def get_all_locations(self) -> List[Location]:
        """Returns a list of all locations in the entire scene."""
        return list(self.environment.locations.values())

    def get_robots_at_location(self, location_name: str) -> List[Robot]:
        """Get all robots at a specific location."""
        location = self.get_location_by_name(location_name)
        return list(location.robots.values()) if location else []

    def get_objects_at_location(self, location_name: str) -> List[Object]:
        """Get all objects at a specific location (not including held objects)."""
        location = self.get_location_by_name(location_name)
        return list(location.objects.values()) if location else []

    # ========================================================================
    # Convenience Helper Methods
    # ========================================================================

    def is_object_held(self, object_name: str) -> bool:
        """Check if object is held by any robot."""
        container = self.get_object_container(object_name)
        return isinstance(container, Robot)

    def is_object_in_location(self, object_name: str) -> bool:
        """Check if object is in a location (not held)."""
        container = self.get_object_container(object_name)
        return isinstance(container, Location)

    # ========================================================================
    # String Helper Methods (for prompt generation)
    # ========================================================================

    def get_robot_location_name(self, robot_name: str) -> str:
        """
        Get the location name where the robot is located.

        Returns:
            Location name or 'unknown' if robot not found
        """
        location = self.get_robot_location(robot_name)
        return location.name if location else "unknown"

    def get_object_location_str(self, object_name: str) -> str:
        """
        Get a human-readable location description for an object.

        Returns:
            'at {location_name}' - if object is in a location
            'held by {robot_name}' - if object is held by a robot
            'unknown' - if object not found
        """
        container = self.get_object_container(object_name)
        if isinstance(container, Robot):
            return f"held by {container.name}"
        elif isinstance(container, Location):
            return f"at {container.name}"
        return "unknown"
