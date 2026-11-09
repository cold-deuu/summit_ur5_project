"""
Scene Manager - Scene Graph 빌드 및 실시간 업데이트 관리

실시간 센서 데이터를 기반으로 Scene Graph를 생성하고 업데이트합니다.
- 외부에서 Environment를 받아 초기화
- Robot/Object 서비스를 통한 실시간 업데이트
- 메모리 기반 고성능 Scene Graph 관리
- Neo4j를 몰라도 됨 (Synchronizer가 담당)
"""

import rospy
import time
from typing import Dict, List, Optional, Any
from threading import Lock
import copy

from common_models import (
    SceneGraph, Environment, Location, Robot, Object, Position, Pose2D
)


class SceneManager:
    """Scene Graph 빌드 및 실시간 업데이트 관리자"""

    def __init__(self):
        """
        Neo4j에 의존하지 않는 순수 메모리 기반 Scene Graph 관리자
        """
        self.scene_graph: Optional[SceneGraph] = None
        self.lock = Lock()  # 스레드 안전성
        
        # Changmin add: Object confidence threshold
        self.confidence_threshold = 0.8


        rospy.loginfo("SceneManager initialized")
    
    def initialize(self, environment: Environment) -> bool:
        """
        외부에서 Environment를 받아서 Scene Graph 초기화
        (Neo4j를 몰라도 됨 - Synchronizer가 Environment 제공)

        Args:
            environment: 초기화할 Environment (Locations 포함)

        Returns:
            bool: 초기화 성공 여부
        """
        try:
            with self.lock:
                rospy.loginfo("Initializing Scene Graph...")

                # Scene Graph 생성 (Robot, Object는 빈 상태)
                self.scene_graph = SceneGraph(
                    environment=environment,
                    timestamp=time.time()
                )

                rospy.loginfo(f"Scene Graph initialized: {len(environment.locations)} locations")
                return True

        except Exception as e:
            rospy.logerr(f"Failed to initialize Scene Graph: {e}")
            return False
    
    def update_robot(self, new_robot: Robot) -> bool:
        """
        Robot 업데이트 (기존 로봇 merge + 위치 자동 할당)

        Args:
            new_robot: 새로 받은 Robot 객체 (일부 필드가 비어있을 수 있음)

        Returns:
            성공 여부 (True/False)

        Note:
            Location은 항상 robot.position 기반으로 자동 계산됩니다.
        """
        try:
            with self.lock:
                if not self.scene_graph:
                    rospy.logwarn("Scene Graph not initialized")
                    return False

                # 1. 기존 로봇 찾기 (SceneGraph 헬퍼 메서드 사용)
                existing_robot = self.scene_graph.get_robot_by_name(new_robot.name)

                # 2. Merge (기존 로봇이 있으면)
                if existing_robot:
                    merged_robot = Robot(
                        name=new_robot.name,
                        type=new_robot.type if new_robot.type else existing_robot.type,
                        position=new_robot.position,  # position은 항상 업데이트
                        status=new_robot.status if new_robot.status else existing_robot.status,
                        capabilities=new_robot.capabilities if new_robot.capabilities else existing_robot.capabilities,
                        held_objects=existing_robot.held_objects,  # 기존 held_objects 유지
                        description=new_robot.description if new_robot.description else existing_robot.description,
                        properties=existing_robot.properties
                    )
                else:
                    # 새 로봇 (기본값 설정)
                    merged_robot = Robot(
                        name=new_robot.name,
                        type=new_robot.type if new_robot.type else "unknown",
                        position=new_robot.position,
                        status=new_robot.status if new_robot.status else "active",
                        capabilities=new_robot.capabilities if new_robot.capabilities else [],
                        held_objects={},
                        description=new_robot.description,
                        properties={}
                    )
                    rospy.loginfo(f"Creating new robot: {new_robot.name}")

                # 3. Location 자동 할당 (항상 현재 position 기반으로 계산)
                target_location_name = self._find_closest_location(merged_robot.position)

                # 4. 기존 위치에서 제거 (위치가 바뀌었으면)
                if existing_robot:
                    old_location = self.scene_graph.get_robot_location(new_robot.name)
                    if old_location and old_location.name != target_location_name:
                        old_location.remove_robot(new_robot.name)
                        rospy.logdebug(f"Removed {new_robot.name} from {old_location.name}")

                # 5. 새 위치에 추가
                target_location = self.scene_graph.get_location_by_name(target_location_name)
                if not target_location:
                    rospy.logerr(f"Target location {target_location_name} not found!")
                    return False

                target_location.add_robot(merged_robot)
                rospy.logdebug(f"Robot {merged_robot.name} updated at {target_location_name}")

                return True

        except Exception as e:
            rospy.logerr(f"Failed to update robot {new_robot.name}: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())
            return False

    def _find_closest_location(self, position: Pose2D) -> str:
        """가장 가까운 Location 찾기 (knowledge_graph_node에서 이동)"""
        min_distance = float('inf')
        closest_location_name = None

        for location in self.scene_graph.get_all_locations():
            distance = position.distance_2d(location.position)
            if distance < min_distance:
                min_distance = distance
                closest_location_name = location.name

        return closest_location_name if closest_location_name else "unknown"
    
    # Original method
    def update_object(self, obj: Object, location_name: str = "", held_by: str = "") -> bool:
        """
        Object 업데이트 (location 또는 robot에 배치)

        Args:
            obj: Object 인스턴스
            location_name: Object가 위치한 Location 이름
            held_by: Object를 들고 있는 Robot 이름 (location_name과 상호 배타적)

        Returns:
            성공 여부 (True/False)

        Object Position Handling:
            - When location_name is provided: obj.position is used (updated by sensor/BT)
            - When held_by is provided: obj.position is ignored (may be outdated)
            - Actual position when held: use Robot.position from container

        Usage Examples:
            # Sensor scan (object discovered at location)
            update_object(obj, location_name="Stem_01", held_by="")

            # Robot picks object
            update_object(obj, location_name="", held_by="harvester_01")

            # Robot places object (position updated in obj)
            update_object(obj, location_name="Dock_Station", held_by="")
        """
        try:
            with self.lock:
                if not self.scene_graph:
                    rospy.logwarn("Scene Graph not initialized")
                    return False
                
                # 1. 기존 Object 제거 (모든 location과 robot에서)
                self._remove_object_from_all_locations(obj.name)
                self._remove_object_from_all_robots(obj.name)

                # 2. Object 배치 (held_by 우선, 없으면 location)
                if held_by:
                    # Robot이 들고 있음
                    target_robot = self.scene_graph.get_robot_by_name(held_by)
                    if not target_robot:
                        rospy.logerr(f"Robot '{held_by}' not found for object '{obj.name}'")
                        return False

                    target_robot.pick_up_object(obj)
                    rospy.logdebug(f"Object '{obj.name}' held by robot '{held_by}'")
                    return True

                elif location_name:  # Location에 토마토 배치하는 로직
                    # Location에 배치
                    target_location = self.scene_graph.get_location_by_name(location_name)
                    if not target_location:
                        rospy.logerr(f"Location '{location_name}' not found for object '{obj.name}'")
                        return False
                    

                    target_location.add_object(obj)
                    rospy.logdebug(f"Object '{obj.name}' placed at '{location_name}'")
                    return True

                else:
                    rospy.logwarn(f"No location or holder specified for object '{obj.name}'")
                    return False
                
        except Exception as e:
            rospy.logerr(f"Failed to update object in Scene Graph: {e}")
            return False
        
    
    def _remove_robot_from_all_locations(self, robot_name: str):
        """모든 Location에서 Robot 제거"""
        if not self.scene_graph:
            return
        
        for location in self.scene_graph.environment.locations.values():
            if robot_name in location.robots:
                location.remove_robot(robot_name)
    
    def _remove_object_from_all_locations(self, object_name: str):
        """모든 Location에서 Object 제거"""
        if not self.scene_graph:
            return

        for location in self.scene_graph.environment.locations.values():
            if object_name in location.objects:
                location.remove_object(object_name)

    def _remove_object_from_all_robots(self, object_name: str):
        """모든 Robot에서 Object 제거"""
        if not self.scene_graph:
            return

        for location in self.scene_graph.environment.locations.values():
            for robot in location.robots.values():
                if object_name in robot.held_objects:
                    robot.drop_object(object_name)

    def delete_object(self, object_name: str) -> bool:
        """
        Object 삭제 (Scene Graph에서 완전히 제거)

        Args:
            object_name: 삭제할 Object 이름

        Returns:
            성공 여부 (True/False)
        """
        try:
            with self.lock:
                if not self.scene_graph:
                    rospy.logwarn("Scene Graph not initialized")
                    return False

                # 모든 곳에서 Object 제거
                self._remove_object_from_all_locations(object_name)
                self._remove_object_from_all_robots(object_name)

                rospy.loginfo(f"Object '{object_name}' deleted from Scene Graph")
                return True

        except Exception as e:
            rospy.logerr(f"Failed to delete object '{object_name}': {e}")
            return False

    def get_scene_graph(self) -> Optional[SceneGraph]:
        """현재 Scene Graph 반환 (깊은 복사)"""
        with self.lock:
            if self.scene_graph:
                return copy.deepcopy(self.scene_graph)
            return None
    
    def get_scene_graph_summary(self) -> Dict[str, Any]:
        """Scene Graph 요약 정보 반환"""
        with self.lock:
            if not self.scene_graph:
                return {"environment": None, "locations": 0, "robots": 0, "objects": 0}
            
            total_robots = sum(len(loc.robots) for loc in self.scene_graph.environment.locations.values())
            total_objects = sum(len(loc.objects) for loc in self.scene_graph.environment.locations.values())
            
            return {
                "environment": self.scene_graph.environment.name,
                "locations": len(self.scene_graph.environment.locations),
                "robots": total_robots,
                "objects": total_objects
            }
    
    def is_initialized(self) -> bool:
        """Scene Graph 초기화 여부 확인"""
        with self.lock:
            return self.scene_graph is not None
    
    def update_location(self, location_data: Dict[str, Any]) -> bool:
        """Scene Graph에 Location 업데이트 (추가 또는 수정)"""
        try:
            with self.lock:
                if not self.scene_graph:
                    rospy.logwarn("Scene Graph not initialized")
                    return False
                
                # Location 객체 생성
                location = Location(
                    name=location_data["name"],
                    type=location_data["type"],
                    position=Pose2D(
                        x=location_data["position"]["x"],
                        y=location_data["position"]["y"],
                        theta=location_data["position"]["theta"]
                    ),
                    environment_name=location_data["environment_name"],
                    connected_locations=location_data.get("connected_locations", [])
                )
                
                # 기존 Location이 있으면 Robot/Object 정보 유지
                if location.name in self.scene_graph.environment.locations:
                    existing_location = self.scene_graph.environment.locations[location.name]
                    location.robots = existing_location.robots
                    location.objects = existing_location.objects
                
                # Environment에 Location 추가/업데이트
                self.scene_graph.environment.locations[location.name] = location
                location.environment_name = self.scene_graph.environment.name
                
                # 타임스탬프 업데이트
                self.scene_graph.timestamp = time.time()
                
                rospy.loginfo(f"Location '{location.name}' updated in Scene Graph")
                return True
                
        except Exception as e:
            rospy.logerr(f"Failed to update location in Scene Graph: {e}")
            return False
    
    def update_timestamp(self):
        """Scene Graph 타임스탬프 업데이트"""
        with self.lock:
            if self.scene_graph:
                self.scene_graph.timestamp = time.time()

    def update_environment_status(self, status: str) -> bool:
        """
        Environment status 업데이트

        Args:
            status: 새로운 상태 (e.g., "idle", "task_planning", "executing")

        Returns:
            성공 여부 (True/False)
        """
        try:
            with self.lock:
                if not self.scene_graph:
                    rospy.logwarn("Scene Graph not initialized")
                    return False

                self.scene_graph.environment.status = status
                self.scene_graph.timestamp = time.time()
                rospy.logdebug(f"Environment status updated to '{status}'")
                return True

        except Exception as e:
            rospy.logerr(f"Failed to update environment status: {e}")
            return False

    # ========================================================================
    # Synchronizer용 캡슐화 메서드 (내부 구조 노출 방지)
    # ========================================================================

    def get_all_current_robots(self) -> Dict[str, Robot]:
        """
        현재 모든 Robot 반환 (Synchronizer가 내부 구조 몰라도 됨)

        Returns:
            Dict[str, Robot]: 모든 Robot (location 상관없이)
        """
        with self.lock:
            if not self.scene_graph:
                return {}

            robots = {}
            for location in self.scene_graph.environment.locations.values():
                robots.update(location.robots)
            return robots

    def get_all_current_objects(self) -> Dict[str, Object]:
        """
        현재 모든 Object 반환 (Synchronizer가 내부 구조 몰라도 됨)

        Returns:
            Dict[str, Object]: 모든 Object
        """
        with self.lock:
            if not self.scene_graph:
                return {}

            objects = {}
            for location in self.scene_graph.environment.locations.values():
                objects.update(location.objects)
            return objects