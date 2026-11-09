"""
BRL Project Common Models

패키지 간 데이터 교환을 위한 공통 Pydantic 모델 제공

모듈 구조:
- base: Position, Pose2D (기본 좌표)
- scene: Robot, Object, Location, Environment, SceneGraph (계층적 환경 모델)
- planning: ActionType, BaseAction, RobotPlan 등 (Task Planning용)
- converters: RosConverter (Pydantic ↔ ROS msg 변환)
"""

from .base import Position, Pose2D
from .scene import Robot, Object, Location, Environment, SceneGraph
from .planning import LocationReasoning, SceneGraphReasoning, ActionType, BaseAction, RobotPlan
from .converters import RosConverter

__all__ = [
    # base.py - 기본 좌표
    "Position",
    "Pose2D",

    # scene.py - 환경 모델
    "Robot",
    "Object",
    "Location",
    "Environment",
    "SceneGraph",

    # planning.py - Task Planning
    "LocationReasoning",
    "SceneGraphReasoning",
    "ActionType",
    "BaseAction",
    "RobotPlan",

    # converters.py - ROS 변환
    "RosConverter",
]
