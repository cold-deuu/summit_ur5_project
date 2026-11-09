#!/usr/bin/env python3
"""
Planning Models - Task Planning 관련 모델

LLM 기반 Task Planning에서 사용하는 데이터 구조:
- ActionType, BaseAction: 로봇 액션 정의
- SceneGraphReasoning: Preprocessing 결과 (instruction 분석)
- RobotPlan: LLM 출력 (계획된 액션 리스트)
"""

from enum import Enum
from typing import Dict, List, Optional, Any
from pydantic import BaseModel, Field

# --- Action Types ---
class ActionType(str, Enum):
    """Enum for different action types."""
    NAVIGATE = "Navigate"
    HARVEST = "Harvest"

    @property
    def description(self) -> str:
        return {
            "NAVIGATE": "Navigate to a specified location.",
            "HARVEST": "Harvest tomatoes from the current location."
        }[self.value]
    
class BaseAction(BaseModel):
    """
    Base Action for integrating the action model.
    """
    type: ActionType = Field(..., description="Action type identifier (must match defined types)")
    robot_id: str = Field(..., description="ID of the robot performing the action.")
    from_loc: Optional[str] = Field(..., description="Location ID where the action starts.")
    to_loc: Optional[str] = Field(..., description="Location ID where the action ends.")
    at_loc: Optional[str] = Field(..., description="Location ID where the robot is currently located.")

    def get_name(self) -> str:
        return self.type.value
    

# --- Reasoning Models ---
class LocationReasoning(BaseModel):
    """개별 location 선택 근거"""
    location_name: str = Field(..., description="Location name")
    relevance_reason: str = Field(..., description="Why this location is relevant to the instruction")
    is_target: bool = Field(..., description="Whether this location should be included in target_locations")


class SceneGraphReasoning(BaseModel):
    """Preprocessing 결과 - instruction 분석 및 관련 location 추출"""
    task_understanding: str = Field(
        ...,
        description="Clear understanding of what the user wants to accomplish. Be specific about the action type and target objects."
    )
    location_analysis: List[LocationReasoning] = Field(
        ...,
        description="Analysis of each location's relevance to the instruction. Only include locations that were explicitly mentioned or logically required."
    )
    target_locations: List[str] = Field(
        ...,
        description="Final list of location names that are DIRECTLY relevant to the task. Must match locations where is_target=True in location_analysis."
    )

# --- Task Planning Request Model ---

class RobotPlan(BaseModel):
    chain_of_thought: str = Field(..., description="Break your problem down into a series of intermediate reasoning steps to help you determine your next command")
    reasoning: str = Field(..., description="justify why the next action is important.")
    plan: List[BaseAction] = Field(..., description="A list of actions that the robot should take to achieve the goal instruction.")
