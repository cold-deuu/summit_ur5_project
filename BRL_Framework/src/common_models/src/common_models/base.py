"""
Base Models - 기본 데이터 타입들

공통으로 사용되는 기본 좌표/위치 데이터 구조
"""

from pydantic import BaseModel, Field


class Position(BaseModel):
    """3D 위치 좌표 (Object용)"""
    x: float = Field(..., description="X 좌표 (meters)")
    y: float = Field(..., description="Y 좌표 (meters)")
    z: float = Field(default=0.0, description="Z 좌표 (meters)")

    def distance_to(self, other: 'Position') -> float:
        """3D 유클리드 거리 계산"""
        return ((self.x - other.x)**2 + (self.y - other.y)**2 + (self.z - other.z)**2)**0.5

    def distance_2d(self, other: 'Position') -> float:
        """2D 거리 계산 (z축 무시)"""
        return ((self.x - other.x)**2 + (self.y - other.y)**2)**0.5


class Pose2D(BaseModel):
    """2D 포즈 (Robot, Location용)"""
    x: float = Field(..., description="X 좌표 (meters)")
    y: float = Field(..., description="Y 좌표 (meters)")
    theta: float = Field(default=0.0, description="방향 (radians)")

    def to_position(self) -> Position:
        """Position으로 변환 (z=0)"""
        return Position(x=self.x, y=self.y, z=0.0)

    def distance_2d(self, other: 'Pose2D') -> float:
        """2D 거리 계산 (theta 무시)"""
        return ((self.x - other.x)**2 + (self.y - other.y)**2)**0.5