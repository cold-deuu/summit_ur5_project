# SceneGraphMsg - 전체 환경 상태
# Knowledge Base에서 발행, BT/Task Planner에서 구독
#
# 구조:
#   SceneGraphMsg
#   ├── environment (단일)
#   ├── locations[] (환경 내 모든 위치)
#   ├── robots[] (모든 로봇)
#   └── objects[] (모든 객체)
#
# 참조 방향: 상향식 (자식 → 부모)
#   - Location.environment_name → Environment
#   - Robot.current_location_name → Location
#   - Object.current_location_name → Location
#   - Object.held_by_robot_name → Robot

# 환경 정보
Environment environment

# 모든 Location
Location[] locations

# 모든 Robot
Robot[] robots

# 모든 Object (센서에서 검출된 객체)
Object[] objects

# Timestamp (생성 시각)
float64 timestamp

# 통계 (디버깅용)
int32 total_locations
int32 total_robots
int32 total_objects
