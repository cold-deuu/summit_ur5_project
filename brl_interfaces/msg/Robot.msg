# Robot - 로봇 정보
# 위치 추적 및 capability 관리

# 기본 필드
string name              # 로봇 이름 (예: "harvester_01")
string type              # 로봇 타입 (예: "harvester", "mobile")
string description       # 설명
string status            # 상태 (예: "active", "fine_tuning", "harvesting")

# Position
geometry_msgs/Pose2D position  # 2D 위치 (AMCL 기반)

# 상향식 참조 (부모 → 자식 방향)
string current_location_name   # 현재 위치한 Location 이름

# Capabilities (타입별로 YAML에서 로드)
string[] capabilities    # 수행 가능한 action 목록 (예: ["NavigateTo", "Harvest"])

# 보유 객체
string[] held_objects    # 로봇이 들고 있는 Object 이름들

# 확장 속성
string[] properties      # key:value 형태의 추가 속성
