# Environment - 환경 정보
# Scene Graph의 최상위 컨테이너

# 기본 필드
string name              # 환경 이름 (예: "tomato_farm")
string type              # 환경 타입 (예: "greenhouse")
string description       # 설명
string status            # 상태 (예: "idle", "task_planning", "executing")

# 확장 속성
string[] properties      # key:value 형태의 추가 속성
