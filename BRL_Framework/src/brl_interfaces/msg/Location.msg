# Location - 위치 정보
# Robot과 Object가 위치할 수 있는 공간

# 기본 필드
string name              # 위치 이름 (예: "Stem_1", "Basket_Zone")
string type              # 위치 타입 (예: "stem", "basket_zone")
string description       # 설명
string status            # 상태

# Position
geometry_msgs/Pose2D position  # 2D 위치 (x, y, theta)

# 상향식 참조 (부모 → 자식 방향)
string environment_name        # 소속된 Environment 이름

# 구조적 관계
string[] connected_locations   # 연결된 다른 Location 이름들

# 확장 속성
string[] properties      # key:value 형태의 추가 속성
