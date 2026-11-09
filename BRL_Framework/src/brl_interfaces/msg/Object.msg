# Object - 객체 정보
# 토마토 등 환경 내 객체

# 기본 필드
string name              # 객체 이름 (예: "tomato_01")
string type              # 객체 타입 (예: "tomato")
string description       # 설명
string status            # 상태 (예: "detected", "picked")

# Position
geometry_msgs/Point position   # 3D 위치 (world frame)

# 상향식 참조 (상호 배타적)
string current_location_name   # 위치한 Location 이름 (또는 빈 문자열)
string held_by_robot_name      # 들고 있는 Robot 이름 (또는 빈 문자열)

# 확장 속성 (토마토의 경우)
# - ripeness:0.8 (익은 정도, 0-1)
# - quality:0.95 (품질 신뢰도)
# - yolo_class:0 (0=ripe, 1=unripe, 2=rotten)
# - bounding_box:[x1,y1,x2,y2]

string[] properties
string index    # '1/3'