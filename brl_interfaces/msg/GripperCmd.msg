# GripperCmd - 그리퍼 명령
# Gripper 제어용 메시지

# Emergency 제어
bool emergency_release       # 긴급 해제 플래그
uint32 emergency_release_dir # 긴급 해제 방향
bool stop                    # 정지 플래그

# 동작 파라미터
float32 position             # 목표 위치 (mm)
float32 speed                # 속도 (mm/s)
float32 force                # 힘 (N)
