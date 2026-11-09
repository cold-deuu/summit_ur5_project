# Detection2D - 2D 검출 결과
# YOLO에서 검출한 단일 객체 정보 (Visual Servoing용)
# brl_sensor → motion_planning 전송

std_msgs/Header header

# Bounding box 중심 (픽셀 좌표)
float32 center_x
float32 center_y

# Bounding box 크기 (픽셀)
float32 width
float32 height

# 유효성 플래그 (검출 없으면 false)
bool valid
