# 하드웨어 실행 안내

이 폴더는 `summit_ur5_hardware_setup` catkin 패키지다. 전체 설치/인터페이스/제약은 [프로젝트 README](../README.md), 이전 실험은 [HARDWARE_HISTORY.md](../docs/HARDWARE_HISTORY.md)를 참고한다.

## BRL 현장

기존 bringup/External Control을 유지한다. motion controller를 전환하지 않고 기존 scaled JTC를 사용한다. speed scaling publisher가 없으면 읽기 전용 state controller만 시작한다.

```bash
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.50.129:11311
export ROS_IP=192.168.50.80  # 현재 PC의 실제 주소
unset ROS_HOSTNAME

# 모델이 바뀌면: 원격 URDF 조회 + 로컬 snapshot 저장
rosrun summit_ur5_hardware_setup prepare_field_ur5e_model.py
# 첫 확인
roslaunch summit_ur5_hardware_setup ur5e_actions_field.launch command_enabled:=false
```

다른 터미널에서 `rostopic echo -n 1 /real_action_controller/status`를 확인한다. reason이 비어 있고 actual이 실제 상태와 맞아야 한다. admittance OFF에서는 F/T를 사용하지 않지만 수집/진단은 계속한다.

확인 후 OFF 서버를 종료하고 사용자가 실행:

```bash
roslaunch summit_ur5_hardware_setup ur5e_actions_field.launch \
  command_enabled:=true admittance_enabled:=false
# 다른 터미널: 미리보기 후 명시적으로 실행
rosrun summit_ur5_hardware_setup field_action_joint_check.py
rosrun summit_ur5_hardware_setup field_action_joint_check.py --execute
```

현재 자세에서 pan+1도/5초, 자동복귀없음. 기존 shell은 `rosrun rci_action_manager mm_client.py`다. `home`은 MoveIt 충돌 계획이 아닌 관절 action이다.

기본 옵션: `arm_namespace:=/brl_robot/ur5e`, `arm_velocity_limit:=1.57`, `joint_kp:=50`, `se3_kp:=50`, `se3_kd:=1`, `command_horizon:=0.02`, `record:=true`. 기존 scaling publisher가 있으면 `start_speed_scaling_controller:=false`.

원본 `/brl_robot/ur5e/wrench`는 `ur5e_tool0_controller` 좌표이며 `/hqp_field/wrench`는 wrist 좌표다. frame과 원점을 혼용하지 않는다. 팔 TF10Hz 병목이 남아 있어 admittance 기본은 OFF다.

## 연구실 단독 UR5e

연구실 모델은 native 관절명(no `ur5e_`)이고 기본 직접 위치 출력을 사용한다. 실제 IP/calibration은 해당 로봇에 맞춰야 한다.

```bash
roslaunch summit_ur5_hardware_setup ur5e_control_ready.launch \
  robot_ip:=192.168.1.101 reverse_ip:=192.168.1.110
# 다른 명령 노드가 없는 정지 상태에서 사용자 실행
rosrun summit_ur5_hardware_setup switch_ur_position_controller.py
rosrun summit_ur5_hardware_setup switch_ur_position_controller.py --execute
# 다음 둘 중 하나만 실행
roslaunch summit_ur5_hardware_setup ur5e_actions_real.launch
# 또는
roslaunch summit_ur5_hardware_setup ur5e_moveit_real.launch
```

전환 스크립트를 현장 controller manager에 그대로 사용하지 않는다. 포함한 calibration은 특정 연구실 로봇의 snapshot이며 다른 로봇용 자동 보정이 아니다.

MoveIt planner/RViz를 따로 켜려면:

```bash
roslaunch summit_ur5_hardware_setup ur5e_moveit_planning_real.launch
rosrun summit_ur5_hardware_setup send_rviz_moveit_plan.py
```

RViz의 현재 상태로 Plan하고 client의 `send`로 전송한다. 실기 추종 서버 자체는 MoveIt을 띄우지 않는다.

## 로그

```bash
rosrun rci_action_manager plot_action_telemetry.py \
  ~/catkin_ws/artifacts/field_action_logs --show
```

logger가 CSV/metadata를 자동 저장한다. 위 명령은 최신run을 선택하고 `plots/`에 그래프를 저장한다. F/T 다중 자세 진단은 연구실 토픽 기준이며 현장 토픽을 자동 선택하지 않는다:

```bash
roslaunch summit_ur5_hardware_setup ur5e_moveit_real.launch admittance_enabled:=false
rosrun summit_ur5_hardware_setup diagnose_ft.py
```
