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

다른 터미널에서 `rostopic echo -n 1 /real_action_controller/status`를 확인한다. reason이 비어 있고 actual이 실제 상태와 맞아야 한다. admittance는 대기·JointPosture·SE3 모두 기본 ON이며, F/T bias 수집 완료와 신선한 F/T가 필요하다.

확인 후 OFF 서버를 종료하고 사용자가 실행:

```bash
roslaunch summit_ur5_hardware_setup ur5e_actions_field.launch command_enabled:=true
# 다른 터미널: 미리보기 후 명시적으로 실행
rosrun summit_ur5_hardware_setup field_action_joint_check.py
rosrun summit_ur5_hardware_setup field_action_joint_check.py --execute
```

현재 자세에서 pan+1도/5초, 자동복귀없음. 기존 shell은 `rosrun rci_action_manager mm_client.py`다. `home`은 MoveIt 충돌 계획이 아닌 관절 action이다.

`real.cpp`의 기본 admittance는 `hold`다. 기본 실행에서 대기·JointPosture·SE3의 admittance stiffness를 모두0으로 설정하고 밀린 위치를 유지한다. 복원 스프링을 쓰려면 `admittance_mode:=spring`으로 실행한다. BRL action damping은 병진75N·s/m, 회전5N·m·s/rad이며 기존 Cartesian/관절 속도 제한은 유지한다. 현재 action 서버에는 런타임 모드 변경 서비스가 없으므로 모드나 damping 변경 시 재시작이 필요하다.

기본 옵션: `arm_namespace:=/brl_robot/ur5e`, `arm_velocity_limit:=1.57`, `joint_kp:=50`, `se3_kp:=50`, `se3_kd:=1`, `command_horizon:=0.02`, `record:=true`. 기존 scaling publisher가 있으면 `start_speed_scaling_controller:=false`.

원본 `/brl_robot/ur5e/wrench`는 `ur5e_tool0_controller` 좌표이며 `/hqp_field/wrench`는 wrist 좌표다. frame과 원점을 혼용하지 않는다. 팔 TF10Hz 병목은 남아 있으므로 명령 OFF 확인에서 F/T 신선도도 확인한다. 현장 admittance의 최대 변위·가속도 제한은 MoveIt native와 같이 OFF이며, Cartesian/관절 속도 제한은 유지한다.

### BRL MoveIt

위 ROS 환경을 각 터미널에 적용한다. 기존 `ur5e_actions_field.launch`와 `mm_client.py`는 종료하고 다음 세 프로세스를 실행한다. 드라이버/External Control은 유지한다.

```bash
# 터미널 1: BRL 관절명, wrist wrench, 기존 scaled JTC를 사용하는 HQP 토픽 제어기
roslaunch summit_ur5_hardware_setup ur5e_moveit_field.launch
# 터미널 2: 유효한 RViz 계획을 시작점/준비 상태 확인 후 즉시 자동 전송
rosrun summit_ur5_hardware_setup send_rviz_moveit_plan.py \
  _display_topic:=/brl_robot/move_group/display_planned_path _auto_send:=true
# 터미널 3: 현장의 /brl_robot/robot_description 및 SRDF를 재사용하는 planner/RViz
roslaunch summit_ur5_hardware_setup ur5e_moveit_planning_field.launch
```

RViz의 `manipulator` 그룹에서 Start State를 현재 상태로 설정하고 Plan한다. `_auto_send:=true`이면 유효한 계획을 받을 때마다 관절명·현재 시작점·제어 준비 상태를 확인한 뒤 `/rci_action_manager/moveit_plan`으로 즉시 전송한다. 실행 완료를 기다리지 않으며, 추종 중 받은 새 계획도 시작점이 맞으면 기존 계획을 대체한다. 준비되지 않았거나 시작점이 틀린 계획은 보류하지 않고 거절한다. 수동 전송은 `_auto_send:=false`로 실행하고 `send`를 입력한다. RViz의 Execute는 연결하지 않는다.

BRL MoveIt은 기본 `admittance_mode:=hold`, `tracking_admittance:=true`로 대기·추종 모두 admittance ON이다. `ur5e_field_moveit.yaml`에서 damping을 병진50N·s/m, 회전3N·m·s/rad로 설정하며, action 서버의75/5와 별도다. 질량·Cartesian/관절 속도 제한은 기존 값을 유지한다. `admittance_mode:=spring`, `tracking_admittance:=false`로 각각 복원 모드와 추종 시 bypass를 선택할 수 있다. gain 변경은 컨트롤러 재시작 후 적용된다.

추종 중 admittance로 이동한 자세를 유지하면 원래 계획의 nominal 종점 허용오차를 만족하지 못해 `goal tolerance timeout`으로 종료될 수 있다. 중계는 이 과거 결과만으로 새 계획을 차단하지 않으며, fresh 상태·bias·명령 활성 및 현재 시작점 검사는 계속 적용한다. 입력 단절·드라이버 준비 실패·latched fault는 계속 차단하고 이전 계획을 자동 재시도하지 않는다.

플래너는 bringup이 발행한 실제 `/brl_robot/robot_description`을 사용한다. 재기동 후 `robot_description_semantic`이 없으면 `field/ur5e_moveit.srdf`의 기존 현장 설정을 대체 파일로 읽는다. 읽기 전 모든 링크·관절·충돌 제외 참조와 manipulator의6축 체인을 실제 URDF에 대조하며, 맞지 않으면 시작을 중단한다. 기존 semantic 파라미터가 있으면 그 값을 우선하고 robot name을 실제 URDF에 맞춘다. 대체 파일은 `fallback_srdf:=...`로 지정할 수 있다.

BRL의 자세 변화 후 F/T 잔류·처짐을 위한 URDF 질량 조회 및 무접촉 중력 모델 수집/보정 도구는 [FT_GRAVITY.md](FT_GRAVITY.md)를 참고한다.

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
