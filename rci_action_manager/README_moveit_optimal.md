# UR5e MoveIt 궤적 토픽 → HQP + Admittance

ROS1 Noetic/catkin용 HQP + admittance 제어기다. 기본 모델은 가져온 Summit 모바일 매니퓰레이터 전체이며, MoveIt에서는 모바일을 고정하고 팔 6축만 제어한다. MoveIt은 시뮬에서 plan만 생성하며, 실행은 JointTrajectory 토픽 → 기존 HQP가 담당한다. 액션 서버/controller manager와 연결하지 않는다.

## 이번 모델 연동의 주요 파일

| 파일 | 역할 |
|---|---|
| `summit_ur5e_moveit_config/scripts/import_mobile_model.py` | 원본 보존, mesh namespace 변환, URDF/SRDF/MuJoCo 모델 생성 |
| `summit_ur5e_moveit_config/scripts/check_mobile_mapping.py` | 9축 URDF ↔ MuJoCo FK 검증 |
| `summit_ur5e_moveit_config/config/mobile_*` | 전체 planning 모델, 9축 action 모델, MuJoCo scene/model, 이름 매핑 |
| `rci_action_manager/scripts/simulation/moveit_mujoco_backend.py` | 이름 기반 joint/actuator 매핑, fixed/commanded base, 외력 입력 |
| `rci_action_manager/src/controller/moveit_optimal_controller.cpp` / `moveit_topic_node.cpp` | 전체 모델의 수동 관절 고정과 6축 HQP 재사용 |
| `rci_action_manager/src/controller/controller.cpp` | action의 9축 limit, sensor 회전, idle hold 기준 유지 |
| `rci_action_manager/src/simulation/simulation.cpp` / `src/real/real.cpp` | 모델 선택, 상태 이름 매핑, 6축 팔/3축 base 분리 |
| `rci_holistic_controller/src/task/task_joint_velocity_limit_inequality.cpp` / `task_base_fix_equality.cpp` | 9축 속도 제한 및 base-fix 차원 수정 |
| `rci_action_manager/launch/mobile_actions_{sim,real}.launch` | 기존 action 진입점의 모바일 모델 설정 |
| `rci_action_manager/config/mobile_backend.yaml` / `moveit_optimal_mobile.yaml` | 모바일 매핑·센서 프레임 설정 |
| `rci_action_manager/scripts/test_mobile_actions.py` / `render_mobile_actions.py` | action 자동 검증과 실측 9축 영상 재생 |

기존 주석과 비활성 SCA 코드는 유지했다. 기존 HQP 및 CartesianAdmittance 클래스를 재사용한다.

## 가져온 모바일 모델과 제어 범위

`~/catkin_ws/ur_e_description/summit_ur5e.urdf`의 전체 49개 링크를 planning 모델에 반영했다. 원본 압축 해제 디렉터리는 수정하지 않는다. 중복된 ROS 패키지를 등록하는 대신 참조 mesh를 `summit_ur5e_moveit_config/mobile_assets`로 복사한다.

| 실행 경로 | 모델/제어 범위 | 모바일 |
|---|---|---|
| `moveit_optimal_sim.launch` 기본 `mobile:=true` | 전체 모바일 URDF 표시·충돌 검사, arm 6축 planning/HQP | MuJoCo x/y/yaw 고정 |
| `mobile_actions_sim.launch` | 기존 JointPosture/SE3 action, base 3 + arm 6 = 9축 | SE3에서 x/y/yaw 속도 명령 가능 |
| `moveit_optimal_sim.launch mobile:=false` | 이전 UR5e 단독 모델 | 없음 |
| `moveit_optimal_real.launch` | 새 모바일 URDF에서 arm 6축으로 축소 | 베이스 명령 발행 없음 |
| `mobile_actions_real.launch` | 기존 real.cpp에 9축 모델·odom 연결 | 코드 경로만 준비, publish 호출은 계속 주석 처리 |

두 시뮬 launch는 같은 명령 토픽을 사용하므로 동일 ROS master에 동시에 실행하지 않는다. JointPosture 목표는 기존 인터페이스대로 팔 6축이며 base-fix 태스크가 모바일을 정지시킨다. SE3는 9축 Jacobian으로 모바일과 팔을 함께 사용한다. MoveIt group은 `ur5e_base_link → graps_link`이고, HQP가 제어하는 wrist 프레임은 계속 `ur5e_wrist_3_link`다.

매핑은 배열 인덱스 대신 이름으로 찾는다. 모바일 alias `v_x_joint/v_y_joint/v_t_joint`는 MuJoCo의 `base_x_slide_joint/base_y_slide_joint/base_z_hinge_joint`에 대응한다. 팔은 `ur5e_` 접두사만 다르다. `/mujoco/mapping_info`에서 실제 qpos/actuator 주소와 실행 모드를 확인할 수 있다. `/mujoco/joint_states`는 base 3 + arm 6 + 수동 관절을 포함하고, MoveIt용 `/joint_states`에는 가상 base 관절을 제외한다. `/mujoco/base_state`는 world pose와 child-local twist를 담는다. 팔 장착 위치는 footprint 기준 `[0.225,0,0.522] m`다.

```bash
source ~/catkin_ws/devel/setup.bash
export MOVEIT_CCD_PRELOAD=/usr/lib/x86_64-linux-gnu/libccd.so.2
# MoveIt: 모바일 고정, 팔 추종, admittance ON
roslaunch rci_action_manager moveit_optimal_sim.launch mobile:=true headless:=true rviz:=false
# 별도 터미널
rosrun rci_action_manager test_moveit_optimal.py --output ~/catkin_ws/artifacts/moveit_optimal_validation/mobile_full
rosrun rci_action_manager test_moveit_hold_push.py --output ~/catkin_ws/artifacts/moveit_optimal_validation/mobile_hold_track_push
# 위 launch를 종료한 뒤 기존 action의 모바일 9축 검증
roslaunch rci_action_manager mobile_actions_sim.launch headless:=true rviz:=false
rosrun rci_action_manager test_mobile_actions.py
```

모델 재생성과 기구학 비교:

```bash
python3 src/summit_ur5e_moveit_config/scripts/import_mobile_model.py ur_e_description
python3 src/summit_ur5e_moveit_config/scripts/check_mobile_mapping.py
```

3개 9축 자세, 11개 body의 FK 비교 최대 위치 오차 `4.44e-16 m`, 회전행렬 오차 `7.77e-16`. URDF fixed-axis RPY를 MuJoCo `eulerseq=XYZ`로 맞췄다. 기존 SE2+팔 MuJoCo 모델의 joint/actuator 구조를 재사용하며, 카메라/그리퍼 형상과 질량을 추가했다. 그리퍼와 바퀴의 독립 회전은 고정이고, 바퀴 접촉으로 주행하는 동역학 대신 기존 가상 x/y/yaw 모델을 사용한다.

시뮬레이션의 제한사항:

- 추가한 그리퍼 mesh의 convex hull이 기존 팔 capsule과 겹치므로 그리퍼는 시각·관성 형상만 반영한다. 물체 파지/그리퍼 접촉 검증은 포함하지 않는다.
- 기존 일체형 chassis convex mesh의 바닥 및 인접 upper-arm capsule 중첩은 contact exclusion으로 제거한다. MoveIt은 별도의 원본 URDF collision geometry를 사용한다. SCA는 기존처럼 비활성이다.
- 모바일 테스트 F/T는 `/mujoco/apply_wrench`로 인가한 외력을 sensor frame으로 변환한 값이다. 툴 관성 하중이 admittance에 재입력되지 않도록 load-compensated 외력으로 취급하며, 실제 raw F/T의 관성 보상 성능을 검증한 것은 아니다. 외력은 실제 MuJoCo body에도 인가한다. 중력은 기존 테스트 백엔드처럼 0이다.
- 강한 base velocity servo는 `implicitfast`, 1 ms 물리 주기로 적분한다. 모바일 action SE3만 Kp=12, Kd=0.2를 사용하며, 기존 기본 gain은 보존한다. 속도 기반 제어에서 큰 측정속도 미분 피드백으로 발생하던 진동을 줄였다.

모바일 실기 action launch는 기본 odom `/brl_robot/odom`, base command `/brl_robot/cmd_vel`, F/T `/brl_robot/ft_sensor`를 파라미터로 제공한다. 실제 드라이버 토픽/프레임과 F/T 장착 변환을 확인해야 한다. odom의 child-local twist를 world x/y로 변환하고 base 명령은 반대로 변환한다. 실기 팔은 이름으로 매핑한 6축 JointTrajectory만 사용한다. 실제 명령 송신은 활성화하지 않았고 실기 노드는 실행하지 않았다.

모바일 검증 결과는 `artifacts/moveit_optimal_validation/mobile_full/results.json`, 요청 시나리오는 `mobile_hold_track_push/results.json`, action 결과는 `mobile_actions/results.json`에 있다. 최종 물리/서보 설정의 오차:

| 시험 | RMS / 최대 관절 오차 또는 결과 |
|---|---|
| Joint / Pose / Cartesian 계획 추종 | 0.000616/0.001716, 0.000394/0.001128, 0.000392/0.001155 rad |
| spring 외력 해제 | 복원 오차 0.00306 mm, 정착 1.04 s |
| hold 외력 해제 | 41.61 mm 변위 유지, drift 0.0000714 mm |
| TRACKING 중 외력 | RMS 0.00638 rad, 최종 오차 0.00157 rad |
| 요청한 2초 HOLD → 큰 이동 → 6 N push → hold | RMS 0.00145 rad, 최대 0.00449 rad, 53.50 mm 변위 유지, drift 0.00528 mm |
| 기존 SE3 action (모바일 포함) | base Δx=0.1783 m, Δy=-0.1092 m, Δyaw=0.1360 rad; EE 위치 오차 0.00142 mm |
| MoveIt fixed-base 확인 | 200개 odom 샘플 pose 오차 0 |
| RT core, 새 전체 URDF 축소 모델 | torque limit on/off 각각 500 step malloc 0회 |

영상은 실측 상태를 headless offscreen으로 재생한 것이다. `mobile_hold_track_push/hold_track_push.mp4`와 `mobile_actions/mobile_actions.mp4`에 저장한다. 기존 renderer의 arm-only 영상 지원도 유지한다.

## 실행

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
roslaunch rci_action_manager moveit_optimal_sim.launch
# headless
roslaunch rci_action_manager moveit_optimal_sim.launch headless:=true rviz:=false
# 다른 터미널
rosrun rci_action_manager test_moveit_optimal.py --output ~/catkin_ws/artifacts/moveit_optimal_validation/sim_ur5e
```

MuJoCo Python과 numpy가 필요하다. GUI는 GLFW 디스플레이가 필요하지만 headless 검증은 렌더링하지 않아서 EGL/Xvfb가 필요 없다. 기본 시작 자세는 [0,-1.57,1.57,-1.57,-1.57,0] rad다. 모바일 모델의 planning root는 summit_xl_base_footprint, EE는 graps_link다. mobile:=false에서는 기존 ur5e_base_link/ur5e_wrist_3_link를 사용한다.

이 환경은 /usr/local의 float libccd가 ROS의 double FCL에 로드되어 충돌 검사 중 stack smashing이 발생했다. 동일한 문제가 있으면 아래 환경변수로 MoveIt 프로세스에만 배포판 라이브러리를 지정한다. 시스템 파일은 변경하지 않는다.

```bash
export MOVEIT_CCD_PRELOAD=/usr/lib/x86_64-linux-gnu/libccd.so.2
roslaunch rci_action_manager moveit_optimal_sim.launch headless:=true rviz:=false
```

## 다른 팀에 전달할 인터페이스

- 입력: /rci_action_manager/moveit_plan, trajectory_msgs/JointTrajectory. YAML의 trajectory_topic으로 변경한다.
- joint_names는 아래 6개를 정확히 한 번씩 포함한다. 순서는 자유이며 unknown/duplicate/missing 이름은 거절한다.
- 매 점 positions 6개 필수. velocities/accelerations는 각 필드를 모든 점에서 제공하거나 모두 생략한다. 생략한 미분은 중앙 차분으로 추정한다.
- 2~5000점, 첫 time_from_start≥0, 이후 엄격히 증가. NaN/Inf 및 잘못된 크기는 거절한다.
- 첫 위치는 현재 측정 위치와 관절별 0.05 rad 이내여야 한다. 마지막 속도는 0이어야 하며 마지막 가속도는 정지 연결을 위해 0으로 구성한다.
- stamp=0이면 다음 제어 tick에 시작한다. 미래 stamp까지는 현재 기준을 유지하고, 수신 시각보다 20 ms 이상 과거인 stamp는 거절한다.
- 새 궤적은 기존 실행을 대체한다. 첫 구간은 기존 기준의 q/v/a와 새 궤적을 잇는 quintic blend로 구성한다.
- 잘못된 새 메시지는 현재 궤적을 중단하지 않고 REJECTED 결과만 기록한다.

```text
ur5e_shoulder_pan_joint
ur5e_shoulder_lift_joint
ur5e_elbow_joint
ur5e_wrist_1_joint
ur5e_wrist_2_joint
ur5e_wrist_3_joint
```

보간은 C2 quintic Hermite다. waypoint뿐 아니라 보간 구간의 한계도 검증한다. waypoint 미분이 위치/시간과 일관되지 않으면 보간 가속도가 더 커질 수 있으므로 보내는 쪽에서 spline retiming 또는 시간 스케일링을 수행해야 한다. 시뮬 MoveIt과 테스트 스크립트는 iterative spline 시간 매개화를 사용한다.

상태 토픽은 /moveit_optimal_controller/status (rci_action_manager/MoveitOptimalStatus)다.

| 필드 | 의미 |
|---|---|
| state | HOLDING / TRACKING. 미래 대기는 TRACKING, elapsed<0 |
| trajectory_id | 현재 또는 마지막 실행 ID |
| last_result / result_trajectory_id / reason | NONE, SUCCEEDED, REJECTED, ABORTED와 해당 ID·사유. 실행 중 새 메시지 거절은 active ID를 바꾸지 않음 |
| tracking_error / tracking_rms / tracking_max | compliance 기준 대비 관절 오차와 실행 누적 RMS/최대, rad |
| nominal_error_max | 원래 궤적 대비 최대 관절 오차 |
| actual / desired / command / command_velocity | 모델 관절 순서의 6개 값 |
| admittance_displacement / admittance_velocity | 고정 world 축의 [x,y,z,rx,ry,rz] |
| ft_bias_ready / command_enabled | F/T 보정 준비와 명령 출력 게이트 |
| loop_duration_max_ms / missed_deadlines | 제어 단계 실행 시간과 주기 초과 횟수 |

궤적 시간이 끝나고 nominal 위치 오차<0.015 rad, 측정 속도<0.025 rad/s이면 SUCCEEDED다. 4초 내 도달하지 못하면 ABORTED다. compliance 기준 path 오차가 0.25 rad를 넘으면 현재 명령을 기준으로 감속해 HOLDING으로 전환한다.

## 제어 구조와 재사용

기존 HQPData에 고정 크기 6축 작업 공간을 추가하고 기존 TaskJointPostureEquality를 재사용했다. 기존 SummitUR5Wrapper의 M-D-K 적분은 CartesianAdmittance로 추출했으며 이전 Joint posture/SE3 액션도 같은 클래스를 사용한다. 기존 MoveIt 액션 경로는 이 노드와 무관하다.

- Priority 0 hard: URDF 위치에서 유도한 다음 위치 제한, 속도·가속도 한계, 선택적 모델 예측 토크 한계.
- Priority 1 soft: qdot+slack = vd + dt/2*ad + Kp*(qd_compliant-q_command).
- slack 최소화와 1e-5 속도 정규화. 6축 joint tracking이 모든 DOF를 사용하므로 중복 posture 태스크는 추가하지 않았다.

출력은 관절 속도를 적분한 위치 명령이다. torque_limits의 제약은 tau_model=M(q)*(qdot-qdot_prev)/dt+nle(q,v)에 대한 예측이며 실제 position servo 토크를 보장하지 않는다. 하드웨어 드라이버의 실제 토크·안전 제한이 필요하다.

콜백은 관절명 검증과 C2 구간을 사전 할당된 3개 슬롯에 준비한다. 100 Hz 제어 스레드는 고정 크기 RtEiquadprog<12,6,24> 및 Eigen 행렬을 사용한다. ROS publish/로그/파라미터/서비스/문자열 처리는 별도 callback 스레드에서 한다. 제어 스레드의 의도적인 대기는 주기 sleep뿐이다. 일반 Linux/ROS1이므로 hard real-time 보장은 아니다. allocation_guard 테스트는 전체 control step에서 malloc/calloc/realloc/aligned allocation을 검사한다.

SCA는 기존 합의대로 추가하지 않았다. MoveIt은 nominal 경로만 충돌 검사하며 외력 변위의 온라인 충돌 회피를 보장하지 않는다. SRDF 인접 링크 면제와 URDF 관성·툴·페이로드는 배포 전에 검토한다.

## Admittance

같은 M*xddot+D*xdot+K*x=F 적분기를 사용한다. TRACKING은 양의 tracking_stiffness로 궤적에 복원한다. tracking_admittance=false이면 새 외력은 막고 기존 offset은 spring으로 부드럽게 줄인다.

HOLDING spring은 종료 위치를 평형점으로 복원한다. hold는 K=0 mass-damper로 속도만 감쇠시켜 변위를 유지한다. 임계값 통과마다 평형점을 갱신하는 방식의 채터링을 피하기 위해 후자를 택했다. 모드 전환은 x/xdot를 보존하고 K만 변경한다. 최종 명령의 가속도 제한은 유지된다.

고정 world 축의 변위를 nominal 손목 Jacobian의 damped inverse로 관절 기준에 반영한다. 작은 변위용 differential compliance이며 큰 회전의 비선형 IK는 아니다. 변위/속도 saturation은 축별이며 anti-windup을 적용한다.

```bash
rosservice call /moveit_optimal_controller/set_admittance_mode "mode: 'hold'"
rosservice call /moveit_optimal_controller/set_admittance_mode "mode: 'spring'"
rosservice call /moveit_optimal_controller/set_tracking_admittance "data: false"
rosservice call /moveit_optimal_controller/set_tracking_admittance "data: true"
rosservice call /moveit_optimal_controller/zero_ft "{}"
```

F/T 입력은 sensor 축 raw wrench다. 기본 100개 정지·무부하 표본으로 bias를 구한 뒤 부호, 저역통과, deadband, sensor→wrist 회전·moment arm 보정, world 회전을 적용한다. zero_ft는 정지 HOLDING에서만 보정을 시작한다. 센서/상태가 0.25초 이상 끊기면 추종을 중단한다.

moveit_optimal.yaml에 모든 M/D/K, deadband, cutoff, 변위·속도 saturation, bias, sign, sensor transform, tolerance와 joint 한계가 있다. 기본 M=[3,3,3,.3,.3,.3], D=[35,35,35,3.5,3.5,3.5], K=[100,100,100,10,10,10]은 기존 free admittance보다 무겁고 댐핑을 높여 작은 외력에서 복원하면서 속도 포화를 줄이도록 선택했다. 실기 툴에 맞춘 재튜닝이 필요하다.

## 실기와 mock

저장소에는 팀 하드웨어 드라이버 launch와 실제 F/T 토픽 명세가 없다. 실기 기본 토픽은 기존 코드 및 placeholder이며 배포팀 확인이 필요하다. MoveIt은 실기 launch에 포함하지 않는다.

```bash
# command 송신 off가 기본
roslaunch rci_action_manager moveit_optimal_real.launch
roslaunch rci_action_manager moveit_optimal_real.launch hardware_launch:=/absolute/path/to/team_hardware.launch
# 실제 장비와 분리된 real wrapper 검증
roslaunch rci_action_manager moveit_optimal_mock_real.launch
rosrun rci_action_manager test_moveit_optimal.py --replay ~/catkin_ws/artifacts/moveit_optimal_validation/sim_ur5e/trajectory.json --output ~/catkin_ws/artifacts/moveit_optimal_validation/mock_real
```

실기 입력은 /brl_robot/ur5e/joint_states, /brl_robot/ur5e/wrench (WrenchStamped)이며, 출력은 /brl_robot/ur5e/ur5e_scaled_pos_joint_traj_controller/command의 한 점 JointTrajectory다. 실제 하드웨어에 명령을 보낸 검증은 하지 않았다. mock은 /mock_hw/*만 사용한다.

실기 첫 실행 순서:

1. 다른 action/servo와 제어권이 겹치지 않게 하고, 드라이버의 100 Hz position-stream 지원과 watchdog을 확인한다.
2. 실제 토픽, joint 이름, F/T frame/sign/offset, URDF/툴 관성, 실제 토크 제한과 e-stop을 확인한다.
3. command_enabled=false로 상태를 확인한다. 정지·무부하에서 zero_ft 완료를 기다린다.
4. 실기 YAML의 .15~.2 rad/s 및 .3~.4 rad/s² 한계와 낮은 계획 속도 스케일을 사용한다.
5. tracking_admittance=false로 작은 궤적 추종부터 확인한다. HOLDING 외력 동작은 무부하로 확인한다.
6. 장비 준비를 책임자가 확인한 다음 별도 실행에서 command_enabled:=true를 명시한다.
7. 작은 힘으로 부호를 확인한 후 admittance를 켜고 spring/hold/센서 단절을 순서대로 검증한다.

## 파일과 검사

| 파일 | 역할 |
|---|---|
| controller/moveit_optimal_controller.hpp/.cpp | C2 보간, 상태 머신, HQP+admittance 공통 코어 |
| controller/moveit_topic_node.hpp/.cpp | 검증·매핑, bias, 서비스, 상태 발행, 스레드 분리 |
| real/real_moveit.cpp | 실기 state/wrench/JointTrajectory 입출력 |
| simulation/sim_moveit.cpp | 시뮬 state/wrench/Float64MultiArray 입출력 |
| msg/MoveitOptimalStatus.msg, srv/SetAdmittanceMode.srv | 외부 상태·모드 인터페이스 |
| config/moveit_optimal*.yaml, launch/moveit_optimal*.launch | 공통·실기 설정과 실행 |
| scripts/simulation/moveit_mujoco_backend.py | 기존 MuJoCo 모델 재사용, 외력, mock backend |
| scripts/test_moveit_optimal.py | plan→토픽→상태 검증, 저장 메시지 재생 |
| test/moveit_core_test.cpp, allocation_guard.c | C2, 제한, 무할당 검사 |
| rci_holistic_controller/.../admittance/cartesian_admittance.hpp | 기존 admittance의 공통 구현 |
| rci_holistic_controller/.../data/hqp_data.* | 기존 HQP의 사전 할당 6축 경로 |
| rci_holistic_controller/.../task/task_joint_posture_equality.hpp | 기존 joint task의 q/v/a 기준 확장 |
| summit_ur5e_moveit_config | UR5e-only URDF/SRDF, KDL·OMPL, planning-only launch |

결과는 output/results.json, 원본 데이터는 status_history.npz, mock 재생 메시지는 trajectory.json이다. 빌드 후 `cd build && ctest -R 'hqp_data_test|moveit_core_test' --output-on-failure`로 회귀/코어 검사를 실행한다.

## 큰 동작 및 영상

외력은 주지 않고 tracking admittance는 **항상 ON**으로 유지하는 별도 시나리오:

```bash
rosrun rci_action_manager test_moveit_long_tracking.py --output ~/catkin_ws/artifacts/moveit_optimal_validation/long_tracking_admittance_on
```

어깨 좌우 약 ±1 rad, 팔 뻗기, 자세 변경, home 복귀를 계획한다. MoveIt 속도 스케일 .75, 가속도 스케일 .6을 사용하며 제어기의 기존 한계는 유지한다. 영상은 제어 중 저장한 실제 관절 상태를 시험 종료 후 offscreen replay한 것이므로 렌더링이 제어 루프를 지연시키지 않는다.

선택적 영상 의존성은 PyOpenGL>=3.1.7, imageio-ffmpeg, opencv-python이다. 기본 테스트의 tracking/admittance 영상을 만들거나 큰 동작 전체를 렌더링한다:

```bash
MUJOCO_GL=egl rosrun rci_action_manager render_moveit_validation.py ~/catkin_ws/artifacts/moveit_optimal_validation/sim_ur5e
MUJOCO_GL=egl rosrun rci_action_manager render_moveit_validation.py ~/catkin_ws/artifacts/moveit_optimal_validation/long_tracking_admittance_on --tracking-only
```

이번 환경에서는 영상 의존성을 /tmp/hqp_record_deps에 격리 설치했다. 재렌더링할 때 필요한 경우 PYTHONPATH 앞에 해당 경로를 추가한다.

## 이번 실행 결과

UR5e 단독, 실제 ROS 토픽 추종, MuJoCo 직접 실행 결과다. 실제 장비는 구동하지 않았다.

| 검사 | 결과 |
|---|---|
| joint_plan | SUCCEEDED, RMS 0.008481 rad, 최대 0.022504 rad |
| pose_plan | SUCCEEDED, RMS 0.003838 rad, 최대 0.010186 rad |
| cartesian_plan | SUCCEEDED, RMS 0.005188 rad, 최대 0.013346 rad |
| spring | 복원 오차 0.0034 mm, 정착 1.053 s |
| hold | 추가 드리프트 0.0086 mm |
| 추종 중 외력 | 복귀 후 goal 최대 오차 0.005358 rad |
| 100 Hz 제어 단계 | 최대 0.265 ms, deadline 초과 0회 |
| 관절명 순서 섞임 / 궤적 대체 / 미래 stamp / 잘못된 메시지 | 모두 통과 |
| 고정 크기 HQP 할당 검사 | torque limit ON/OFF 각각 500주기, 0회 |
| 실기 래퍼 mock 재생 | SUCCEEDED, RMS 0.008036 rad, 최대 0.021245 rad |

큰 동작 영상은 admittance ON / 외력 0 조건이다. 네 경로 모두 SUCCEEDED이며, 실제 EE 누적 경로 약 3.353 m, 가장 긴 한 경로 약 1.177 m, 어깨 최대 변화 약 2 rad였다. RMS 0.0206~0.0251 rad, 최대 오차 약 0.0391 rad, 최종 goal 최대 오차는 0.00381 rad 이하였다.

남은 배포 결정: 실제 하드웨어 드라이버 launch, state/command/F/T 토픽, F/T 부호·좌표계·moment arm, 툴/페이로드와 실제 토크 제한. 현재 검증 범위는 UR5e 단독이며 모바일 전신 계획·실기 운동은 검증하지 않았다.


## 2초 HOLD → 계획/추종 → 외력 후 밀린 위치 유지

```bash
rosrun rci_action_manager test_moveit_hold_push.py --output ~/catkin_ws/artifacts/moveit_optimal_validation/hold_track_push
MUJOCO_GL=egl rosrun rci_action_manager render_moveit_validation.py ~/catkin_ws/artifacts/moveit_optimal_validation/hold_track_push
```

F/T bias와 MoveIt 연결이 준비된 초기 자세에서 영상 기록을 시작한다. 2초 HOLD, 이후 MoveIt 계획을 토픽으로 전달해 TRACKING, 완료 후 1초 HOLD, world +X 방향 6 N을 0.8초 인가, 외력 제거 후 감속 및 밀린 위치 유지 순서다. 전 구간 admittance ON이며 HOLDING은 K=0이다.

실행 결과: 초기 HOLD 최대 관절 드리프트 0 rad, 추종 SUCCEEDED, RMS 0.019431 rad/최대 0.036803 rad. 외력 제거 후 EE가 원래 종료 위치에서 55.886 mm 이동한 상태를 유지했고, 감속 후 추가 드리프트는 0.0395 mm였다. 영상은 hold_track_push.mp4, 단계 시각은 scenario_events.json, 외력/모드 설정은 scenario.json에 저장된다.
