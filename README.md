# Summit UR5e HQP Controller — BRL

Summit 모바일 매니퓰레이터/UR5e의 ROS 1 HQP 제어 코드다. **`brl` 브랜치에는 연구실 HQP·admittance 튜닝과 BRL 현장 action 연결 설정이 포함된다.**

빌드 기준은 **Ubuntu 20.04 / ROS Noetic / GCC 9 / C++17**이다. 현장 Ubuntu 18.04 PC의 기존 UR driver/bringup은 재사용한다. 이 코드를 그 PC에서 바로 빌드할 수 있다고 보장하지 않는다.

## 설치와 catkin_make

ROS Noetic과 ROS apt 저장소가 준비된 PC에서, 저장소를 catkin workspace의 **`src` 아래**에 받는다.

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone --branch brl https://github.com/cold-deuu/summit_ur5_project.git

# 최초 1회. sudo/인터넷 필요. ROS 노드나 로봇 명령은 실행하지 않는다.
bash ~/catkin_ws/src/summit_ur5_project/scripts/install_dependencies.sh

source /opt/ros/noetic/setup.bash
cd ~/catkin_ws
catkin_make -j2
source devel/setup.bash
```

의존성이 설치된 환경에서는 바로 `catkin_make`하면 된다. Pinocchio/Eigen 컴파일의 메모리 사용량 때문에 처음에는 `-j2`를 권장한다. 개인 `/home/chan/local` 경로는 필요하지 않다.

주요 의존성은 Pinocchio, eiquadprog, **toppra v0.6.4 C++**(설치 스크립트가 소스 빌드), **ROS Noetic FCL**, libccd, ROS control/controllers, UR driver/calibration, MoveIt, NumPy/SciPy/Matplotlib이다. 오래된 시스템 `libfcl-dev`만으로 `ros-noetic-fcl`을 대체하지 않는다. 기존 MuJoCo 2.1 SDK, 모델 mesh와 C++ SCA 런타임 weight는 포함되어 있다. C++ 빌드에 PyTorch/CUDA는 필요하지 않다.

Python MuJoCo 시뮬레이션·영상 렌더링까지 사용할 경우:

```bash
bash ~/catkin_ws/src/summit_ur5_project/scripts/install_dependencies.sh --with-sim
```

검증에 사용한 Python MuJoCo는 **3.2.3**이다. 추가 Python 버전은 [requirements-simulation.txt](scripts/requirements-simulation.txt)에 고정했다.

## 패키지 구성

| 경로 / 패키지 | 역할 |
|---|---|
| `rci_holistic_controller` | HQP solver·제약·태스크·로봇 모델·공통 CartesianAdmittance |
| `rci_action_manager` | JointPosture/SE3 action, 토픽 경로 추종, telemetry |
| `hardware_setup` / `summit_ur5_hardware_setup` | 현장/연구실 launch, 작은 동작 시험, 진단 도구 |
| `summit_ur5e_moveit_config` | 팔/모바일 모델, planning-only MoveIt |
| `robot_description` | Summit·UR·gripper URDF, MuJoCo 장면, mesh |
| `sca_pkg` | 기존 SCA 코드와 weight. **현재 제약은 비활성** |
| `mujoco_ros` | 기존 C++ simulator, 메시지, MuJoCo 2.1 SDK |

## BRL 현장: real.cpp action 서버

현장에서는 기존 bringup과 External Control을 재사용한다. **이 launch는 motion controller를 새로 시작/전환하지 않고, 이미 running인 scaled JTC에 한 점씩 보낸다.**

확인 당시 네트워크:

| 역할 | 주소 |
|---|---|
| ROS master | `192.168.50.129:11311` |
| UR driver PC (`sxls0-220429aa`) ROS망 | `192.168.50.165` |
| 같은 driver PC의 로봇망 | `192.168.0.200` |
| UR 로봇 | `192.168.0.210` |
| HQP 개발 PC 예시 | `192.168.50.80` |

펜던트 External Control Host는 기존 **`192.168.0.200:50002`**를 유지한다. ROS master/개발 PC 주소로 바꾸는 항목이 아니다. 현장 driver가 있는 상태에서 개발 PC의 bringup을 중복 실행하지 않는다.

각 터미널의 환경:

```bash
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.50.129:11311
export ROS_IP=192.168.50.80  # 이 PC의 실제 IP로 변경
unset ROS_HOSTNAME
```

모델이 달라졌다면 현장 URDF에서 arm6 snapshot을 다시 만든다. 이 스크립트는 원격 파라미터를 읽고 로컬 파일만 저장한다.

```bash
rosrun summit_ur5_hardware_setup prepare_field_ur5e_model.py
```

첫 연결은 **명령 OFF**로 확인:

```bash
roslaunch summit_ur5_hardware_setup ur5e_actions_field.launch command_enabled:=false
# 다른 터미널
rostopic echo -n 1 /real_action_controller/status
```

`HOLDING`, 빈 `reason`, 올바른 actual 관절값을 확인한다. 기본 admittance는 ON이며 대기·JointPosture·SE3 모두에 적용된다. **Bias 수집 완료와 신선한 F/T가 제어 준비 조건이다.** 진단을 위해 `admittance_enabled:=false`로 실행하면 F/T 수집은 계속하지만 제어 준비 조건에서는 제외한다.

준비가 끝나면 OFF 서버를 종료하고 사용자가 실행:

```bash
roslaunch summit_ur5_hardware_setup ur5e_actions_field.launch command_enabled:=true

# 다른 터미널: 현재 자세에서 pan +1도 / 5초, 자동 복귀 없음
rosrun summit_ur5_hardware_setup field_action_joint_check.py
rosrun summit_ur5_hardware_setup field_action_joint_check.py --execute
```

작은 +1°/5초 action은 현장에서 **SUCCEEDED**를 확인했다. 기존 shell client:

```bash
rosrun rci_action_manager mm_client.py
```

`home` 순서는 pan/lift/elbow/wrist1/wrist2/wrist3이고, 대략 **[180°, −90°, −90°, −90°, +90°, 0°]**, 5초다. 이 action은 충돌 검사된 MoveIt 계획이 아니다. 현재 자세와 주변 환경을 먼저 확인한다.

### 현장 인터페이스와 기본값

| 용도 | 이름 / 타입 |
|---|---|
| 팔 상태 | `/brl_robot/ur5e/joint_states` — `sensor_msgs/JointState` |
| 원본 F/T | `/brl_robot/ur5e/wrench` — `WrenchStamped`, `ur5e_tool0_controller` |
| 변환 F/T | `/hqp_field/wrench` — wrist 원점/축의 `WrenchStamped` |
| 출력 | `/brl_robot/ur5e/ur5e_scaled_pos_joint_traj_controller/command` — `JointTrajectory` |
| JointPosture / SE3 action | `/rci_action_manager/jointControl`, `/rci_action_manager/se3Control` |
| 상태 / telemetry | `/real_action_controller/status`, `/real_action_controller/telemetry` |

- 출력은 **100 Hz**, stamp 0, **positions만 있는 점 1개**, 기본 horizon **0.02초**다.
- 관절은 `ur5e_` 접두사 6축이며 이름으로 매핑한다. startup에서 기존 scaled JTC의 종류/running/관절명을 조회한다.
- 팔만 제어하며 모바일은 고정한다. SE3 제어점은 `ur5e_wrist_3_joint`다. MoveIt의 `graps_link`와 구분한다.
- 기본 Kp50, arm speed cap1.57rad/s, SE3 Kp50/Kd1, admittance ON이다. 초기 상한은 `arm_velocity_limit:=0.3`으로 낮출 수 있다.
- speed scaling publisher가 없으면 **읽기 전용** SpeedScalingStateController를 시작한다. 이미 있다면 `start_speed_scaling_controller:=false`로 둔다.
- 다른 MoveIt Execute/action과 동일 팔을 동시에 제어하지 않는다.

## 연구실 UR5e / MoveIt 토픽 추종

연구실은 단독 UR5e와 native 관절명(no `ur5e_` prefix), 기본 **JointGroupPositionController 직접 위치 출력**을 사용한다. 현장 launch와 혼동하지 않는다. [하드웨어 실행 안내](hardware_setup/README.md)에 bringup/전환 순서가 있다.

```bash
# 둘 중 하나만 실행
roslaunch summit_ur5_hardware_setup ur5e_actions_real.launch
roslaunch summit_ur5_hardware_setup ur5e_moveit_real.launch
```

MoveIt 토픽 노드에는 `move_group`이 필요 없다. 다른 팀이 아래 토픽으로 계획을 보내면 검증 후 실행한다. 위 launch는 연구실 모델/namespace용이다. **BRL 현장에서는 `ur5e_moveit_field.launch`와 `ur5e_moveit_planning_field.launch`를 사용한다.** 현장 모델·토픽·기존 scaled JTC에 맞춘 연결과 RViz 전송 명령은 [하드웨어 안내의 BRL MoveIt](hardware_setup/README.md#brl-moveit)에 있다. 현장 실기 추종 검증은 별도다.

| 항목 | 명세 |
|---|---|
| 입력 | `/rci_action_manager/moveit_plan` — **`trajectory_msgs/JointTrajectory`** |
| 연구실 joint_names | `shoulder_pan_joint`, `shoulder_lift_joint`, `elbow_joint`, `wrist_1_joint`, `wrist_2_joint`, `wrist_3_joint` |
| 시작점 허용오차 | 실제 관절과 축별 **0.05 rad** |
| stamp | 0이면 즉시, 미래이면 그 시각부터 시작 |
| 결과 | `~status`: `SUCCEEDED/REJECTED/ABORTED`와 reason |

관절 순서는 이름으로 매핑한다. 빈 궤적·중복/모르는 관절명·비유한 값·단조 증가하지 않는 시간을 거절한다. 추종 중 새 궤적은 기존 궤적을 대체한다.

현재 native 프로필은 **위치 기반 경로 추종**이다. MoveIt의 도착 시간표를 강제하지 않는다. 연속 quintic 경로와 전체 끝점에서만 정지하는 S-curve 진행 상태를 사용하고, 실제 추종 지연이 커지면 smooth clock이 느려진다. 실제 종점/정지 조건으로 완료한다. HQP solver는 그대로다.

기본 Kp50 / 관절 속도한계1.57rad/s / 명시적 QP 가속도 제한OFF. 진행 설정은 path_speed1.0, path_acceleration2, path_jerk10, path_clock_bandwidth10이다. 이들은 **목표 진행 상태의 제한**이며 곡률/clock을 합성한 모든 실제 관절에 대한 hard jerk bound는 아니다. 현장 action 서버에는 이 topic 전용 governor를 적용하지 않는다.

계획 생성/RViz를 따로 사용하는 경우:

```bash
roslaunch summit_ur5_hardware_setup ur5e_moveit_planning_real.launch
# 다른 터미널. RViz에서 현재 상태로 Plan 후, client에서 send 입력 시에만 전송
rosrun summit_ur5_hardware_setup send_rviz_moveit_plan.py
```

자세한 입력 검증·상태 머신은 [README_moveit_optimal.md](rci_action_manager/README_moveit_optimal.md)를 참고한다. 해당 문서의 이전 시간 기반 프로필과 현재 native 기하 모드를 구분한다.

## Admittance 설정과 남은 문제

공통 CartesianAdmittance를 재사용한다. **연구실 MoveIt은 TRACKING OFF/HOLD spring ON, BRL MoveIt은 hold 모드·TRACKING ON, 현장 action은 대기·JointPosture·SE3 모두 기본 ON**이다.

| 연구실 native HOLD 설정 | 현재 값 |
|---|---|
| 병진 M / D / K | 5 / 150 / 150 |
| 회전 M / D / K | 0.5 / 10 / 2 |
| deadband / LPF | 축별 2N·0.15Nm / 4Hz |
| 병진 속도 | 축별0.06m/s, norm0.08m/s |
| 회전 속도 | 축별0.08rad/s, norm0.10rad/s |
| admittance 관절 속도 | 0.15rad/s |
| native 변위·admittance 가속도 clip | OFF |

BRL action/MoveIt의 `ur5e_field_action.yaml`은 `displacement_limits_enabled: false`, `acceleration_limits_enabled: false`를 사용한다. Cartesian 최대 변위·가속도와 admittance 관절 가속도 제한을 해제하며, 위 표의 Cartesian/관절 속도 제한과 기존 HOLD reference-lag 제한은 유지한다. BRL action damping은 병진75/회전5이며, MoveIt은 마지막에 로드하는 `ur5e_field_moveit.yaml`에서50/3으로 덮어쓴다. 현장 action과 BRL MoveIt의 기본 모드는 hold(K=0)이며 `admittance_mode:=spring`으로 복원 스프링을 선택할 수 있다. 위 연구실 native 표와 BRL 설정 파일은 별도다.

연구실에서는 무접촉 자세에도 bias 보정 후 수 N의 잔류 힘이 관측됐다. startup 상수 bias는 자세 의존 오차를 없애지 못한다. 접촉 힘을 지울 수 있는 자동 HOLD 재영점은 구현하지 않았다. spring도 잔류 힘을 제거하지 않으며 `F/K`만큼 편위될 수 있다.

현장에서는 원본 wrench/controller TCP TF가 약500Hz인데 팔 link TF가 약10Hz여서 exact-stamp 변환이 느리고 불규칙했다. 어댑터는 최대0.2초 이내 센서 시각의 TF를 기다리고, 임의 identity/latest TF로 대체하지 않는다. **현장 admittance ON 전 TF 병목과 무접촉 wrench를 검증해야 한다.**

## 로그와 진단

기본 launch가 CSV/metadata를 자동 저장한다. 별도 logger 실행은 필요 없다.

- 현장 action: `~/catkin_ws/artifacts/field_action_logs/시간/`
- 연구실 action: `~/catkin_ws/artifacts/real_action_logs/시간/`
- 연구실 MoveIt: `~/catkin_ws/artifacts/moveit_real_logs/시간/`

```bash
rosrun rci_action_manager plot_action_telemetry.py \
  ~/catkin_ws/artifacts/field_action_logs --show
```

최신run의 EE3D 경로, translation/rotation XYZ error, wrench/admittance 그래프를 `plots/`에 저장한다. topic controller의 `command_timing.csv`는 **노드 발행 간격·샘플 나이**이며 드라이버 수신 시각은 아니다. action 서버에는 이 timing 채널이 없다.

F/T 다중 자세 진단은 연구실 토픽 기준이다:

```bash
roslaunch summit_ur5_hardware_setup ur5e_moveit_real.launch admittance_enabled:=false
rosrun summit_ur5_hardware_setup diagnose_ft.py
```

자세 이름으로 정지 구간을 기록하며 명령/영점 서비스는 보내지 않는다. 측정 사이에 bias·Payload를 바꾸지 않는다.

## 통신·시간과 검증 범위

ROS master 연결은 PC 시계를 자동 동기화하지 않는다. 현장에서 미래 stamp 약150ms 및 Wi-Fi 수신 공백 최대324ms가 관측됐다. 이후 수 ms stamp age도 확인했지만 장기 안정성 검증은 아니다. **고주기 제어는 유선 또는 driver와 같은 PC에서**, 무선은 전체 경로/action 전달과 모니터링에 사용하는 구성이 적합하다. 평균500Hz만 보지 말고 최대 간격과 stamp age도 확인한다. 시간 강제 변경은 제어 정지 상태에서 한다.

```bash
cd ~/catkin_ws/build
ctest -R 'hqp_data_test|moveit_core_test' --output-on-failure
```

검증: clean workspace 전체빌드, 위 단위 테스트, 빠른 입력경로/지연응답/종점정지, spline C2, S-curve/clock, RT할당0 및 현장 작은 action 성공. **모든 home/SE3/작업공간·현장 admittance가 검증됐다는 뜻은 아니다.**

SCA 제약은 현재 비활성이다. 연구실 calibration snapshot을 다른 로봇에 그대로 쓰지 않는다. 원격 JTC에 다른 client가 동시에 명령하는 것을 네트워크 전체에서 자동 차단하지는 않는다.

이전 실험 이력은 [HARDWARE_HISTORY.md](docs/HARDWARE_HISTORY.md)에 보존했다. 현재 기본값은 launch/YAML과 이 README를 우선한다. 기존 패키지/SDK/모델 라이선스는 유지한다([LICENSE](LICENSE), MuJoCo `THIRD_PARTY_NOTICES`, 모델별 LICENSE).
