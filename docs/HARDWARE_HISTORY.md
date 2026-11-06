# 실기 첫 연결: ROS1 Noetic / UR5e + Summit

> 최신 기본값: `ur5e_actions_real.launch`는 **command_enabled=true**로 시작한다. OFF가 필요하면 `command_enabled:=false`를 명시한다. 입력·bias·driver 준비 검사와 fault latch는 유지한다.

현재 PC는 Ubuntu 20.04 / ROS1 Noetic이다. ROS2 Foxy도 설치되어 있으므로 새 터미널에서는 Noetic만 source한다. 드라이버·URCap 다운로드만 수행했고 실제 로봇 접속, 전원/브레이크 조작, 명령 발행은 하지 않았다.

## 1. 준비한 다운로드와 설치

`downloads/debs/`에 UR 공식 Noetic driver 2.4.1, ur_calibration 및 현재 PC에서 누락된 의존 패키지까지 18개(약 22.3 MB)를 다운로드했다. `downloads/externalcontrol-1.0.6.urcap`은 로봇 티치펜던트에 설치할 파일이다. 파일 목록·SHA256은 `downloads/manifest.json`에 있다.

이 PC는 sudo 비밀번호가 필요해서 시스템 설치는 하지 못했다. 사용자가 터미널에서 아래만 실행한다. 비밀번호는 채팅에 보내지 않는다.

```bash
cd ~/catkin_ws
sudo apt install ./hardware_setup/downloads/debs/*.deb
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rospack find ur_robot_driver
rospack find ur_calibration
```

온라인 설치가 필요하면 `sudo apt update` 후 `sudo apt install ros-noetic-ur-robot-driver ros-noetic-ur-calibration`을 사용한다. ROS1은 2025년 5월 지원 종료되었지만 이번 저장소와 다운로드는 Noetic 기준으로 맞췄다. 지금 단계에서 ROS2로 바꾸지 않는다.

## 2. 누가 하드웨어 드라이버를 실행하는지 먼저 확인

UR5e 컨트롤 박스는 Ethernet으로 UR 드라이버 PC에 연결한다. Summit 본체 PC는 모바일 모터와 odom의 별도 드라이버를 담당할 수 있다. 로봇에 이미 `/brl_robot/...` bringup이 있으면 그 구성을 먼저 확인한다. 기존 드라이버와 새 UR driver를 같은 팔에 동시에 연결하지 않는다.

`ur_e_description`과 현재 `summit_xl_description`은 모델 파일이며 실제 바퀴 모터 bringup이 아니다. 공개 `summit_xl_common`도 모델·공통 패키지 중심이므로 이것만 clone하면 실기 모바일이 구동된다고 가정하지 않는다. 실제 모바일 드라이버/기동 방법은 본체 PC의 기존 설치, 제조사 장비별 설정을 확인한 뒤 사용한다. 기존 패키지 중복을 피하기 위해 추가로 clone하지 않았다.

처음 목표는 모바일 정지 상태에서 UR5e 관절 상태 읽기다. 그다음 팔 소규모 추종, 마지막에 admittance와 모바일 9축을 진행한다.

## 3. Ethernet / IP 설정

이 PC 점검 당시 Wi-Fi IP는 `172.20.10.3`, 활성 유선 IPv4는 없었다. 네트워크 설정은 변경하지 않았다. UR5e 실제 IP와 로봇 본체 PC IP는 아직 확인되지 않았다. 아래 주소는 예시이며 그대로 실제 주소라고 생각하면 안 된다.

| 장치 | 예시 |
|---|---|
| 드라이버를 실행할 PC 유선 IP | 192.168.0.20 / 255.255.255.0 |
| UR5e 컨트롤 박스 | 192.168.0.10 / 255.255.255.0 |
| Summit 본체 PC | 장비의 기존 IP 유지·확인 |

PC와 UR 사이 유선 연결/같은 스위치를 사용한다. Ubuntu 설정 → 네트워크 → 유선 → IPv4 → 수동에서 사용하지 않는 IP를 설정한다. 로봇 티치펜던트의 Network 설정에서 실제 robot IP/netmask를 확인한다. 직접 연결망은 gateway가 필요하지 않을 수 있다. 기존 장비 네트워크 주소를 확인하기 전 바꾸지 않는다.

```bash
ip -br -4 addr
# 아래는 실제 UR IP로 바꾼 뒤 사용자 실행
ping -c 3 192.168.0.10
ip route get 192.168.0.10
```

UR→PC 역방향 통신도 필요하다. External Control의 Host IP와 reverse_ip는 로봇에서 도달 가능한 PC 유선 IP다. 로봇 IP를 Host IP에 쓰지 않는다. 기본 포트 50001~50004는 PC측, 29999/30001/30002/30003/30004 등은 UR측 통신 용도로 쓰인다. 방화벽 문제가 있으면 해당 망/필요 포트 규칙을 확인하고 방화벽 전체를 무작정 끄지 않는다.

## 4. ROS의 기본 개념과 터미널 설정

ROS master는 노드·토픽의 주소록이고, driver는 하드웨어 상태를 토픽으로 내보내는 노드다. `/joint_states`는 관절 각도/속도, wrench는 힘(N)/모멘트(Nm)다. ROS1에서 master 주소가 맞아도 각 PC IP가 서로 도달 가능해야 실제 토픽이 흐른다.

로컬 PC에 드라이버와 제어기를 함께 실행하는 첫 구성에서는 새 터미널마다 다음을 적용한다. 예시 IP를 실제 유선 IP로 바꾼다.

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
unset ROS_HOSTNAME
export ROS_MASTER_URI=http://192.168.0.20:11311
export ROS_IP=192.168.0.20
```

본체 PC의 기존 master를 쓰기로 했다면 ROS_MASTER_URI는 그 PC 주소로 설정하고 ROS_IP는 이 노트북 주소로 유지한다. master는 하나만 사용한다. 기존 시뮬 노드가 같은 master에 `/joint_states`나 명령을 발행하지 않도록 종료/분리한다.

## 5. 첫 bringup: 관찰 전용

로봇 상태와 주변 공간은 현장 담당자와 확인한다. 모바일은 정지시키고 e-stop 위치/작동을 확인한다. 필요한 전원·브레이크 조작은 티치펜던트에서 장비 매뉴얼대로 한다. 여기서는 ROS로 전원/브레이크 명령을 보내지 않는다.

준비한 observe launch는 상태 controller만 시작하고 팔 운동 controller는 stopped로 로드한다. headless 자동 URScript도 사용하지 않는다. 기존 driver가 실행 중이면 이 launch 대신 기존 토픽을 확인한다.

```bash
# 실제 주소로 바꿔 사용자 실행. 이 명령도 UR에 네트워크 접속한다.
roslaunch ~/catkin_ws/hardware_setup/ur5e_observe.launch \
  robot_ip:=192.168.0.10 reverse_ip:=192.168.0.20
```

다른 터미널에서 위 ROS 환경을 똑같이 source/export한 뒤:

```bash
rosnode list
rostopic list
rostopic echo -n 1 /joint_states
rostopic hz /joint_states
rosservice list | grep list_controllers
# 루트 namespace로 새 driver를 실행한 경우
rosservice call /controller_manager/list_controllers "{}"
rostopic list | grep -E 'wrench|force|speed_scaling'
```

성공 기준은 관절 6개의 name/position/velocity가 계속 들어오고 motion controller는 stopped인 것이다. ROS 각도는 rad다. 펜던트 degree 값은 π/180을 곱해서 비교한다. namespace가 다르면 위 토픽·서비스 이름도 실제 목록에 맞춘다. 이 단계에는 External Control 프로그램을 Play할 필요가 없다. 공식 driver는 URCap 없이도 상태 확인 연결이 가능하다.

## 6. 로봇 교정 정보 저장

로봇별 factory calibration을 추출한다. 아래는 사용자 수동 실행 명령이고 아직 실행하지 않았다.

```bash
roslaunch ur_calibration calibration_correction.launch \
  robot_ip:=192.168.0.10 \
  target_filename:=$HOME/catkin_ws/hardware_setup/calibration/ur5e_factory.yaml
```

운동용 bringup에는 생성 파일을 `kinematics_config:=...`로 지정한다. 현재 HQP 모델은 가져온 nominal URDF다. driver에 교정 YAML을 넣는 것만으로 HQP URDF가 자동 교정되지는 않는다. SE3 실기 추종 전에 교정값이 HQP/시각 모델에도 같은 관절 프레임으로 반영되도록 해야 한다. 모바일에 대한 장착 변환도 별도로 확인한다. 이전 제어 모델에 있었던 180° fixed yaw와 새 모델의 0° 기준 차이를 실물 장착 방향과 대조한다.

## 7. URCap 설치: 실제 제어를 준비할 때

UR5e가 PolyScope 5.x인지 버전을 확인한다. 공식 ROS1 driver는 e-Series PolyScope >=5.9.4 지원을 명시한다. PolyScope X라면 이 파일/절차를 그대로 적용하지 말고 별도로 확인한다.

1. `downloads/externalcontrol-1.0.6.urcap`을 USB에 복사한다.
2. 펜던트 Settings → System → URCaps → + → 파일 선택 → 안내에 따라 재시작한다.
3. Installation → External Control에서 Host IP를 드라이버 PC의 유선 IP로 설정한다. Custom port는 기본값 50002를 사용한다.
4. Program에 External Control 노드를 추가하고 installation/program을 저장한다.
5. 관찰 전용 단계에서는 Play하지 않는다. 실제 첫 동작 준비가 끝난 뒤 현장에서 프로그램을 시작한다.

표준 bringup은 운동 controller가 활성화될 수 있으므로 관찰 전용 launch와 구분한다. headless_mode 자동 프로그램 실행, test_move 예제의 큰 동작은 첫 연결에서 사용하지 않는다.

## 8. 우리 HQP에 바로 연결하기 전 확인할 항목

| 항목 | 공식 driver 기본값 | 현재 HQP 실기 설정 |
|---|---|---|
| 관절 상태 | `/joint_states` | `/brl_robot/ur5e/joint_states` |
| 관절 이름 | `shoulder_pan_joint` 등 | `ur5e_shoulder_pan_joint` 등 |
| 팔 명령 | `/scaled_pos_joint_traj_controller/command` | `/brl_robot/ur5e/ur5e_scaled_pos_joint_traj_controller/command` |
| F/T | 실제 wrench 토픽과 frame_id를 읽어 확인 | `/brl_robot/ur5e/wrench`, `ur5e_wrist_3_link` |

토픽 remap만으로 관절명이 바뀌지는 않는다. 이름 변환/정확한 driver 설정이 필요하다. F/T frame_id 문자열만 바꾸는 것은 좌표 변환이 아니다. UR 내장 센서인지 외장 센서인지, 이미 중력/툴 하중이 보상됐는지 확인하고, 센서→wrist 회전과 원점 이동, 부호, bias 순서를 맞춘다. 시뮬에서 쓰던 음수 부호/Rx(-90°)를 실제 센서에 복사하지 않는다.

현재 `moveit_optimal_real.launch command_enabled:=false`는 명령이 꺼져 있고, 기존 real.cpp의 publish 호출도 주석 상태다. 실제 이름/프레임을 맞춘 뒤 먼저 command-disabled 상태에서 실제 q가 맞고 계산된 명령이 현재 자세와 가까운지 확인한다. 본 문서는 그 매핑이 이미 완료됐다고 가정하지 않는다.

소규모 동작으로 넘어갈 때는 모바일 정지, 낮은 속도(펜던트 5~10%부터), 현장 e-stop 담당자를 두고 현재 자세 기준 아주 작은 JointPosture 변화부터 확인한다. 시뮬 home으로 자동 이동시키지 않는다. HQP의 100 Hz 명령/10 ms horizon, driver speed scaling/stop 및 명령 타임아웃 동작은 실기에서 별도 검증해야 한다. 첫 추종 확인 단계에는 admittance를 일시적으로 끌 수 있고, F/T 보정/방향 확인 후 ON으로 HOLD/외력 시험을 진행한다. 모바일 x/y/yaw는 마지막에 별도 저속 검증한다.

## 다음에 필요한 현장 정보

- UR 컨트롤 박스 IP, 본체 PC IP, 제어 PC 유선 IP
- 펜던트 PolyScope 버전, External Control 기존 설치 여부
- 본체 PC의 ROS 버전·기존 bringup·master 주소
- 실제 joint_states 메시지 1개, wrench 메시지 1개(frame_id 포함), controller 목록
- 모바일 odom/cmd_vel 토픽과 실제 lateral(y) 주행 가능 여부

모르는 것은 모른다고 해도 된다. 먼저 관찰 단계의 출력으로 확인한다.

## 공식 자료

- https://github.com/UniversalRobots/Universal_Robots_ROS_Driver
- https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/usage_example.md
- https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_e_series.md
- https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases/tag/v1.0.6
- https://github.com/RobotnikAutomation/summit_xl_common

## 연구실 UR5e 단독: 명령 없는 HQP 검증

현장 모바일은 연결하지 않는다. `ur5e_shadow.launch`는 실행 중인 observe driver의 `/joint_states`와 `/wrench`만 읽으며 하드웨어 driver를 추가 실행하지 않는다. `ur5e_calibrated.urdf`는 공식 `/opt/ros/noetic/share/ur_description` xacro와 이 로봇의 `ur5e_factory.yaml`로 생성했다. 워크스페이스의 구버전 ur_description과 섞이지 않도록 생성 시 ROS_PACKAGE_PATH를 공식 패키지 경로로 한정했다.

- 공식 bare joint_names를 그대로 사용하고 HQP wrist_joint를 `wrist_3_joint`로 지정한다. 수신 배열이 elbow부터 시작해도 이름으로 매핑한다.
- `/wrench`의 stamped TF `wrist_3_link <- tool0_controller`를 적용한다. R·F와 R·τ + p×(R·F)를 계산해 `/hqp_shadow/wrench`로 발행한다. TF 도착을 위해 30 ms 지연하고, 해당 stamp의 변환이 없으면 샘플을 버린다. identity나 최신 TF로 임의 대체하지 않는다.
- Admittance는 ON, 명령은 `command_enabled=false`로 고정한다. 명령 토픽도 `/hqp_shadow/unused_command`로 분리했다. trajectory 입력은 `/hqp_shadow/trajectory`다.
- 실제 힘 방향/중력 보상 검증은 아직 하지 않았다. shadow 통과가 실기 운동/admittance 동작 검증을 의미하지 않는다.

기존 observe bringup을 유지하고, 새 터미널에서:

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch ~/catkin_ws/hardware_setup/ur5e_shadow.launch
```

다른 터미널에서 로봇을 밀거나 만지지 않은 상태로 약 2초 기다린 뒤:

```bash
source ~/catkin_ws/devel/setup.bash
rostopic echo -n 1 /moveit_optimal_controller/status
```

예상 상태는 HOLDING, command_enabled: False, ft_bias_ready: True이며 reason이 비어 있어야 한다. 실제 posture 기준 desired/command와 actual의 차이도 확인한다. External Control Play와 controller switching은 이 단계에서 하지 않는다. TF/입력 timeout이 나오면 그 원인을 해결하고 진행한다.

별도 master 11400에서 synthetic 입력으로 검증했다. 이름 순서 매핑, 90° 회전과 원점 이동이 있는 wrench 변환, HOLDING/bias-ready 및 명령 메시지 0개를 확인했다. 결과는 `shadow_mock_result.json`이다. calibration 모델의 native joint 이름으로 RT core도 torque limit on/off 각각 500 step malloc 0회, 기존 HQP/core 회귀 테스트 2개 통과. 실제 로봇에 대한 명령은 발행하지 않았다.

## URCap 설정 후: 첫 1도 관절 시험 준비

`ur5e_control_ready.launch`는 observe bringup을 대체한다. 동시에 실행하지 않는다. calibrated driver와 scaled position controller를 준비하지만 HQP, planner, 궤적 publisher는 실행하지 않는다. headless 자동 프로그램 전송도 비활성이다.

`first_joint_check.py`는 기본 실행 시 실제 현재 관절 상태를 이름으로 읽고 shoulder_pan만 +1°인 5초 궤적을 표시한다. 기본 모드는 action client 자체를 만들지 않는다. `--execute`를 사용자가 명시해야 실제 driver action goal을 보낸다. 이것은 HQP 이전의 공식 driver 동작 시험이다. 모바일/다른 관절 자동 목표/home 이동/자동 복귀는 없다. 오래된 관절 상태, 움직이는 로봇, 누락·중복 관절명, 비정상 값, 위치 한계 근처 목표를 거부한다.

현재 observe 상태에서 먼저 preview:

```bash
source ~/catkin_ws/devel/setup.bash
python3 ~/catkin_ws/hardware_setup/first_joint_check.py
```

운동 준비 시 shadow 및 observe launch를 Ctrl+C로 종료하고 다음 driver 하나만 실행한다:

```bash
roslaunch ~/catkin_ws/hardware_setup/ur5e_control_ready.launch
```

현장에서 팔의 1° 회전 공간과 e-stop 담당자를 확인하고 펜던트 속도를 5~10%로 설정한다. 저장한 External Control 프로그램을 수동 Play하고 driver의 `Robot ready to receive control commands.`를 확인한다. 그다음 controller 목록에서 scaled_pos_joint_traj_controller가 running인지 확인한다. driver가 기다리는 동안 임의 controller switching이나 test_move 예제는 실행하지 않는다.

사용자가 실제 +1° 이동을 실행하려면 아래 명령이다. 이 문서를 작성하면서 이 명령이나 실제 로봇 동작은 실행하지 않았다.

```bash
python3 ~/catkin_ws/hardware_setup/first_joint_check.py --execute
```

5초는 trajectory 시간이며 scaled controller가 펜던트 속도에 맞춰 늘린다. 10%에서는 대략 50초, 5%에서는 대략 100초가 될 수 있고 ramp/정지에 따라 더 길어진다. 성공하면 그 위치에서 끝나며 자동 복귀하지 않는다. 다음 HQP 실기 시험은 이 driver 시험과 별도로 구성한다.

## 기존 real.cpp ROS action으로 HQP 실기 연결

사용자 요청대로 topic MoveIt 노드 대신 기존 `rci_action_manager_real` 실행 파일과 `/rci_action_manager/jointControl`, `/rci_action_manager/se3Control` action을 재사용한다. 연구실에서는 whole_body=false, 실제 calibrated UR5e 6축 모델만 쓴다. 기존 MoveIt action은 이 launch에서 비활성이다.

변경 파일:

- `src/rci_action_manager/src/real/real.cpp`: 상태/명령/F/T 토픽 선택, 모델에서 관절명 추출, WrenchStamped 입력과 stationary bias, 입력 freshness/External Control/speed scaling gate, 조건부 명령 발행, status. 기존 commented publish 줄을 보존하고 별도의 command_enabled 분기를 추가했다. 실제 측정 `/joint_states`에 명령을 다시 발행하지 않는다.
- `controller.hpp/.cpp`: wrist joint와 wrench sign/gain/팔 속도 제한 파라미터. 기존 HQP와 CartesianAdmittance 그대로 사용. action generation으로 동시에 들어온 JointPosture/SE3가 서로의 task를 계산하지 않게 한다.
- `action_server_base.hpp`, `joint_posture_server.cpp`, `wholebody_server.cpp`: 속도 스케일을 반영한 trajectory clock, 잘못된 목표 거부, 도달 오차 확인 및 result.is_completed 설정. sim의 기본 speed scale=1은 유지한다. JointPosture name이 비어 있으면 기존 팔 순서를 유지하고, 있으면 이름으로 매핑한다.
- `hardware_setup/ur5e_actions_real.launch`: arm6, admittance ON, command ON 기본. 출력을 끄려면 command_enabled:=false를 명시한다.
- `hardware_setup/first_hqp_action.py`: native 관절명 JointPosture +1° 목표. 기본 preview, --execute만 실제 action goal 전송. 이전 first_joint_check.py의 공식 driver action 시험과 구분한다.

현재 운동용 driver bringup과 External Control 프로그램은 유지한다. shadow 노드 및 다른 HQP/명령 publisher를 종료하고:

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch ~/catkin_ws/hardware_setup/ur5e_actions_real.launch command_enabled:=false
```

외력을 주지 말고 약 2초 기다린 뒤 새 터미널:

```bash
source ~/catkin_ws/devel/setup.bash
rostopic echo -n 1 /real_action_controller/status
python3 ~/catkin_ws/hardware_setup/first_hqp_action.py
```

HOLDING, reason 빈 문자열, ft_bias_ready=True, command_enabled=False를 확인한다. 이 상태에서는 계산만 하고 명령은 보내지 않는다. 기본 preview는 action client도 만들지 않는다.

실제 움직임을 진행할 때 action launch만 Ctrl+C로 종료하고(UR driver는 유지):

```bash
roslaunch ~/catkin_ws/hardware_setup/ur5e_actions_real.launch command_enabled:=true
```

이제 HOLDING 중에도 admittance 명령이 발행된다. 현장 e-stop, 낮은 펜던트 속도, 팔 주변 공간을 확인하고 센서 bias 중 외력을 주지 않는다. 정상 status를 확인한 뒤:

```bash
python3 ~/catkin_ws/hardware_setup/first_hqp_action.py --execute
```

현재 자세 기준 shoulder pan +1°, 다른 관절 유지, trajectory 시간 10초다. 시간은 speed_scaling_factor를 적분하므로 10%에서 약 100초 + settling, 5%에서 약 200초 + settling이 걸릴 수 있다. 이동 중 스케일 0/프로그램 중지/입력 stale은 action을 abort하고 추가 명령 발행을 중단한다. action만 취소하면 HOLDING admittance가 계속 동작한다. 완전 중지는 현장 정지 수단을 사용한다.

명령은 100 Hz, 한 점의 50 ms horizon으로 native scaled_pos_joint_traj_controller/command에 보낸다. 최대 팔 qdot 0.03 rad/s, 측정 자세와 명령 차이 0.001 rad 제한, 실제 URDF 위치 제한을 확인한다. 이 값들은 실기 최초 시험을 위한 보수적 설정이며 이미 실기로 검증된 gain은 아니다. M/D/K 등 기존 action admittance 설정과 기존 SCA 비활성/주석 코드는 보존했다. 현재 hold는 기존 K=0 idle admittance다.

검증 결과(모두 하드웨어와 분리된 mock):

- native 이름/순서 섞인 JointPosture +1° 성공. scale=0.5에서 duration 1초+settling 2초가 wall 약 6.003초에 완료, 관절 최대 오차 0.001369 rad.
- malformed goal은 ABORTED, F/T 단절 시 action ABORTED 및 추가 명령 0개.
- command_enabled=false 시험에서 명령 메시지 0개.
- 기존 admittance controller 6/9축 및 JointPosture/SE3 회귀 실행 성공, HQP/core ctest 2개 통과.
- `real_actions_mock_result.json`, `real_actions_disabled_mock_result.json`에 수치를 저장했다. 실제 로봇 goal은 에이전트가 발행하지 않았다. SE3 실기 목표는 JointPosture 검증 후 진행한다.

### home 시험 중단 후 설정 변경

사용자가 home 실패를 보고한 뒤 상태/로그만 읽어 진단했다. 기존 home=5초와 arm_velocity_limit=0.03 rad/s는 90° 이동에 불가능한 조합이다. cubic peak 기준 약 78.6초 이상이 필요하며, 출력의 50 ms horizon과 driver 보간에 따른 추가 지연은 별도로 존재한다. 소리의 원인은 상태 한 번으로 확정하지 않았다. 실제 정지 상태에서 elbow 약 0.0246 rad가 관측되어 명령이 전혀 전달되지 않은 것으로 단정하지 않는다.

사용자 요청대로 `mm_client.py`의 home 시간만 60초로 바꾸고, `ur5e_actions_real.launch` 팔 속도 제한을 0.08 rad/s로 올렸다. 위치 제한, max_position_step=0.001 rad, input timeout, command_enabled 기본 OFF는 유지한다. 출력 적분 step 10 ms에서 qdot 0.08은 0.0008 rad 이내다. 펜던트 속도 스케일은 별도로 적용된다.

JointPosture 서버는 현재 자세와 cubic peak velocity로 불가능한 duration을 미리 거부한다. timeout/입력 이상/다른 action 대체 사유를 action status text와 ROS warning으로 남기고 mm_client가 이를 표시한다. 기존 목표값 및 나머지 명령의 duration은 유지했다.

새 0.08 설정 mock 시험: 5초 home 사전 거부(최소 이론 시간 약 29.48초), 60초 home 접수 후 즉시 취소, +1° action 성공(최대 오차 0.0000646 rad), malformed goal 거부, stale F/T 후 명령 중단을 확인했다. 60초 home을 끝까지 이동하거나 실제 로봇에서 실행한 검증은 아니다. 빌드 및 기존 HQP/core ctest 통과. 적용하려면 사용자가 실행 중인 action 서버와 mm_client를 종료 후 다시 실행해야 한다. driver bringup을 자동으로 재시작하거나 하드웨어 목표를 발행하지 않았다.

### 최종 요청: 1.57 rad/s 및 안전검사 실패 유지

사용자 최종 요청은 1.57 rad/s(약 90°/s)다. `ur5e_actions_real.launch`의 arm_velocity_limit 기본값과 100 Hz step 기준 20% 여유인 max_position_step=0.01884 rad로 반영했다. home duration은 60초 유지다. 공식 UR5e 사양/설치 모델의 6축 최대는 180°/s≈π rad/s이며 이 설정은 약 50%다. 이는 HQP 상한이며 궤적·펜던트 scaling·driver 보간의 실제 속도는 별개다. 공식 출처: https://www.universal-robots.com/media/1807465/ur5e_e-series_datasheets_web.pdf

보고된 반복 Warning 시점에 읽은 실행 파라미터는 여전히 arm_velocity_limit=0.08, max_position_step=0.001이었다(`action_fault_snapshot.json`). 기존 Warning에는 관절별 수치가 없어 원인은 확정하지 않았다. 신규 real.cpp는 검사 실패 시 관절명/actual/command/limit를 출력하고 fault를 latch한다. 입력이 회복되거나 다른 목표가 와도 명령을 자동 재개하지 않으며 노드 재시작이 필요하다. Idle admittance inverse-Jacobian qdot에도 같은 팔 속도 제한을 적용했다. 하드웨어 e-stop을 대체하지 않는다.

빌드, HQP/core ctest 2개 및 admittance regression 통과. 별도 master의 mock에서 +1° action 성공(최대 오차 0.0000597 rad), stale F/T 후 중단, 명령 step 위반 시 fault latch와 상태 복원 후에도 추가 명령 0개를 확인했다. 60초 home은 mock에서 접수 후 취소만 확인했고 실기 재실행은 하지 않았다. 이 설정 파일 변경은 실행 중인 서버에 적용되지 않으며 사용자가 서버를 재시작해야 한다.

### SE3 中 command-step fault 실제 원인 수정

사용자가 보고한 SE3 shoulder lift fault는 actual=-1.472248, command=-1.453288, step=0.018960 rad로 step limit 0.018840 rad를 넘었다. 코드에서 이전 idle hold 변경이 SE3 Control에도 적용되어 `ref_.manip_pos`를 누적 적분하고 있음을 확인했다. 이 경우 HQP qdot이 제한돼도 actuator 추종 지연이 측정-명령 차이에 누적된다. SE3 Control만 측정 state_.q 기준의 한 주기 적분으로 복구했다. Idle hold의 held reference 적분은 유지했다. JointPosture 적분과 속도/step 한계는 변경하지 않았다.

추가한 admittance_controller_test 회귀는 측정 q를 고정한 채 3초 상대 SE3 목표를 300주기 계산한다. 수정 전 실제로 실패했고, 수정 후 arm6 및 base3+arm6 모두 abs(command-q)<=1.570001*0.01을 만족했다. 기존 force/torque/reset/dimension 테스트와 HQP/core ctest 2개도 통과했다. 결과 요약은 se3_lag_regression_result.json이다. 실기 명령은 발행하지 않았다. 변경된 바이너리를 사용하려면 action 서버를 재시작해야 한다.

## 최신: action telemetry / Matplotlib / 부드러운 admittance

적용 대상은 현재 `real.cpp` JointPosture/SE3/HOLD action 경로다. 기존 HQP·CartesianAdmittance를 재사용하고 SCA 비활성/기존 주석을 유지했다. 아래 설정은 이전 절의 admittance 기본값보다 우선한다. 실기 F/T/명령은 에이전트가 직접 시험하지 않았고, mock 및 계산 테스트만 했다.

### 갑작스러운 힘 반응 수정

기존 action의 norm deadband는 힘이 4.5 N을 넘으면 0에서 전체 힘으로 갑자기 전환했고, M=1/D=10 및 기본 무한 속도/변위 한계를 사용했다. 최신 real launch는 `src/rci_action_manager/config/ur5e_action_admittance.yaml`을 로드한다.

| 항목 | 최신 값 |
|---|---|
| Translation M / D / K | 각 축 5 kg / 60 N·s/m / 0 N/m |
| Rotation M / D / K | 각 축 0.5 / 6 / 0 (rad 기반) |
| Deadband | 축별 2 N / 0.15 Nm, 크기에서 threshold를 빼는 연속 방식 |
| 저역통과 | 4 Hz |
| Cartesian 속도 | 축별 0.02 m/s, norm 최대 0.03 m/s |
| Angular 속도 | 축별 0.08 rad/s, norm 최대 0.10 rad/s |
| 가속도 | 축별 0.15 m/s², 0.5 rad/s² |
| 변위 한계 | 축별 0.10 m, 0.30 rad |
| 외력 반응 관절 속도/가속도 | 0.15 rad/s, 0.40 rad/s² (nominal tracking 1.57 rad/s와 분리) |
| Idle reference lag | 측정 관절로부터 축별 0.005 rad까지만 유지 + 한 주기 변위 |

필터는 action 전환 시 불필요하게 초기화하지 않는다. CartesianAdmittance에 가속도 및 속도 norm saturation을 추가했고, action/idle의 외력 관절 속도에도 별도 제한과 slew limit을 적용했다. HOLD reference가 servo 지연 중 누적되지 않게 anti-windup을 넣었다. K=0이라 밀린 위치를 유지하는 동작은 보존한다. 단단한 변위 한계에 도달하면 anti-windup이 속도를 0으로 만들 수 있다.

모든 gain/한계는 위 YAML에서 변경한다. 위 값은 실기 시작점이며 하드웨어에서 체감·안정성을 검증한 최종 gain이라고 보장하지 않는다. 툴 payload/TCP/센서 bias 오류는 필터만으로 해결되지 않는다.

### Safety 기준

HQP nominal 팔 속도 상한 1.57 rad/s와 URDF 위치 한계, 입력 timeout 및 fault latch를 유지한다. 기존 max_position_step 검사는 실제로 abs(command_q - measured_q)를 검사하므로 velocity step뿐 아니라 추종 lag도 포함한다. 이를 명시적으로 분리했다:

```
max_position_step = 1.2 * arm_velocity_limit * 0.01 + tracking_lag_margin
                 = 0.01884 + 0.015 = 0.03384 rad (~1.94 deg)
```

tracking_lag_margin은 launch arg이며 기본 0.015 rad다. 이것은 한계를 없앤 것이 아니고 비교량에 servo 지연 여유를 별도로 부여한 것이다. 위치 한계/비정상값/허용 gap 위반은 계속 latch되며 노드 재시작 전까지 추가 명령을 발행하지 않는다. 바깥 하드웨어 정지 기능은 별도로 유지해야 한다.

### 기록되는 데이터

`/real_action_controller/telemetry` (`rci_action_manager/ControlTelemetry`)를 제어 주기 100 Hz로 발행한다. Header는 제어 루프 시각이며, 실제 joint 입력은 freshness 검사 범위 내의 최신 측정이다.

- actual_pose: 실제 관절로 계산한 wrist FK
- nominal_pose: admittance 적용 전 태스크 궤적
- reference_pose: admittance 적용 후 HQP 태스크 기준
- translation_error: reference position - actual position, m
- rotation_error: log(R_reference * R_actualᵀ), rad, model world/base 축의 rotation vector. Euler angle 성분을 뺀 값이 아니다.
- joint actual/command/error, bias 보정 wrench/필터 적용 wrench, admittance 변위/속도
- action_id, 계산 mode, reason, command_enabled, ft_bias_ready, fault_latched

현재 UR5e에서는 Cartesian pose/error/displacement/velocity가 base_link 기준이다. Wrench 필드는 제어 wrist 원점의 wrist 축 기준으로 표시한다. command_enabled=True여도 fault/reason이 있으면 명령은 발행되지 않을 수 있다. 기록되는 joint_command는 계산값이다.

### 자동 CSV 기록과 사용 방법

`ur5e_actions_real.launch`는 record=true 기본으로 read-only logger를 별도 프로세스로 실행한다. 콜백은 bounded queue에만 넣고 CSV/JSON 디스크 쓰기는 worker에서 수행한다. 큐가 차면 dropped 수를 남기고, 제어 루프를 막지 않는다. 종료 시 큐를 비우고 메타데이터를 저장한다.

```
~/catkin_ws/artifacts/real_action_logs/YYYYMMDD_HHMMSS_microseconds/
  telemetry.csv
  metadata.json   # 실제 ROS gain/한계, 좌표 정의, 샘플/드롭 수
```

서버를 Ctrl+C로 종료해 기록을 마무리하고 plot한다. 루트 디렉터리를 주면 가장 최근 run을 선택한다. 개별 run 디렉터리 또는 CSV 파일도 입력 가능하다.

```bash
source ~/catkin_ws/devel/setup.bash
rosrun rci_action_manager plot_action_telemetry.py ~/catkin_ws/artifacts/real_action_logs
# Fault 이후 고정/잘못된 상태가 그래프 축을 지배하면, 원본 CSV는 유지한 채 fault 직전까지 표시:
rosrun rci_action_manager plot_action_telemetry.py ~/catkin_ws/artifacts/real_action_logs --until-fault
# GUI로도 보기
rosrun rci_action_manager plot_action_telemetry.py ~/catkin_ws/artifacts/real_action_logs --show
```

run의 plots/에 다음을 저장한다:

- trajectory_3d.png: EE XYZ 3D 궤적(명목/순응 기준/실제)
- trajectory_translation_xyz.png: 위치 XYZ 시간 이력
- trajectory_rotation_xyz.png: 초기 자세 대비 log(R(t)R(0)ᵀ) rotation vector XYZ, deg (큰 회전 π 근처에서 표현 branch 변화 가능)
- tracking_error_xyz.png: 위치 XYZ mm / 회전벡터 XYZ deg 오차와 nominal deviation
- tracking_error_3d.png: 위치·회전 오차 각각의 3D 경로
- wrench_filter.png: bias-corrected/filtered 힘·토크
- admittance_response.png: admittance 변위·속도
- summary.json: 유효 샘플 전체 및 mode/action_id별 각 축 RMS/최대, norm 최대. reason이 있거나 bias 미완료인 샘플은 통계에서 제외한다. 유효 샘플이 전혀 없으면 전체 통계로 fallback한다.

필요 라이브러리 numpy/matplotlib/scipy는 현재 PC에서 확인했다. record:=false로 recorder만 비활성화할 수 있고 log_dir:=...로 저장 루트를 바꿀 수 있다. rosrun rci_action_manager log_action_telemetry.py --output /원하는/run/경로로 수동 기록도 가능하지만 자동 logger와 중복 실행할 필요는 없다.

### 실기 최초 적용 순서

기존 action 서버를 종료하고 driver bringup은 유지한다. 새 서버는 기본 명령 ON이다. 최초 확인을 명령 OFF로 하려면 command_enabled:=false를 명시한다. 외력을 주지 않은 정지 상태에서 bias를 받고, status의 HOLDING / reason 빈 값 / ft_bias_ready=True와 actual-command 근접을 확인한다. 그다음 서버를 종료하고 command_enabled:=true로 재실행한다. 다시 bias가 끝난 뒤 이동 action 없이 HOLDING에서 작은 힘부터 시험한다. e-stop을 바로 사용할 수 있는 현장 상태에서 진행하고, 강한 충격이나 갑자기 손을 놓는 시험은 나중에 한다. 경고/fault/예상치 못한 방향이면 중단하고 CSV를 확인한다. 재시작 자체가 실기 안정성 검증을 의미하지 않는다.

### 검증과 한계

- 기존 HQP/core ctest 2개, 기존 6/9축 JointPosture/SE3 admittance 및 SE3 지연 회귀 통과.
- synthetic 200 N pulse + frozen measured robot: 6/9축 모두 linear velocity norm 최대 0.03 m/s, joint reference gap 최대 약 0.005916 rad, release 후 속도 감쇠 확인. 실제 접촉/백드라이브 시험이 아니다.
- synthetic 6 N, 0.8초 sensor pulse를 포함한 real.cpp mock: HOLD push/release, JointPosture action, 잘못된 목표 거부, F/T stale 중단, 위치 한계 fault latch 통과.
- mock CSV 5,949샘플 dropped=0. pose에서 재계산한 translation error와 CSV 일치, rotation-vector residual 약 1.24e-15. Matplotlib PNG 7개 및 summary 생성 확인.
- 예시 산출물은 `artifacts/action_telemetry_validation/20261105_202134_451682/`에 있으며 실제 로봇 기록이 아닌 mock이다.

### 회전 감쇠 조정

사용자 실기 피드백에 따라 ur5e_action_admittance.yaml의 회전 damping만 [6,6,6] → [3,3,3] Nm·s/rad로 낮췄다. 병진 damping 60, 회전 mass 0.5, deadband 0.15 Nm, angular velocity norm 0.10 rad/s, 각 축 acceleration 0.5 rad/s²와 나머지 보호 설정은 유지한다. 적용은 action 서버 재시작이 필요하고 재빌드는 필요 없다. 실기 명령을 발행하지 않았으며 실제 응답 변화는 새 로그로 확인한다.

### 명령 ON 기본값

사용자 요청에 따라 현재 사용하는 action launch의 command_enabled 기본값을 true로 변경했다. 이제 `roslaunch ~/catkin_ws/hardware_setup/ur5e_actions_real.launch`만으로 명령 ON이다. `command_enabled:=false`를 명시하면 계산/로깅만 한다. 예전 절의 OFF 예시 명령에는 false 옵션을 명시했다. 준비 상태·bias·freshness 검사와 fault latch, 자동 로깅은 유지한다. 이 변경을 검증하면서 실제 노드를 실행하거나 하드웨어 명령을 보내지 않았다.

## 최신: 상위 position command를 100 Hz로 통일

HQP 적분, position command 발행, JointTrajectory 점의 목표 시간을 모두 10 ms로 맞췄다.

- real.cpp 제어 루프는 100 Hz 유지, command_horizon 기본값 0.05 → 0.01 s.
- ur5e_actions_real.launch의 command_horizon arg와 C++ fallback 모두 0.01 s.
- JointPosture/SE3 적분은 기존 dt_=0.01을 사용한다.
- 메시지는 여전히 positions만 보낸다. 속도 제어로 전환하지 않았다. header.stamp=0으로 수신 시점을 시작으로 삼는다.
- UR 드라이버 내부 500 Hz와 상태 수신 500 Hz는 유지한다. Admittance의 4 Hz는 필터 차단주파수이며 제어 주파수가 아니다.
- Kp/Kd, admittance gain, 속도/보호 한계, 기본 명령 ON은 유지했다.

독립 master mock에서 real.cpp의 실제 발행 간격 중위수 0.0099999905 s, 모든 점의 time_from_start=0.01 s 및 position-only를 확인했다. 기존 action/F/T stale/fault latch 검사와 HQP/core ctest도 통과했다. ROS 원본 position-only spline과 가정한 30-ms 일차 관절 servo의 A/B 소프트웨어 모델에서는 양쪽 모두 8초 동안 800회 발행했으며, 10-ms horizon에서 모델 RMS가 0.046289 → 0.0279106 rad로 감소했다. 실제 UR 동역학을 식별한 모델이나 실기 성능 예측은 아니다.

산출물: artifacts/position_command_100hz/. 실기 명령은 발행하지 않았다. Action 서버를 재시작해야 적용되며 driver bringup은 유지할 수 있다. 실제 지연/떨림은 작은 이동 로그로 재검증해야 하며 100 Hz 정합만으로 모든 추종 지연이 사라진다고 보장하지 않는다.

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch ~/catkin_ws/hardware_setup/ur5e_actions_real.launch
# 다른 터미널에서 읽기만 하는 확인
rosparam get /real_action_controller/command_horizon
rostopic hz /scaled_pos_joint_traj_controller/command
```

예상값은 0.01과 입력/driver가 준비된 동안 약 100 Hz다.

## SE3 gain 비교: Kp 6 → 10

최근 실기 로그 20261105_205632_795852에서 10-ms horizon 적용 후에도 SE3 지연이 남았고 속도 제한 saturation은 없었다. 사용자 요청으로 mock 테스트를 생략하고 SE3 Kp만 6 → 10으로 변경했다. se3_kp launch arg를 추가해 비교값을 명시할 수 있다. Kd=0.1, JointPosture Kp=12, admittance M/D/K, 속도/보호 한계는 유지했다. 실제 실행은 하지 않았다.

```bash
roslaunch ~/catkin_ws/hardware_setup/ur5e_actions_real.launch se3_kp:=10
# 기존 값으로 돌아가 비교
roslaunch ~/catkin_ws/hardware_setup/ur5e_actions_real.launch se3_kp:=6
```

서버를 재시작해야 적용된다. rosparam set만으로 실행 중 gain이 바뀌지는 않는다. 각 run metadata.json에 gain이 자동 기록된다. 동일 시작 자세·동일 SE3 목표·같은 펜던트 속도로 비교한다. 한 번에 여러 gain/limit를 바꾸지 않는다.

### 다음 gain 비교값

사용자 요청대로 SE3 Kp 10 → 12, JointPosture Kp 12 → 15로 조정했다. joint_kp도 launch arg로 노출했다. Kd=0.1, admittance 및 속도/보호 한계는 유지한다. mock/실기 실행은 하지 않았다. 다음 실행은 `roslaunch ~/catkin_ws/hardware_setup/ur5e_actions_real.launch se3_kp:=12 joint_kp:=15`이며 서버 재시작만 필요하다. 각 gain의 효과는 JointPosture와 SE3 구간을 각각 이전 로그와 비교한다.

## 직접 gain 튜닝과 Kd 비교

실행 중인 action 서버를 Ctrl+C로 종료한 뒤 아래처럼 다시 실행한다. Driver bringup은 유지한다. 노드는 gain을 시작 시 읽기 때문에 rosparam set만으로 실행 중 gain이 변경되지는 않는다. 아래는 비교용 Kd=0.2이며 기본값은 0.1을 유지했다.

```bash
roslaunch ~/catkin_ws/hardware_setup/ur5e_actions_real.launch \
  se3_kp:=12 se3_kd:=0.2 joint_kp:=15
```

- SE3: se3_kp / se3_kd. Kd는 measured EE twist에 대한 속도 수준 피드백 계수이며 토크 PD의 damping과 같지 않다. 임계감쇠 2*sqrt(Kp) 공식을 그대로 적용하지 않는다.
- JointPosture: joint_kp. 현재 태스크는 P-only라 joint_kd 옵션은 없다.
- HOLDING 외력 반응: ur5e_action_admittance.yaml의 M/D/K. HOLDING에는 SE3 Kp/Kd가 적용되지 않는다.
- 세 옵션은 launch arg로 노출했다. 기본은 SE3 Kp=12, Kd=0.1, JointPosture Kp=15다. 기본 명령 ON은 유지한다.

SE3 Kp를 고정하고 Kd 0.1 → 0.2만 먼저 비교한다. 결과가 좋아야 다음 작은 증가를 고려하고, 노이즈/지연이 커지면 이전 값으로 되돌린다. JointPosture를 튜닝할 때는 joint_kp만 바꾸고 SE3 gain은 고정한다. 같은 home, 같은 목표/시간/펜던트 속도로 비교하며 metadata.json에 각 gain이 자동 기록된다.

최근 네 기록(205632/210518/210643/210750) 분석: Kp 6→10→12에서 SE3 위치 RMS는 42.6→28.0→23.7 mm로 감소했다. 최근 정지 HOLD의 joint command는 완전히 일정했고 filtered wrench와 admittance velocity가 0이었다. 3 Hz 이상 FK 성분 RMS는 위치 약 0.032 mm, 회전 약 0.0035°로 초기 정지와 비슷했다. 이동 중 회전 Y에 약 7.8 Hz의 작은 성분이 있고 고주파 peak-to-peak 약 0.035°였지만 큰 gain-induced 발산을 확인한 것은 아니다. 따라서 SE3 Kd를 크게 올릴 근거는 없으며 이동 구간에 한해 0.2 비교가 합리적이다. 정지 후 떨림은 SE3 Kd로 해결되지 않을 수 있다.

이 분석은 100 Hz 관절 FK 기반이므로 실제 기계 진동과 측정 노이즈를 단정할 수 없고 50 Hz 이상은 평가할 수 없다. 저장된 입력 joint velocity/twist가 없어 derivative feedback의 노이즈 영향도 완전히 분리하지 못한다.

```bash
# 최근 4개 run의 gain/미세 성분 비교. 현재 기본 mm_client의 3초 SE3 이동 구간을 기준으로 분석.
python3 ~/catkin_ws/hardware_setup/analyze_gain_vibration.py --last 4
```

최신 run의 analysis/gain_and_vibration_comparison.png 및 vibration_analysis.json에 저장한다. 모의/실기 goal은 실행하지 않고 파일만 읽는다.

### 추가 실기 로그 확인: 사용자 수동 튜닝 진행

분석 중 새 로그가 추가되어 최신 20261105_211352_713550까지 다시 확인했다. 실제 실행은 SE3 Kp=40, Kd=1, JointPosture Kp=40이며 SE3 위치 RMS는 약 9.09 mm다. Kp=15/Kd=.1은 19.08 mm, Kp=20/Kd=1은 17.41 mm, Kp=30/Kd=1은 12.05 mm였다. 여러 gain이 함께 달라 정확한 Kd 단독 효과를 판정할 수 없다.

최신 이동 구간 0.5~2.8초의 3-Hz high-pass 회전 Y 성분 RMS는 약 0.00875°, peak-to-peak 0.0378°, 주요 성분 약 8.59 Hz다. 이전 Kp12/Kd.1의 RMS 0.00790°, peak-to-peak 0.0354°와 비슷하고 줄어들었다고 보기 어렵다. 정지 후 위치 high-pass RMS 약 0.0318 mm, 회전 약 0.00359°이며 명령은 완전 고정이다. 이 측정만으로 물리적 떨림/센서 노이즈/툴 탄성을 분리하지 못한다.

이미 Kd=1을 사용한 최신 데이터 기준으로 추가 인상이 필요하다는 근거는 없다. Kp와 다른 값들을 고정하고 Kd=.5 vs 1.0처럼 한 값만 비교하는 쪽이 해석 가능하다. 예: `roslaunch ~/catkin_ws/hardware_setup/ur5e_actions_real.launch se3_kp:=40 se3_kd:=0.5 joint_kp:=40`. 기본 gain을 이 값으로 자동 변경하거나 mock/실기 goal을 실행하지 않았다. 앞 절의 .1→.2 제안은 당시 Kp12 로그 기준의 초기 비교안이다.

### 사용자 지정 순응 감쇠: 병진 30 / 회전 1.0

사용자 요청대로 ur5e_action_admittance.yaml의 damping을 [30,30,30,1.0,1.0,1.0]으로 변경했다. 질량, deadband, 속도/가속도/변위 한계 및 추종 gain은 변경하지 않았다. 서버 재시작으로 적용되며 실제 노드나 goal은 실행하지 않았다. 낮은 감쇠는 같은 힘에 더 쉽게 움직이고 손을 놓은 뒤 속도 감쇠가 더 오래 걸린다.

### C153 보고 후 사용자 요청 감쇠 비교값

사용자 요청에 따라 admittance damping만 병진 30 → 20, 회전 1.0 → 0.8로 한 단계 낮췄다. 질량, 속도/가속도/변위 및 관절 반응 한계, tracking gain, fault latch는 유지했다. 이 변경은 C153 해결을 보장하지 않으며 낮은 감쇠는 기준 속도와 release 후 잔여 움직임을 증가시킬 수 있다. 실제 보호 정지 원인 분석은 미완료다. 실제 노드/goal은 실행하지 않았으며 강하게 밀어 protective stop을 재현하는 시험은 안내하지 않는다. 적용은 서버 재시작이 필요하다.

### 최신: 직접 위치 출력 및 실기 MoveIt 토픽 노드

`ur5e_actions_real.launch`와 새 `ur5e_moveit_real.launch`는 공통 출력 클래스를 사용한다. HQP가 계산한 6축 위치를 `/hqp_joint_position_controller/command`에 `std_msgs/Float64MultiArray`로 100 Hz 발행한다. 드라이버의 500 Hz 주기는 유지한다. 출력 관절 순서는 shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3이며 모두 `_joint` 접미사를 붙인다. 시작 시 controller type/order/resource 충돌을 확인하고 같은 PC에서 같은 출력 토픽을 쓰는 두 HQP 노드의 동시 실행을 차단한다.

기존 HQP 서버를 Ctrl-C로 종료하고, bringup과 External Control은 실행한 상태에서 정지 상태를 확인한다. 아래 첫 명령은 조회만 하고, 두 번째는 실제 컨트롤러를 전환한다. 전환 스크립트는 직접 위치 컨트롤러를 로드하고 기존 관절 명령 컨트롤러를 정지시킨다. 관절 목표, Play, 보호 정지 해제는 보내지 않는다.

```bash
source ~/catkin_ws/devel/setup.bash
python3 ~/catkin_ws/hardware_setup/switch_ur_position_controller.py
python3 ~/catkin_ws/hardware_setup/switch_ur_position_controller.py --execute

# 둘 중 하나만 실행. 기본 명령 ON, admittance ON.
roslaunch ~/catkin_ws/hardware_setup/ur5e_moveit_real.launch
# 기존 action + mm_client를 사용할 때:
# roslaunch ~/catkin_ws/hardware_setup/ur5e_actions_real.launch
```

MoveIt 노드는 planner를 띄우지 않는다. 다른 팀이 `/rci_action_manager/moveit_plan`에 보내는 `trajectory_msgs/JointTrajectory`를 보간하고 HQP로 추종한다. 입력은 전체 궤적이고 하드웨어 출력만 직접 위치값이다. 상태는 `/moveit_optimal_controller/status`, telemetry는 같은 namespace의 `/telemetry`다. 작은 토픽 시험은 아래 순서로 직접 실행한다.

```bash
python3 ~/catkin_ws/hardware_setup/publish_moveit_joint_test.py
# preview 확인 후 +1도/5초 목표를 실제 전송:
python3 ~/catkin_ws/hardware_setup/publish_moveit_joint_test.py --publish
rosrun rci_action_manager plot_action_telemetry.py ~/catkin_ws/artifacts/moveit_real_logs
```

실기 MoveIt 설정은 calibrated UR5e 6축 모델, joint Kp=100, 속도 한계=1.57 rad/s, command gap=0.03384 rad와 동일 admittance YAML(M/D/K, 필터, deadband, 속도/가속도 제한)을 사용한다. D는 병진20/회전0.8, K=0이고 HOLD 및 TRACKING에서 admittance가 켜진다. K=0이므로 TRACKING 중 밀린 offset의 자동 원복은 없다. 원래 topic HQP의 가속도/예측 토크 제약은 유지하며, 관절 궤적 태스크에는 action SE3의 Kp/Kd를 적용하지 않는다.

직접 위치 컨트롤러에는 JointTrajectoryController의 보간/trajectory speed scaling이 없다. HQP 쪽 driver readiness와 speed scaling 시간 진행, 위치 간격/속도 제한은 유지하지만 이전 scaled JTC와 같은 응답을 보장하지 않는다. 기존 Kp=100도 새 전송 방식에서 작은 이동부터 재확인이 필요하다. 빌드와 기존 단위 테스트만 검증했으며 실기 컨트롤러 전환/동작 또는 mock bringup은 실행하지 않았다. 이전 `first_joint_check.py`는 scaled JTC 전용이므로 직접 위치 전환 후에는 위 토픽 시험 또는 기존 mm_client를 사용한다.

### RViz에서 실기 관절 상태로 계획하고 토픽으로 전달

bringup, External Control, 직접 위치 컨트롤러 및 `ur5e_moveit_real.launch`를 유지한다. action 서버는 종료한다. 별도 터미널 두 개에서 각각 실행한다.

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch ~/catkin_ws/hardware_setup/ur5e_moveit_planning_real.launch
```

```bash
source ~/catkin_ws/devel/setup.bash
python3 ~/catkin_ws/hardware_setup/send_rviz_moveit_plan.py
```

RViz MotionPlanning에서 group=manipulator, Start State=current로 선택하고 작은 목표를 지정한 후 **Plan**만 누른다. 터미널의 `Plan received`를 확인하고 `send`를 입력하면 최신 계획을 `/rci_action_manager/moveit_plan`으로 한 번 전송한다. 서버의 SUCCEEDED/REJECTED/ABORTED와 RMS/최대 오차를 출력한다. Plan & Execute/Execute는 연결하지 않았다. 새 동작은 현재 상태에서 새로 Plan한다. bridge를 먼저 켜면 계획 수신을 확인하기 쉽다.

이 launch는 bringup의 calibrated `/robot_description`, `/joint_states`, TF를 재사용하며 URDF를 덮어쓰거나 robot_state_publisher를 중복 실행하지 않는다. native SRDF/관절 한계/RViz 설정만 별도로 추가했다. planner joint velocity=0.3 rad/s, acceleration=0.4 rad/s²로 초기 계획은 보수적으로 유지했다. 테이블/공구/주변 장애물은 자동으로 생기지 않으므로 실제 환경에 맞는 planning scene이 필요하다. launch 파라미터 전개, Python 구문, SRDF 링크/관절 일치만 검증했고 실제 planner/실기 궤적 전송은 실행하지 않았다.

### Planner 속도 통일 수정

실행 중 읽기 전용 확인에서 HQP cap=1.57 rad/s, planner cap=0.3 rad/s였고, 최신 display plan은 86.1913초/base 최고속도0.03 rad/s였다. 이는 planner cap의 10%와 일치한다. UR driver speed scaling은 당시1.0이었다. native planner YAML을6축1.57 rad/s, 가속도 앞3축0.8/손목3축1.0 rad/s²로 HQP 기본값과 맞췄다. 위 이전 절의 planner0.3/0.4 값은 이 수정으로 대체된다. planner/RViz만 재시작하고 RViz Planning 탭의 Velocity Scaling 및 Acceleration Scaling을 확인한 후 새로 Plan해야 한다. scaling=1.0이면 설정된 한계 전체를 사용하며 기존 plan의 timestamp는 변경되지 않는다. 전달 클라이언트는 이제 계획 수신 시 duration/peak joint velocity를 출력한다. 현재 실행 중 파라미터/하드웨어 명령은 변경하지 않았다.

### HOLD offset 재적용 수정

3.65초 계획의 path abort 로그(action3)는 약0.22초에 최대오차0.263672rad였지만 command-to-actual 간격은 최대0.00824rad였다. 당시 admittance 병진 displacement는 [-0.1,-0.1,+0.1]m, 회전도 누적된 상태였다. K=0 HOLD로 이동한 현재 자세에서 새 계획을 만들었는데 이전 누적 offset을 새 궤적에 다시 더하는 시작 전환 문제가 있었다. 새 plan 수신 시 command 위치를 blend 시작점으로 잡고 admittance displacement/joint offset의 좌표 원점을 재설정한다. admittance 속도는 유지하고 blend 시작 속도에서 offset 속도를 제외하여 총 command 속도의 연속성을 맞춘다. path tolerance/게인/하드웨어 제한은 완화하지 않았다. 변경 후 추종 서버 재시작 및 현재 자세에서 재계획이 필요하며, 초기 bias 중 외력을 주지 않는다.

### 직접 위치 추종 지연을 반영한 gap 검사 완화

사용자 요청에 따라 action/MoveIt native launch의 gap 계산을 동일하게 변경했다. 이전 `1.2*velocity_limit*0.01+0.015`는 한 번의 publish 간격만 고려했다. 최신 실기 로그 wrist_3의 약40ms 추종 지연을 반영하여 `following_lag_seconds=0.04`를 노출하고 `1.2*velocity_limit*following_lag_seconds+tracking_lag_margin`으로 계산한다. 기본1.57rad/s에서0.03384→0.09036rad(약5.18도)다. 이는 순간 step 제한이 아니라 명령 위치와 측정 위치 사이의 총 following gap이다. fault latch, driver readiness, 속도/가속도/토크/path tolerance 및 로봇 자체 보호는 유지한다. 두 launch 파라미터 전개를 확인했으며 실기 동작은 실행하지 않았다. 서버 재시작 후 현재 자세에서 작은 목표를 재계획하여 검증한다. 이 변경은 UR C153의 해결을 보장하지 않는다. 이전 threshold는 필요시 `following_lag_seconds:=0.01`로 복원할 수 있다.

### 無접촉 wrench 진단용 TRACKING admittance OFF

사용자 요청으로 native 실기 MoveIt profile의 `tracking_admittance`를 false로 변경했다. TRACKING에서 wrench 입력만0으로 만드는 대신 누적 Cartesian displacement/velocity 및 joint offset/velocity도 우회하여 순수 계획 궤적을 추종한다. 시작 blend는 직전 전체 command 속도를 사용한다. 필터와 F/T 기록은 계속 유지하고 HOLDING에서는 기존 admittance를 다시 적용한다. action real.cpp 및 시뮬 기본 설정은 변경하지 않았다. 추종 서버 재시작 후 현재 자세에서 다시 계획한다. HOLDING에서도 무접촉 잔류 힘의 영향은 여전히 존재하며 이번 변경은 F/T 원인 해결이 아니다.

### 실기 MoveIt admittance 전체 OFF

사용자 요청으로 native MoveIt profile에 `admittance_enabled: false`를 추가했다. master OFF는 TRACKING/HOLDING 모두 Cartesian 변위/속도와 joint offset을0으로 유지하며, HOLD 기준을 측정값 쪽으로 재설정하는 admittance anti-windup도 우회한다. 따라서 HOLD에서 고정된 위치 기준을 유지한다. F/T 필터/로깅과 기존 readiness/safety 검사는 유지한다. 공통 클래스의 master 기본값은 true여서 다른 profile의 기존 동작은 유지한다. native MoveIt 추종 서버 재시작으로 적용하며 실제 하드웨어 명령은 보내지 않았다. 기존 action 서버 설정은 이번 변경 범위에 포함하지 않는다.

### 위치 기반 경로 추종 (native MoveIt 기본)

native MoveIt은 `geometric_tracking: true`, joint Kp30, 속도 한계6축1.0rad/s, admittance 전체OFF로 변경했다. 입력 JointTrajectory의 보간된 관절공간 경로를 비RT 콜백에서 폴리라인으로 준비한다. 원래 timestamp는 경로 보간과 미래 시작 시각에만 사용하고 완료 시간표로 사용하지 않는다. 시간 재조정/retiming은 하지 않는다. 기하 모드에서는 입력 궤적의 시간미분 속도/가속도가 controller 한계보다 크다는 이유로 reject하지 않으며 실제 출력은 기존 QP hard constraints로 제한한다. 위치/관절명/시작점/단조 시간/종점 정지/메시지 유효성 검사는 유지한다.

측정 관절 자세를 직전 progress 주변의 앞쪽 경로에 투영하고 progress가 감소하지 않게 한다. 검색은 관절공간 거리0.12rad 안의 최대64개 선분으로 제한한다. 전체 경로에서 최근접점을 찾지 않아 멀리 떨어진 교차구간으로 건너뛰는 것을 제한하지만, 검색창 안에서의 매우 촘촘한 교차까지 완전히 구별한다고 보장하지는 않는다. 목표는 progress보다0.03rad 앞이다(6축 유클리드 거리이며 TCP 미터 단위가 아님). planner 시간표의 속도/가속도 feedforward는 사용하지 않는다. 큰 gain으로 시간표를 따라잡는 대신 실제 로봇의 진행에 따라 목표가 전진한다. driver program/speed readiness와 안전 검사는 유지하며 driver speed scaling은 pursuit 목표 속도에 반영한다.

최종 구간에 도달한 뒤 실제 goal tolerance 및 측정 속도 조건으로 성공을 판정한다. 이전 nominal duration+4초 timeout 대신 경로 진행이15초 이상 멈추면 기존 goal timeout 결과로 abort한다. elapsed 상태값은 이 모드에서 실제 시작 후 경과 초다. desired/tracking error는 앞쪽 pursuit 목표와의 차이이며 최단 경로 횡오차 전용 지표가 아니다. 제약 아래의 추종으로 실제 완료가 더 늦어지는 것은 정상이다. QP는 여전히 속도 최적화이며 Cartesian 직선/시간 동기화/엄밀한 경로 오차 보장을 추가한 것은 아니다.

`path_lookahead`, `path_search_window`, `path_stall_timeout`은 native profile YAML에서 조절한다. lookahead가 커지면 빠르게 움직이려 하고 코너 오차도 커질 수 있다. 경로 샘플은 segment당20개 이상, derivative bound 기반 분할(최대4096개)로 만들며 전체200000점 초과는 reject한다. RT에서 경로 벡터를 확장하거나 전체 경로를 순회하지 않는다. 기존 action 서버 및 다른 profile의 시간 기반 기본 동작은 유지한다. 적용은 추종 서버를 재시작하고 현재 자세로 새 Plan 후 send한다. 실기 명령이나 controller switch는 직접 실행하지 않았다.

### 정상 계획이 INVALID_POINTS로 거절되던 빌드 불일치 수정

4.13초/13점 계획을 읽기 전용으로 확인한 결과 모든 position/velocity/acceleration은6차원이고 finite, 종점 속도0이었다. 실제 생성 path 상한도약1030점으로200000점 한계 미만이다. 실행 라이브러리의 buildPath 함수가 이전 void 구현으로 남은 반면 콜백은 bool 성공 반환을 기대하는 빌드 불일치를 확인했다. 관련 공통 라이브러리 파일을 재빌드하고, 단위 테스트에서도 valid path의 buildPath() 반환값이 true인지 명시적으로 검사하도록 수정했다. 경로 생성 실패시 전용 경고도 남긴다. 기존 source 편집 도중 진행 중인 빌드가 낡은 구현을 생성한 문제이며 입력 궤적이나 속도 제한을 완화해서 해결한 것이 아니다. 기존 실행 프로세스는 변경된 라이브러리를 자동으로 다시 읽지 않으므로 추종 서버 재시작이 필요하다.

### 사용자 비교 요청: 명시적 가속도 제한 OFF

native MoveIt launch에 `acceleration_limits` arg를 추가하고 기본false로 설정했다. QP의 이전 속도±가속도*dt bound만 제거한다. 속도1.57rad/s, Kp20, admittance OFF 및 위치/명령gap/driver/path 검사는 유지한다. 예측 토크 hard constraint는 별도로 켜져 있으므로 이것이 속도 변화량을 제한할 수 있으며 모든 물리적 가속도 제한이 사라지는 것은 아니다. 원복은 `roslaunch ~/catkin_ws/hardware_setup/ur5e_moveit_real.launch acceleration_limits:=true`로 한다. 다른 profile의 공통 기본값은true다. 실기 명령은 보내지 않았다. Kd 추가는 아직 적용하지 않았다.

### Kp50 및 HOLD 전용 admittance 재활성화

사용자 요청으로 native MoveIt joint Kp50, admittance master ON/tracking_admittance OFF로 변경했다. TRACKING에서는 compliance 변위/속도를 우회하고 HOLDING에서만 기존 K=0 admittance를 적용한다. 기존 회전 상태 rotation과 log displacement를 동시에 초기화하여 새 추종/우회 중 이전 회전 offset이 재생성되지 않도록 수정했다. 명시적 nominal 가속도 제한 OFF, 속도1.57rad/s 및 admittance 자체 속도/가속도/변위 한계는 유지한다. 무접촉 wrench 원인은 해결 전이므로 HOLD에서 다시 이동할 수 있다. 실제 명령/컨트롤러 전환은 실행하지 않았다.

### 읽기 전용 F/T 다중 자세 진단

추종 서버를 종료 후 `roslaunch ~/catkin_ws/hardware_setup/ur5e_moveit_real.launch admittance_enabled:=false`로 다시 켠다. bringup/External Control은 유지하고 초기 bias 동안 접촉하지 않는다. 이 옵션은 HOLD/TRACK 모두 admittance OFF이며 기본 프로필 HOLD ON을 영구 변경하지 않는다. 별도 터미널에서 `source ~/catkin_ws/devel/setup.bash` 후 `python3 ~/catkin_ws/hardware_setup/diagnose_ft.py`를 실행한다.

현재 정지 자세에서2초 이상 기다린 후 `A`를 입력하면3초간 기록한다. 기존 RViz Plan→send로 작고 충돌 없는 자세 변경을 하고, 완료/정지 후2초 이상 기다려 `B`, `C`처럼 기록한다. 서로 다른 TCP 방향을 포함해3~5개 자세, 마지막에 처음 방향으로 돌아오는 A_return이 유용하다. 이 진단 중에는 서버 재시작, F/T 영점/bias 재설정, Payload/CoG 변경을 하지 않는다. freedrive와 활성 HQP를 동시에 사용하지 않는다. 로봇 자세 변경은 진단 스크립트가 수행하지 않는다.

스크립트는 /wrench, /hqp_moveit/wrench, controller telemetry, /joint_states만 구독하고 로봇 publisher/service/parameter write를 만들지 않는다. master admittance OFF, 정상 HOLDING/bias ready, 메시지 신선도0.2초 및 source stamp 차이0.08초 이하, 관절 속도0.01rad/s 이하를 검사한다. 수집 중 관절 peak-to-peak0.003rad 초과 또는 충분한 샘플이 없으면 REJECTED로 표시한다. 정지 상태에서만 사용하는 근사 시간 정렬이며 고속 동적 wrench 식별용이 아니다.

출력은 artifacts/ft_diagnostics/시간/ 아래 pose_NN.csv, metadata.json, summary.json, wrench_by_pose.png다. 힘N, 토크Nm, 관절rad. CSV에 source별 frame/stamp, 관절 위치/속도와 measured wrist quaternion을 보존한다. 그래프의 raw는 driver wrench, wrist는 TF 변환 후, corrected는 시작 시 상수bias 차감 후, filtered는 LPF+deadband 후다. component 비교 전 frame을 확인한다. quit 또는Ctrl-C로 종료한다. 구문/OFF launch/합성 summary 및 PNG 생성/읽기 전용 API 검증만 했고 실기 자세 변경/센서 재영점은 실행하지 않았다.

### HOLD 복원(spring) 모드

사용자 요청으로 native MoveIt HOLD를 spring 모드로 변경하고 holding stiffness=[200,200,200,2,2,2]로 설정했다(병진N/m, 회전Nm/rad). damping=[50,50,50,1,1,1] 및 TRACKING admittance OFF는 유지한다. spring에서는 equilibrium hold_를 측정 자세에 따라 갱신하지 않도록 수정하여 고정된 기준으로 복원한다. 자유 hold 모드의 lag anti-windup 기준 갱신만 유지한다. 잔류 외력이 지속되면 spring도 F/K만큼 편위된 평형을 가지므로 영점오차 해결을 대체하지 않는다. 기본 launch로 재시작하면 적용되며 진단용 admittance_enabled:=false 옵션 사용시에는 동작하지 않는다.

### HOLD admittance 변위 상한 제거

사용자 요청으로 native MoveIt `admittance/displacement_limits_enabled: false`를 추가했다. 로더에서 CartesianAdmittance의 displacement_limit를 infinity로 설정하여 병진축별0.10m/회전축별0.30rad clipping을 제거한다. YAML의 displacement_limit 숫자는 다른 profile과 복원용으로 남지만 이 flag가false면 적용되지 않는다. spring 복원 강성150/2, 감쇠150/10, 병진 속도0.06/norm0.08, admittance 가속도/관절 속도·가속도 및 모델 관절 한계/명령gap 검사는 유지한다. 다른 profile의 기본 flag는true다. 회전은 기존exp3/log3 표현을 유지하므로 무한 회전 hand-guide를 구현한 것은 아니다. 서버 재시작으로 적용한다. 실제 로봇 명령은 실행하지 않았다.

### HOLD admittance 가속도 상한 제거

사용자 요청으로 native MoveIt `admittance/acceleration_limits_enabled: false`를 추가했다. CartesianAdmittance 병진0.15m/s²/회전0.5rad/s² clipping과 joint offset의0.4rad/s² slew limit를 함께 infinity로 우회한다. 일반 추종의 acceleration_limits=false는 기존대로 유지한다. YAML에 남은 acceleration_limit 숫자는 위flag=false일 때 실행 설정으로 적용되지 않는다. 속도 상한/관절 위치/명령gap/driver readiness/예측 토크 제약은 유지한다. 따라서 물리적인 가속도나 예측 토크에 의한 속도 변화 제약까지 모두 해제하는 것은 아니다. 다른 profile의 flag 기본값은true이며 기존 action real.cpp 설정은 변경하지 않는다. 서버 재시작으로 적용하고 실기 동작은 직접 실행하지 않았다.

### 연속 spline 및 부드러운 위치 기반 경로 진행

native geometric mode의 최종 목표를 폴리라인 보간 대신 원래 quintic spline에서 직접 평가한다. 샘플 폴리라인은 현재 자세의 국소 projection/거리 조회에만 사용한다. 전체 spline에 공통인 parameter_scale(샘플링된 최대 관절 미분 norm의1.02배)로 원래 매개변수를 정규화하여 구간 경계 q/v/a 연속성을 유지한다. 이는 MoveIt timestamp를 실행 시간표로 사용하거나 trajectory를 retime하는 작업이 아니다.

projection+lookahead가 주는 지점은 즉시 적용하는 목표가 아니라 scalar PathProgressGovernor의 목적지다. governor는 현재 진행 위치/속도/가속도를 시작 조건으로 갖는 quintic을 만들고, Bernstein derivative control 값으로 진행 속도>=0, 속도/가속도/jerk 한계를 검사한다. 최대80개의 horizon을 검사하며 새 C2 연결이 불가능하면 이전 유효 곡선을 계속 실행한다. projection을 따라 목표가 변해도 상태를 점프시키지 않고, 각 곡선의 목적지에서 속도/가속도0으로 멈춘다. 따라서 종점 감속은 governor가 미리 계획한다. native 기본path_speed=.5, path_acceleration=2, path_jerk=10이다. 이 한계는 정규화 spline 진행 상태 기준이며 곡률 항까지 포함한 모든 실제 관절의 hard acceleration/jerk 보장은 아니다. 실제 관절 속도/위치/예측 토크 hard bound는 기존HQP가 유지한다.

목표 관절 속도와 가속도는 chain rule(q_s*s_dot, q_ss*s_dot²+q_s*s_ddot)로 함께 계산한다. 새 궤적은 이전 reference q/v/a에서 새 시작점까지 transition_duration=.4초 quintic으로 연결한 뒤 진행한다. 이 handover는C2지만 scalar governor와 동일한 jerk bound를 별도로 검증하는 구간은 아니다. HQP solver 및 general acceleration_limits=false/HOLD admittance 설정은 변경하지 않는다. governor가 최종 진행 위치에 도달하고 정지한 뒤 실제 goal/velocity tolerance를 만족해야 성공한다.

실제 노드 publish 직전의 command_timing을 별도 기록한다. `~command_timing` Float64MultiArray는 [ROS publish time, steady-clock interval_ms, snapshot_age_ms, control_sequence_gap] 네 값이다. telemetry logger가 같은run의command_timing.csv로 저장하고 plot_action_telemetry.py가command_timing.png와 interval percentiles를 만든다. 이는 노드 발행 시각이며 드라이버/로봇 수신 시각 또는 네트워크 지연 측정이 아니다. 기존 telemetry stamp 간격과 구분해야 한다. 과거run에는 이 파일이 없을 수 있다.

프로필/구문/빌드 및 기존 단위 테스트 외에 spline knot C2, 이동하는 governor 목표에 대한 단조 진행/속도·가속도·jerk bound/종점 정지, 지연된 관절 응답에서 빠른 입력경로 추종, RT malloc0을 확인한다. 실기 노드/컨트롤러/로봇 명령은 직접 실행하지 않았다. 서버와 함께 실행되는 logger를 재시작하고 현재 자세에서 작은 계획을 보내 비교한다. 주파수는100Hz 그대로다.

### 연속 순항 개선: 중간 projection 목표의 정지 조건 제거

이전 가까운 목표마다 정지하는 quintic governor를 대체했다. 전체 경로의 실제 끝점까지7구간 S-curve(+jerk/일정가속/-jerk/순항/-jerk/일정감속/+jerk)를 한 번 구성한다. 긴 경로에서는 순항속도를 유지하며 짧은 경로는 도달 가능한 peak speed로 낮춰 실제 끝에서만 정지한다. spline 평가/C2 handover는 유지한다. native path_speed는0.5→1.0으로 올렸고 HQP6축한계1.57rad/s, Kp50, 명시적QP 가속도OFF 및 HOLD설정은 그대로다.

측정 관절과 직전 reference 사이의 norm 오차가path_lookahead=.03rad 이하이면 정상진행, 그두배=.06rad 이상이면 path clock 목표속도를0으로 낮춘다. 중간은 선형감소다. exact critically damped filter(path_clock_bandwidth=10/s)가 clock rate와 그 미분을 연속으로 바꾸며, clock의 정확한 적분만큼 S-curve를 전진시킨다. 따라서 느린 로봇을 무시하고 원래시간표를 진행하거나, projection마다reference를 리셋하지 않는다. 이soft lag feedback은오차를반드시.06rad안에가두는hardconstraint가아니며 기존path tolerance/command gap/fault검사를 유지한다. projection은 계속 국소진행과stall판정에사용한다.

path_acceleration2/path_jerk10은 가상시간의S-curve 한계다. clock시간변환 및 spline곡률을합성한실제관절가속도/jerk의hard bound는아니다. 목표속도/가속도에는clock rate와derivative를포함한chain rule을사용한다. 이번단위테스트의0.2rad경로/지연응답/속도cap0.4rad/s 조건에서완료3.71→1.61초, peak command0.095→0.4rad/s였고RT할당0이었다. 실제로봇최대속도에서의검증결과는아니다. S-curve순항/짧은경로종점정지/단조성/속도·가속도·jerk한계,clock연속성,기존HQP·core테스트를확인했다. 서버와logger재시작후작은계획으로실기확인한다. 로봇명령은직접실행하지않았다.

### 현장 real.cpp: 기존 scaled JTC에 한 점씩 출력

현장 master192.168.50.129, HQP PC192.168.50.80, UR driver node192.168.50.165, driver robot_ip192.168.0.210 구성을 읽기 전용으로 확인했다. 기존 External Control Host IP192.168.0.200 설정은 변경하지 않는다. 새 `hardware_setup/ur5e_actions_field.launch`는 기존 real.cpp/action 클래스를 재사용하며 모바일은 고정(whole_body=false)한다. 팔만 사용할 때는 base cmd_vel publisher도 만들지 않는다.

`prepare_field_ur5e_model.py`가 현장 `/brl_robot/robot_description`에서 ur5e_base_link→ur5e_wrist_3_link의6축과고정툴부착부를로컬field/ur5e_arm.urdf로추출했다. 바퀴/가동그리퍼와모바일3축은포함하지않는다. 원본URDF hash와정보는동명json에저장한다. 현장URDF기구학을재사용하는것이며factory calibration을추출하거나일치를확인한작업은아니다. 모델교체시 `python3 ~/catkin_ws/hardware_setup/prepare_field_ur5e_model.py`로다시준비한다(읽기전용ROS접근+로컬파일저장만).

출력 `/brl_robot/ur5e/ur5e_scaled_pos_joint_traj_controller/command`는 JointTrajectory이며6개ur5e_관절명, positions만있는점1개, stamp0, horizon기본0.02초를100Hz로갱신한다. 기존running scaledJTC를그대로사용하고motion controller를로드/전환하지않는다. 시작시list_controllers와joint설정으로이름/종류/running상태/자원충돌을확인한다. 명령ON기본, admittance OFF기본이며기존클래스에admittance_enabled 옵션을추가했다(다른launch의기본동작은true로보존). wrench로깅은OFF에서도유지한다. F/T는 `/brl_robot/ur5e/wrench`의ur5e_tool0_controller에서stampedTF로ur5e_wrist_3_link에변환하고100정지샘플bias를차감한다. 상태/pub은 `/real_action_controller/status`, `/real_action_controller/telemetry`, 로그는artifacts/field_action_logs다. SE3 제어점은graps_link가아니라wrist_3_joint의관절프레임이다.

현장에속도스케일publisher가없고scaled_controllers/SpeedScalingStateController plugin이설치된것을확인했다. fieldlaunch는읽기전용hqp_speed_scaling_state_controller를기존 `/brl_robot/ur5e/controller_manager`에로드/시작하여 `/brl_robot/ur5e/speed_scaling_factor`를공급한다. motion자원은claim하지않는다. 기존스케일publisher가있으면 `start_speed_scaling_controller:=false`로중복시작을피한다. 로봇프로그램/속도스케일topic은real.cpp파라미터로지정가능하게했으며기존기본경로는유지했다.

**실행전시간동기화필요:** 현장관절stamp가이PC보다약0.15초앞선샘플을확인했고다음측정에서도약0.066초앞섰다. 현재50ms미래시각검사를완화하지않았으므로이상태에서는readiness가막힐수있다. real.cpp가원인경고를출력하도록했다. NTP동기화표시yes만으로컴퓨터간시각일치를보장하지않는다. 제어서버정지상태에서같은LAN시간원으로맞추고검증해야한다. 현재로봇PC의UDP123에NTP응답은받지못했다(미설정/차단등원인미확정). Wi-Fi(wlp4s0)경로에서관절메시지약106ms수신공백도관측했으므로유선연결이바람직하다. 시계설정/컨트롤러/로봇명령은직접변경하거나실행하지않았다.

시간동기화/단독제어준비후사용자가실행할명령:

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch ~/catkin_ws/hardware_setup/ur5e_actions_field.launch arm_velocity_limit:=0.3
```

별도터미널에서미리보기후사용자가명시적으로실행:

```bash
source ~/catkin_ws/devel/setup.bash
python3 ~/catkin_ws/hardware_setup/field_action_joint_check.py
python3 ~/catkin_ws/hardware_setup/field_action_joint_check.py --execute
```

첫시험은현재자세에서pan+1도/5초이며자동복귀없다. 미리보기는actionclient/goal을생성하지않는다. 실행은fresh정지상태와서버HOLD/bias/command플래그를확인한다. 기존 `python3 ~/catkin_ws/src/rci_action_manager/scripts/mm_client.py`도같은글로벌action명으로사용가능하다. home base는+pi(180도)로수정했다. **나머지home5축은기존값**이므로현장현재자세와다르고큰이동이될수있어첫검증에는home대신작은action을사용한다. 다른MoveIt Execute/자율action클라이언트와동시에동일scaledJTC를제어하지않는다.

빌드/launch전개/Python구문/기존HQP·core단위테스트와추출한현장URDF의core테스트를확인했다. 관절방향이달라진모델에서도world변위의특정Z성분대신norm으로회귀시험전제조건을확인하도록테스트를수정했다. 실제field서버나speed-state spawner/action을직접실행하지않았다.

### 현장 wrench 변환 정지 진단/수정

원본 `/brl_robot/ur5e/wrench` 수신은 정상이나 실제 어댑터 health 로그에서received카운터만증가하고transformed=0/timer_alive=False임을확인했다. TF지연만이아니라변환Timer스레드가종료된상태였고, 종료예외의정확한원인은기존로그에없었다. 변환처리를monotonic wall-clock 메인루프로옮겨예외가주기스레드를조용히종료시키지않게했다. 예외시출력하지않고상세traceback을로그로남기며다음주기에bounded-age샘플을재시도한다. exact-stamp TF사용/최대0.2초샘플age/freshness gate는유지한다. 실제TF와wrench를구독하되출력은메모리로만보내는진단으로변환진행을확인했다. 실제/hqp_field/wrench나로봇명령을새진단노드에서발행하지않았다. Python수정이므로빌드는필요없고기존어댑터/서버launch를command_enabled:=false로재시작한뒤5~10초정지하여bias readiness와변환출력부터확인한다.

### 現장 nominal action이F/T freshness로abort하던의존성분리

현장+1도action로그에서5초목표중약3.6초에변환F/T가stale이되어readiness가끊기고abort함을확인했다. 원본F/T와controllerTCP TF는약500Hz이지만URDF팔link TF는10Hz여서변환wrench가12~14Hz이고간격이불규칙했다. admittance_enabled=false일때는제어에wrench를사용하지않으므로F/Tfreshness/bias를motion readiness필수조건에서분리했다. 관절신선도/driverprogram/speed scaling/position-gap검사는유지하고F/T수집·bias·로깅도계속한다. admittance_enabled=true에서는기존F/Tready/freshness검사를그대로유지한다. fieldactiontest도같은조건을읽으며action실패시status text를출력한다. activeaction중단시real.cpp가구체적인readiness사유를로그로남긴다. 변환TF10Hz병목은별도로해결할문제이며이수정으로admittance를고주파검증한것은아니다. 서버재시작후명령ON/admittanceOFF에서작은action을검증한다. 실제action이나컨트롤러전환은직접실행하지않았다.
