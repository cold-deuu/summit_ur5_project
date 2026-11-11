# HQP 제어기: Ubuntu 18.04 + ROS Melodic 설치 (도커 없음)

이 폴더만 따라 하면 됩니다. 각 스크립트는 다시 실행해도 안전합니다.
브랜치는 `melodic-devel`.

## 0. 코드 받기

```bash
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone -b melodic-devel https://github.com/cold-deuu/summit_ur5_project.git
cd summit_ur5_project/hardware_setup/melodic
```

이미 클론돼 있으면 `git fetch && git checkout melodic-devel && git pull`.

## 1. 라이브러리 설치 (한 번, 약 10분)

```bash
./install_deps.sh
```

apt 패키지와 소스 빌드 3개를 설치합니다(`/usr/local`). 버전은 Noetic과 같습니다.

| 라이브러리 | 버전 | 18.04 apt를 안 쓰는 이유 |
|---|---|---|
| fcl | 0.6.1 | apt는 0.5 (API 다름) |
| toppra | 0.6.4 | apt에 없음 |
| eiquadprog | 1.2.9 | apt 1.2.5는 링크 에러 |

pinocchio는 apt 2.6을 그대로 씁니다.

**현장 PC에 인터넷이 없을 때**

```bash
# 인터넷 되는 PC에서
./install_deps.sh --fetch ~/hqp_deps_src
# → ~/hqp_deps_src 폴더를 USB로 현장 PC에 복사

# 현장 PC에서
./install_deps.sh --src <복사한 폴더>
```

apt 패키지는 인터넷이 필요합니다. 이미 설치돼 있으면 건너뜁니다.

## 2. 빌드

```bash
./build.sh                                  # ~/catkin_ws 기준, 제어기 패키지 5개만
source ~/catkin_ws/devel/setup.bash         # 새 터미널마다 (~/.bashrc에 넣어두기)
```

## 3. 설치 점검

```bash
./check.sh
```

모든 줄이 `[PASS]`여야 합니다. `[FAIL]`이면 바로 아래 `->` 줄의 명령을 실행하세요.

## 4. 로봇 없이 동작 시험

```bash
./selftest.sh                               # 약 1분
```

- **무엇을 하나:** 현장 launch 파일(`ur5e_moveit_field.launch`, `ur5e_actions_field.launch`)을 그대로 띄우고, UR 드라이버만 가짜(`fake_ur_driver.py`)로 바꿉니다.
- **시험 항목:**
  - 저장된 MoveIt 계획 → TOPP-RA → 추종
  - 잘못된 궤적 거부
  - JointPosture 홈
  - SE3 이동
  - 어드미턴스 반응
  - 텔레메트리 CSV 기록
- **안전:** 별도 ROS master(포트 11399)에서 돌기 때문에, 로봇이 켜져 있어도 실제 로봇에는 명령이 가지 않습니다.
- **기준값:** 결과를 Noetic 결과(`selftest_reference.json`)와 비교합니다. 마지막 줄이 `SELFTEST PASS`면 정상입니다.
- **로그:** `~/.ros/hqp_selftest/<날짜>/`

## 5. 실제 로봇 연결 점검 (로봇 bringup 후)

```bash
./check.sh --live                           # 네임스페이스가 다르면: ./check.sh --live /brl_robot/ur5e
```

점검 항목:
- `joint_states` 주기
- wrench
- External Control 실행 여부
- 속도 스케일
- `ur5e_scaled_pos_joint_traj_controller` 실행 여부
- `robot_description`

전부 `PASS`면 기존 안내대로 실행합니다.

```bash
roslaunch summit_ur5_hardware_setup ur5e_moveit_field.launch     # MoveIt 궤적 추종
roslaunch summit_ur5_hardware_setup ur5e_actions_field.launch    # JointPosture / SE3 액션
```

## 자주 나는 에러

| 증상 | 원인 / 해결 |
|---|---|
| `install_deps.sh`: `CMake 3.16 or higher is required` | toppra 줄 수정이 안 됨. `~/hqp_deps_src/toppra` 지우고 다시 실행 |
| 빌드: `multiple definition of ...solve_quadprog2` | apt eiquadprog 1.2.5가 잡힘 → `sudo apt remove ros-melodic-eiquadprog` 후 `build.sh` |
| 빌드: `Could not find a package configuration file provided by "toppra"` (또는 fcl, eiquadprog) | `install_deps.sh` 미실행 또는 실패. 다시 실행 |
| 빌드: `fatal error: filesystem: No such file` | `melodic-devel` 브랜치가 아님 (`git branch`로 확인) |
| 빌드: `Python has not been found` (eigenpy) | `sudo apt install python-dev`. `build.sh`는 python2.7 경로를 넘김 |
| 실행: `error while loading shared libraries: libtoppra.so` | `sudo ldconfig` |
| 실행: `/usr/bin/env: 'python3'` | `melodic-devel` 브랜치가 아님 |
| 실행: `Field trajectory controller is not running` | 드라이버의 `ur5e_scaled_pos_joint_traj_controller`가 꺼져 있음. `check.sh --live` |
| 실행: `Another local HQP node owns this command topic` | 다른 HQP 노드(moveit 또는 actions)가 이미 실행 중. 하나만 실행 |
| 상태 `External Control inactive` | 티치펜던트에서 External Control 프로그램 실행, 속도 슬라이더 > 0 |

Python 노드(텔레메트리, F/T, 서보)는 이 브랜치에서 Python 2/3 양쪽으로 돌도록 수정돼 있습니다.
시뮬레이션, 분석 도구, BRL 플래너는 18.04 대상이 아닙니다(Noetic 데스크톱용).
