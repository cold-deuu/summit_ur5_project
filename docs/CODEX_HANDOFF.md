# Codex 인수인계 — 2026-11-06 BRL 현장

이 문서는 이전 대화의 **작업 상태 요약**이다. 대화 원문/인증 정보는 포함하지 않는다. 새 컴퓨터의 Codex에서 README와 함께 읽고 실제 코드/실행 파라미터를 재확인한다.

## 새 Codex에 넣을 첫 메시지

```text
README.md, hardware_setup/README.md, docs/CODEX_HANDOFF.md를 읽고 이어서 작업해줘.
현장 실기이므로 읽기 전용 확인과 코드/빌드 검증부터 진행해줘.
로봇 목표, controller switch, URScript/Play/reset 명령은 직접 보내지 마.
현장 action 서버는 기존 scaled JTC에 한 점 JointTrajectory를 100 Hz로 보내는 경로야.
가장 먼저 실제 ROS master/네임스페이스/실행 파라미터와 빌드 환경을 확인해줘.
```

## 사용자 선호와 실기 경계

- 한국어로 간결하게 설명한다. 필요한 코드는 수정까지 진행하되 surgical하게 바꾸고 기존 주석/비활성 SCA를 보존한다.
- 기존 HQP/admittance 클래스를 재사용한다. solver를 근거 없이 교체하거나 같은 제어기를 다시 만들지 않는다.
- 실기 action goal, controller 전환, URScript, Play/reset 등은 에이전트가 직접 보내지 않았고 사용자가 수동 실행했다. 이후에도 이 범위를 지킨다.
- user가 직접 시험 후 로그를 보내는 흐름이었다. 새 검증은 작은 목표부터 하고, 실행하지 않은 실기 검증을 완료했다고 쓰지 않는다.
- command는 launch 기본ON/명시 옵션OFF 선호. 현장 admittance는 문제 분리를 위해 기본OFF다.

## 현재 두 실행 경로

### BRL field / real.cpp action

- launch: `roslaunch summit_ur5_hardware_setup ur5e_actions_field.launch`
- source: `rci_action_manager/src/real/real.cpp`, 기존 `controller.cpp`/action server 사용.
- 모바일은 고정, 팔6축만 제어한다. base publisher도 arm-only에서는 만들지 않는다.
- 기존 remote scaled JTC에100Hz, stamp0, positions-only1점, horizon0.02초를 계속 갱신한다.
- topic: `/brl_robot/ur5e/ur5e_scaled_pos_joint_traj_controller/command`
- joint state: `/brl_robot/ur5e/joint_states`; 이름은ur5e_ 접두사6축, 입력 순서를 이름으로매핑한다.
- raw F/T: `/brl_robot/ur5e/wrench`, frame `ur5e_tool0_controller`.
- adapter: `/hqp_field/wrench`, frame `ur5e_wrist_3_link`. 약90도회전/0.1m원점차가있으므로원본을identity로바로넣지않는다.
- status/telemetry: `/real_action_controller/status`, `/real_action_controller/telemetry`.
- 기본값: commandON, admittanceOFF, jointKp50, SE3Kp50/Kd1, nominal관절속도한계1.57rad/s.
- speed scaling state controller는현장에없어서읽기전용plugin만로드한다. motion controller를전환하지않는다.
- startup에서scaledJTC의type/running/jointnames를read-only검사한다.
- admittanceOFF에서는FTfreshness/bias가nominal motion을막지않는다. ON에서는기존FTready/freshness검사를유지한다.
- `field_action_joint_check.py --execute`: pan+1도/5초, 자동복귀없음. **사용자가실기SUCCEEDED를확인했다.**
- 기존client: `rosrun rci_action_manager mm_client.py`. home=[pi,-1.57,-pi/2,-1.57,pi/2,0],5초. base는현재+185도근처와가까운+180도로설정했다. -pi로치환하면unwrapped공간에서거의한바퀴돌수있다.

### 연구실 native UR5e / topic MoveIt

- launch: `ur5e_moveit_real.launch`, 별도 `moveit_optimal_real` binary.
- native barejointnames(no ur5e_). calibrated standalone model. fieldnamespace와그대로호환되지않는다.
- 기본하드웨어출력은JointGroupPositionController의Float64MultiArray100Hz다.
- 입력 `/rci_action_manager/moveit_plan` = JointTrajectory. 액션/MoveIt execution controller manager 연결없음.
- 현재는시간기반이아닌기하경로추종: 기존quintic spline직접평가,projection은국소탐색/진행/stall에사용한다.
- 초기짧은lookahead마다rest-to-rest quintic을넣어너무느렸음. 이를실제끝점만정지하는7구간S-curve+추종지연에반응하는smooth path clock으로수정했다.
- path_speed1.0/path_acceleration2/path_jerk10/path_clock_bandwidth10, Kp50, 관절속도한계1.57.
- 명시적QP가속도제약은OFF. scalar진행한계는spline곡률/clock까지합성한실제관절의hardacc/jerk제약과다르다.
- 현장action real.cpp에는이topic전용governor가적용되지않는다.

## Admittance 상태

- 공통클래스 `rci_holistic_controller/.../admittance/cartesian_admittance.hpp`.
- 연구실MoveIt: TRACKING OFF / HOLD spring ON. M5/.5, D150/10, K150/2.
- deadband2N/.15Nm, LPF4Hz, 병진속도축별.06/norm.08m/s, 회전축별.08/norm.10rad/s, 관절.15rad/s.
- 연구실native의변위clip과admittance가속도clip은사용자요청으로OFF. nominalQP가속도도OFF.
- fieldaction은별도 `ur5e_field_action.yaml`에유한변위/가속도한계를두며master기본OFF. 이차이를유지한다.
- 회전상태를재설정할때displacement뿐아니라rotation도identity로맞추는버그를수정했다.
- spring HOLD에서는평형점을측정자세로따라갱신하지않도록했다. 자유hold만laganti-windup을위해기준을갱신한다.
- 연구실무접촉다중자세기록에서상수bias차감후최대약7N잔류. 원본부터자세의존차이가있었고TF변환전후차이는약.034N뿐이었다.
- 단순중력+고정sensorbias모델잔차도수N이라자동보정값은적용하지않았다. Payload0.3kg/CoG0 설정과가벼운플라스틱부착이논의됐지만원인은확정못했다.
- 자동HOLD재영점은접촉힘을지울수있어구현하지않았다. D/K만조정해센서오차가없어졌다고주장하지않는다.

## 현장 통신/시간/TF

- ROSmaster192.168.50.129; URdriverhost192.168.50.165 (`sxls0-220429aa`,Ubuntu18.04).
- 동일URdriverPC의로봇망IP192.168.0.200, 로봇192.168.0.210. 펜던트ExternalControl Host0.200:50002유지.
- HQP개발PC는50.80, 사진의Dell노트북은50.105였다.
- 이전미래stamp150ms,이후66ms,다시수ms수신age를관측했다. systemd-timesyncd yes만으로PC간오차가보장되지는않는다.
- Wi-Fi에서관절수신공백최대324ms. 평균약500Hz이더라도고주기제어에는부적합할수있다. 유선/로봇PC로고주기루프이전검토가필요하다.
- rawFT와controllerTCP TF500Hz, arm link TF10Hz. joint_state_publisher relay/default rate가의심된다. 원격config나노드는자동변경하지않았다.
- wrenchadapter는원본stamp에맞는TF를기다리고.2초이내샘플만변환한다. 현재변환FT12~14Hz/불규칙이남아있다.
- 한때Timer스레드가죽어원본카운터만증가했다. monotonicwall-clock메인루프로옮기고상세예외/health로그를추가했다. 임의latest/identityTF로우회하지않는다.
- field+1도action첫실패는admittanceOFF에서도변환FTstale이readiness를막았기때문이었다. 이의존성을분리한후성공했다.

## 로그와 다음 작업

- `artifacts/field_action_logs`, `real_action_logs`, `moveit_real_logs`의실측CSV/영상은Git에올리지않았다.
- logger자동실행. `plot_action_telemetry.py <root> --show`가최신run과3D/XYZ/오차를그린다.
- topicnode에만command_timing.csv가있고노드발행간격/snapshotage/sequencegap을기록한다. 로봇수신timestamp가아니다.
- 우선순위: 유선또는로봇PC에서입출력간격비교 → 현장TF10Hz병목해결 → 무접촉FT검증 → fieldadmittance켜기.
- 현장MoveIt토픽노드연결은아직별도준비/검증이필요하다. fieldaction성공을MoveIt노드검증으로간주하지않는다.
- SCA제약은여전히비활성이고기존주석은보존했다.

## 저장소 이전 사항

- 원래작업폴더에는.git이없어원격ros-noetic에서brl브랜치를만들고소스/모델/하드웨어설정을복사했다.
- 업로드용체크아웃에서만portable path/manifest/CMake정리를했다. 실행중이던원본workspace는별도로남는다.
- workspace의build/devel/artifacts/zip/cache/개인인증파일은포함하지않는다.
- hardware_setup을별도catkin패키지로등록하여roslaunch/rosrun으로실행한다.
- clone후의존성설치스크립트와cleanworkspace `catkin_make -j2`를사용한다. 단위테스트는build에서ctest hqp_data_test/moveit_core_test.
- 제어코드빌드는Noetic20.04기준이며현장18.04driverPC에서의재빌드는별도호환성작업이다.
- 인증토큰은커밋/README/대화원문파일에포함하지않는다. 다음Codex에는이요약과코드만전달한다.
