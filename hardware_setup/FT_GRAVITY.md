# BRL 툴 중력과 F/T 잔류 보정

`ft_gravity.py`는 URDF 질량 조회, 무접촉 자세 수집, 잔류 중력 모델 추정/검증, 보정 wrench 출력을 제공한다. 로봇 이동·controller 전환·UR Payload 변경·센서 영점 서비스 호출은 하지 않는다.

UR Payload/CoG와 Mounting을 먼저 실제 장착물에 맞춘다. UR의 Payload 측정 마법사는 질량과 CoG를 네 자세에서 추정한다. URDF에 기록된 그리퍼 질량 합계는 카메라/브래킷을 포함하지 않는다. 현재 F/T 스트림이 이미 UR에서 보상됐을 수 있으므로, 이 스크립트는 스트림에 **남아 있는 중력 성분**을 식별한다. 추정 질량/CoG를 펜던트에 그대로 입력하지 않는다. Payload/CoG, 툴 구성, 센서 영점 설정이 바뀌면 새 데이터로 다시 보정한다.

## 1. URDF의 값 확인

```bash
source ~/khu_ws/setup.bash
rosrun summit_ur5_hardware_setup ft_gravity.py inspect-urdf
# 로컬 파일 확인도 가능
rosrun summit_ur5_hardware_setup ft_gravity.py inspect-urdf \
  --urdf ~/khu_ws/src/summit_ur5_project/hardware_setup/field/ur5e_arm.urdf
```

전체 live 모델은 그리퍼 약0.828kg, arm6 snapshot은 약0.637kg만 포함한다. 카메라/브래킷의 inertial은 누락돼 있다. 출력의 CoM은 각 링크의 로컬 좌표이며 flange 기준 합성 CoG가 아니다.

## 2. 무접촉 정지 자세 수집

기존 HQP를 종료하고 다음처럼 **수집하는 동안 admittance OFF**로 재시작한다. 플래너/중계는 기존 BRL 설정을 사용할 수 있다. 자세 변경은 사용자가 RViz Plan 등으로 수행하며 수집 스크립트는 이동 명령을 보내지 않는다.

```bash
roslaunch summit_ur5_hardware_setup ur5e_moveit_field.launch admittance_enabled:=false
# 새 터미널: 공통 ROS 환경을 설정한 뒤
rosrun summit_ur5_hardware_setup ft_gravity.py collect \
  --output ~/khu_ws/artifacts/ft_gravity/poses.json
```

툴에 접촉/외력이 없는 상태에서 정지한 다음 `A`, `B` 같은 고유한 라벨을 입력한다. 라벨당4초, 최소20개 고유 F/T 시각을 수집한다. **위치 이동뿐 아니라 툴 방향을 두 축 이상으로 다양하게 기울인 최소6개 자세**가 필요하다. 가능하면8~10개와 첫 자세로 돌아오는 재현 측정을 포함한다. `quit`으로 끝낸다. 수집 중 Payload/CoG·bias·영점 설정은 바꾸지 않는다. 기울어진 로봇 베이스라면 `--gravity gx gy gz`로 실제 base-frame 중력 벡터를 지정한다.

관절 속도0.01rad/s 미만, 수집 중 관절 범위0.003rad 이하, fresh/healthy HOLDING, 정확한 시각의 wrist TF, 충분한 샘플 및 낮은 wrench 분산을 확인한다. **정지·낮은 분산만으로 무접촉을 자동 판별할 수는 없다.** 라벨 입력은 사용자가 무접촉 상태임을 확인했다는 뜻이다. 기존 파일은 덮어쓰지 않는다.

## 3. 추정과 독립 검증

```bash
rosrun summit_ur5_hardware_setup ft_gravity.py fit \
  ~/khu_ws/artifacts/ft_gravity/poses.json \
  --model ~/khu_ws/artifacts/ft_gravity/model.yaml
```

모델은 wrist 좌표에서 `F = m*g + bF`, `T = h × g + bT`이다. `g = Rᵀ * gravity_base`, `h`는 유효 질량 1차 모멘트다. UR 내부 보상의 잔류를 다루므로 유효 질량은 음수일 수도 있다. `effective_cog_m`은 잔류 모델의 파생값이며 실제 flange CoG나 펜던트 입력값으로 보장하지 않는다.

방향 다양성/조건수와 한 자세씩 제외한 교차 검증을 통과해야 파일을 만든다. 기본 교차 검증 RMS 한계는 힘1N, 토크0.08Nm다. 일관된 접촉이나 체계적인 모델 오차는 교차 검증으로도 놓칠 수 있으므로, 새로운 무접촉 자세도 확인한다.

수집에 쓰지 않은2~3개 방향을 `collect --output .../validation.json`으로 새로 저장하고:

```bash
rosrun summit_ur5_hardware_setup ft_gravity.py validate \
  ~/khu_ws/artifacts/ft_gravity/validation.json \
  --model ~/khu_ws/artifacts/ft_gravity/model.yaml
```

## 4. 보정값 적용

검증 후 기존 HQP를 종료하고 다음처럼 재실행한다. 보정은 기본 OFF다.

```bash
roslaunch summit_ur5_hardware_setup ur5e_moveit_field.launch \
  gravity_compensation:=true \
  gravity_model_file:=$HOME/khu_ws/artifacts/ft_gravity/model.yaml
# real.cpp action 경로에도 같은 모델 사용 가능. 두 HQP 중 하나만 실행한다.
roslaunch summit_ur5_hardware_setup ur5e_actions_field.launch \
  gravity_compensation:=true \
  gravity_model_file:=$HOME/khu_ws/artifacts/ft_gravity/model.yaml
```

보정 노드가 원본 wrist wrench에서 검증된 중력/고정 오프셋을 빼고 `.../wrench_compensated`로 발행한다. 컨트롤러의 startup bias는 이 보정 스트림의 남은 작은 고정 오차를 수집한다. 런타임에 접촉력을 학습하거나 계획 끝마다 자동 영점화하지 않는다. 원래 입력 wrench의 frame/stamp를 보존하고 정확한 시각의 TF가 없거나 입력이0.2초보다 오래되면 발행을 보류한다. 기존 F/T freshness 검사와 제어 제한은 유지한다.

이 모델은 정적 중력 보정이다. 툴 가속도에 의한 관성력, 케이블 장력, 온도 드리프트, 센서 비선형성은 별도로 남을 수 있다. 소프트웨어 시험은 합성 데이터로 수행하며, 현장 데이터 수집/무접촉 오차 및 실제 접촉 응답 확인은 사용자가 수행한다.
