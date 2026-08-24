# 프로젝트 동영상 촬영 계획

> 대상 프로젝트: **ROS 2 기반 실물 매니퓰레이터 Application/System Integration**  
> 촬영 목적: ROS 2 Application/System Integration 취업 포트폴리오의 실물 증거 확보  
> 최종 결과물: **60~75초 대표 영상 1개** + 원본 테이크 + 선택형 안전·자가정리 보조 영상  
> 원칙: 기능 수보다 **실물에서 인지부터 실행 결과까지 연결되는 한 번의 연속 증거**를 우선한다.

## 1. 촬영의 한 문장 목표

이번 촬영에서 반드시 확보할 장면은 하나다.

> 고정 RGB 카메라가 가운데 작업 구역의 빨간 블록 위치와 방향을 찾고, ROS 2 `pick`과 `place` Action을 통해 실물 팔이 블록을 `counter`로 옮긴 뒤 `home`으로 복귀하며 명시적인 성공 결과를 남기는 연속 장면

이 한 장면이 다음 연결을 눈으로 증명한다.

```text
RGB camera → perception → /scene_state → C++ Action skill
→ analytic IK → MoveIt 2 → ros2_control → DYNAMIXEL arm/gripper
→ success/failure result
```

다국어 LLM, 자가정리, 복구 상태 머신은 대표 실물 장면을 보강하는 차별점이다. 대표 장면을 대신하지 않는다.

---

## 2. 최종 산출물

| 산출물 | 필수 여부 | 권장 길이 | 내용 |
|---|---|---:|---|
| 실물 master take | 필수 | 35~50초 | 빨간 블록 `pick → place counter → home` 무편집 |
| PC 화면 녹화 | 필수 | master와 동일 | 검출 영상, Action 명령, 결과 |
| 셀 전경 establishing shot | 필수 | 3~5초 | 팔·카메라·구역·물리 전원 스위치 |
| 안전 기능 clip | 권장 | 8~12초 | 검증된 `move_to` cancel과 CANCELED 결과 |
| 자가정리 clip | 선택 | 8~15초 | 실물이 안정적이면 초록 블록, 아니면 Gazebo T3 |
| 복구 clip | 선택 | 6~10초 | Gazebo T4의 `GRIPPER_EMPTY → REGRASP` |
| 결과·한계 end card | 필수 | 7~10초 | 실측 수치와 정직한 한계 |
| 전체 무편집 원본 | 필수 보존 | 원본 전체 | 편집본의 신뢰성을 뒷받침하는 자료 |

---

## 3. 촬영 범위 결정

### 필수로 촬영

1. 실물 셀 전체
2. `/perception/debug_image`의 구역과 빨간 블록 검출
3. 빨간 블록 direct `pick`
4. 빨간 블록 direct `place` to `counter`
5. 그리퍼가 실제로 닫히고 물체를 들어 올리는 장면
6. place 후 `home` 복귀
7. 터미널의 Action 성공 결과

### 조건이 맞을 때만 촬영

- `move_to` cancel: 이전에 실물 검증한 절차를 그대로 재사용할 수 있을 때
- 초록 블록 자가정리: 촬영 당일 최종 리허설에서 `work → bin`이 안정적일 때
- 다국어 live command: RunPod가 안정적이고, 모델을 미리 warm-up했으며, 실물 master를 이미 확보했을 때

### 촬영하지 않음

- 파란 링을 대표 장면으로 사용
- 팔이 힘들어하는 양끝 위치
- 6개 구역이나 새로운 좌표 실험
- 실물에서 물체를 손으로 빼앗는 fault injection
- pick/place 동작 중 취소 시도
- 새로운 watchdog 조건이나 안전 동작을 현장에서 즉흥적으로 설계
- 10개 언어를 실물 팔로 전부 반복 실행
- RunPod 모델 전체 benchmark를 실시간으로 재실행

---

## 4. 왜 빨간 블록과 counter인가

### 빨간 블록

- 서로 다른 위치에서 camera-derived pick 3회 연속 실물 성공 기록이 있다.
- 파지 폭이 넓어 남은 카메라 위치 오차를 흡수할 수 있다.
- 파란 링보다 파지와 운반 반복성이 높다.
- 초록 블록과 형상은 같지만, 초록 자가정리는 agent와 방치 판정까지 함께 검증해야 해 대표 장면보다 변수가 많다.

### counter

- 현재 작업 공간에서 비교적 안정적인 `-y` 방향 목적지다.
- `bin`은 2026-08-22에 셀 안쪽으로 옮겨졌지만, 착지가 경계에서 1~2mm 차이로 갈린 실측이 있어 촬영 당일 리허설이 필요하다.
- `shelf`는 `+y` 영역 보정과 최근 구역 조정이 적용된 위치다.
- 대표 영상은 가장 어려운 위치가 아니라 가장 설명 가능하고 재현 가능한 경로를 사용해야 한다.

빨간 블록은 `work`의 정확한 경계가 아니라 **가운데 안전 영역**에 놓는다. 시각적으로 변화를 주기 위해 끝점으로 옮기지 않는다.

---

## 5. 촬영 전 보존

현재 프로젝트에는 실물 튜닝값, 구역 변경, 인지 오버레이, LLM 평가 CSV 등 미커밋 작업이 포함될 수 있다. 실물 테스트 전에 현재 상태를 잃지 않게 보존한다.

### 체크리스트

- [ ] `git status`를 확인했다.
- [ ] 현재 diff와 설정을 별도 위치 또는 Git에 보존했다.
- [ ] 최신 빌드가 성공한 상태인지 확인했다.
- [ ] 촬영 원본을 저장할 여유 공간이 있다.
- [ ] 휴대폰 배터리와 PC 화면 녹화 저장 공간이 충분하다.
- [ ] 촬영 당일 사용한 코드 상태를 나중에 찾을 수 있게 commit hash 또는 날짜를 기록했다.

최신 전체 테스트 결과가 현재 코드 기준으로 깨끗하게 재실행되지 않았다면 영상에 `All tests passed`를 넣지 않는다.

---

## 6. 하드웨어 사전점검

실물 동작을 시작하기 전 다음 순서로 확인한다.

### 6.1 물리 환경

- [ ] 팔 주변에 케이블, 공구, 휴대폰 거치대가 없다.
- [ ] 물리 전원 스위치에 즉시 접근할 수 있다.
- [ ] 사람이 팔의 작업 반경 밖에 있다.
- [ ] 카메라와 종이 구역이 마지막 보정 이후 움직이지 않았다.
- [ ] 조명이 일정하고 카메라 화면에 강한 반사나 그림자가 없다.
- [ ] 빨간 블록 외 불필요한 물체를 작업 구역에서 치웠다.
- [ ] 팔을 끌 때 받칠 준비가 되어 있다. 토크가 꺼지면 팔이 중력으로 무너진다.

### 6.2 ROS 2 bring-up

대표 실물 촬영은 에이전트를 끈 direct Action 모드로 시작하되, 실행 주체를 한꺼번에 올리지 않는다. 부록 C4의 절차대로 서로 다른 터미널에서 다음 순서로 하나씩 켜고, 각 단계의 통과 화면을 찍은 뒤에만 다음으로 간다.

```text
real_arm → real_camera → camera_tf → perception → MoveIt → skills
```

`agent`는 master를 확보하고 work 구역을 비운 뒤 선택 촬영에서만 마지막으로 켠다. 독립 launch들이 실행 중일 때 `real.launch.py`를 추가로 실행하면 controller와 node가 중복되므로 금지한다. 각 단계에서 바로 움직이지 말고 다음을 확인한다.

```bash
ros2 node list
ros2 action list
ros2 topic echo /scene_state --once
ros2 topic echo /dynamixel_hardware_interface/dxl_state --once
```

### 6.3 통과 조건

- [ ] `move_group`과 `skill_server`가 살아 있다.
- [ ] `/move_to`, `/pick`, `/place`가 모두 보인다.
- [ ] `/scene_state`에 `red_block`이 있고 좌표가 유한한 값이다.
- [ ] `/scene_state`의 frame은 로봇이 사용하는 world/base 계열과 일치한다.
- [ ] `/joint_states`가 약 100Hz로 들어온다.
- [ ] `/scene_state`가 약 30Hz로 들어온다.
- [ ] DYNAMIXEL `comm_state`와 hardware state에 오류가 없다.
- [ ] `dxl_state`의 `torque_state`는 초기화 뒤 갱신되지 않는 값일 수 있으므로 합격 판정에 쓰지 않는다.
- [ ] 작업 반경을 비운 뒤 그리퍼 `0.5` 무부하 시험에서 실제 개방·복귀가 확인된다.
- [ ] 실행 중인 client는 하나뿐이다.

### 즉시 중단 조건

- 그리퍼 무부하 시험에서 움직이지 않거나 힘이 없는 상태
- 빨간 LED, 비정상 진동, 반복적인 시리얼 오류
- 카메라 좌표가 이전 위치와 수 cm 이상 갑자기 달라짐
- `skill_server` 또는 `move_group` 재시작/종료 흔적
- 사람 손, 케이블, 촬영 장비가 작업 반경 안으로 들어감
- 팔이 테이블을 누르거나 관절 한계에서 계속 힘을 줌

이 경우 촬영을 계속하기 위해 여러 번 재명령하지 않는다. 전원·통신·controller·인지 중 어느 계층 문제인지 먼저 분리한다.

---

## 7. 카메라와 화면 구성

### 7.1 휴대폰 촬영

- 가로 화면
- 1080p 60fps 권장, 저장 공간이 부족하면 1080p 30fps
- 삼각대 또는 단단한 고정대 사용
- 팔의 정면이 아니라 30~45도 측면
- 팔 전체, 그리퍼, 빨간 블록, work, counter, 카메라가 한 화면에 들어오게 구성
- 가능하면 물리 전원 스위치도 화면 가장자리에 포함
- 디지털 줌 사용 안 함
- 초점과 노출을 팔/탁자에 고정
- 팔 동작 중 카메라를 따라 움직이지 않음

그리퍼만 크게 찍으면 물체가 어느 구역에서 어느 구역으로 이동했는지 보이지 않는다. 셀 전체를 보여주는 것이 우선이다.

### 7.2 PC 화면 녹화

화면을 두 영역으로 나눈다.

```text
┌──────────────────────────┬──────────────────────────┐
│ /perception/debug_image  │ ROS 2 Action 명령·결과  │
│ 구역·물체·검출 오버레이  │ accepted / success / code│
└──────────────────────────┴──────────────────────────┘
```

`rqt_image_view`를 실행하고 `/perception/debug_image`를 선택한다.

```bash
ros2 run rqt_image_view rqt_image_view
```

터미널 글자는 편집 후에도 읽을 수 있게 키운다. 너무 많은 topic과 로그를 동시에 띄우지 않는다. 핵심은 다음 세 가지다.

- detector가 `red_block`을 보고 있음
- 사용자가 정확히 어떤 Action goal을 보냄
- 결과가 명시적으로 성공 또는 실패로 끝남

### 7.3 동기화

휴대폰과 PC 녹화를 모두 시작한 뒤 다음 중 하나를 한다.

- 책상을 한 번 두드린다.
- 손뼉을 한 번 친다.
- “master take 1 시작”이라고 말한다.

편집할 때 소리 파형과 화면 반응을 맞추는 기준이 된다.

---

## 8. 필수 master take 실행서

### 8.1 촬영 직전 배치

- 팔: `home`
- 빨간 블록: `work`의 가운데 안전 영역
- counter: 비어 있음
- blue ring과 green block: 작업 화면 밖
- agent: `false`
- 실행 client: direct Action terminal 하나

### 8.2 실행 명령

새 터미널에서 컨테이너 환경을 불러온다.

```bash
sudo ./dc730 exec sim bash
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
export ROS_DOMAIN_ID=48
```

팔의 시작 상태를 확정한다.

```bash
ros2 action send_goal /move_to arm_interfaces/action/MoveTo \
  "{pose_id: home, attempt: 1}" --feedback
```

두 녹화를 시작하고 동기화 신호를 남긴 뒤 다음을 실행한다.

```bash
ros2 action send_goal /pick arm_interfaces/action/Pick \
  "{object_id: red_block, attempt: 1}" --feedback

ros2 action send_goal /place arm_interfaces/action/Place \
  "{object_id: red_block, target_id: counter, attempt: 1}" --feedback
```

현재 `place`는 놓기 이후 `home` 복귀까지 시도한다. 물체를 놓은 직후 바로 녹화를 끄지 말고 다음을 모두 담는다.

- 그리퍼 open
- retreat
- home 복귀
- Action 결과
- counter에 놓인 빨간 블록

### 8.3 master take 성공 판정

- [ ] `/perception/debug_image`에서 빨간 블록이 검출되었다.
- [ ] 팔이 블록 위로 접근할 때 불필요하게 양끝으로 가지 않았다.
- [ ] 그리퍼가 실제로 닫혔다.
- [ ] 블록이 테이블에서 완전히 들렸다.
- [ ] 운반 중 블록을 떨어뜨리지 않았다.
- [ ] counter에 블록이 놓였다.
- [ ] 팔이 물러나 `home`으로 복귀했다.
- [ ] `pick`과 `place` 결과에 명시적인 성공이 남았다.
- [ ] pick부터 place까지 영상이 끊기지 않았다.
- [ ] 사람 손이 동작 중 작업 반경에 들어오지 않았다.

master take가 성공하면 같은 장면을 더 예쁘게 만들기 위해 계속 반복하지 않는다. 우선 원본 두 개를 즉시 복사하고 재생해 확인한다.

### 8.4 실패했을 때

한 번 실패했다고 바로 같은 goal을 반복하지 않는다.

| 증상 | 먼저 확인할 것 |
|---|---|
| 물체를 못 찾음 | debug image, 새 scene frame, 조명, 카메라 위치 |
| 물체 옆에서 그리퍼가 닫힘 | 검출 좌표, 팔 가림, 카메라 보정 표류 |
| 그리퍼가 안 움직임 | DYNAMIXEL comm/hardware state, 전원, gripper joint feedback, 안전한 무부하 동작 |
| plan은 성공했는데 팔이 안 움직임 | controller, 시리얼, execute 결과 |
| 들었지만 떨어뜨림 | 파지 관절값, hold 목표, 운반 속도 |
| 목적지에 못 감 | target 좌표, 실제 reach, single-flight 상태 |

원인을 확인하지 않은 반복 시도는 성공 장면을 얻는 대신 서보 과부하와 상태 오염을 만든다.

---

## 9. 선택 촬영 A — 자연어와 다국어

이 장면의 목적은 LLM이 모터를 제어한다는 인상을 주는 것이 아니다. 자연어를 제한된 ROS 2 스킬 계약으로 번역한다는 것을 보여주는 것이다.

### 조건

- 실물 master take를 이미 확보했다.
- RunPod/Ollama 연결이 안정적이다.
- `gemma4:26b`가 미리 로드되어 있다.
- agent를 켜기 전에 작업 구역의 2초 자가정리 조건을 고려했다.

### 권장 방식

실물 팔을 여러 언어로 반복 움직이지 않고, 화면 녹화로 다음을 보여준다.

1. 한국어 명령 한 개
2. 일본어 또는 중국어 명령 한 개
3. 각 명령이 같은 `pick → place` 계획으로 변환됨
4. 실제 동작은 한국어 한 번 또는 direct master 영상으로 연결

명령 예시:

```bash
ros2 topic pub --once /command std_msgs/msg/String \
  "{data: '빨간 블록을 카운터에 놓아줘'}"
```

agent가 켜진 상태에서 direct Action 명령을 동시에 보내지 않는다. single-flight가 두 번째 goal을 거부하더라도 촬영 시나리오가 혼란스러워진다.

### 화면에 넣을 평가 수치

- `gemma4:26b`: 220/242, 90.9%, 중앙 지연 3.57초
- 다국어 부분: 59/60

자막에는 “10개 언어 제어 완성”이 아니라 다음처럼 쓴다.

> 10개 언어 60개 계획 변환 시험 · gemma4:26b 59/60  
> LLM은 고수준 스킬 계획만 제안하며 motor command를 생성하지 않음

---

## 10. 선택 촬영 B — 초록 블록 자가정리

자가정리는 “로봇이 스스로 작업을 만든다”는 프로젝트의 가장 독특한 시나리오다. `bin`은 2026-08-22에 셀 안쪽으로 옮겼지만 착지 여유가 작았던 실측이 있으므로, 촬영 당일 리허설을 통과할 때만 실물로 찍는다.

### 실물로 촬영하는 조건

- [ ] 촬영 당일 green pick이 안정적이다.
- [ ] `bin` place와 place 후 검출이 안정적이다.
- [ ] 카메라에서 bin 구역 전체가 보인다.
- [ ] 최종 리허설에서 자가정리가 완주했다.
- [ ] master take가 이미 백업되어 있다.

### 장면 순서

1. `IDLE` 상태와 빈 work 구역
2. 사람이 초록 블록을 work 가운데에 놓고 손을 뺌
3. 2초 방치 판정
4. `/robot_status`에서 자가 명령/RUNNING 표시
5. `green_block pick → bin place`
6. 검증 후 `cleaned` 증가

자가정리의 목적지는 코드의 고정 셀 정책이다. “AI가 초록색을 보고 불량을 추론했다”고 자막을 쓰지 않는다.

권장 자막:

> 작업 구역 2초 방치 감지 → 물체별 셀 정책으로 자가 작업 생성  
> green_block → bin

### 실물이 불안정하면

새 좌표나 6개 구역을 만들지 않고 기존 Gazebo 영상을 사용한다.

- [`media/edited/T3_자가정리.mp4`](media/edited/T3_자가정리.mp4)

화면 상단에 `GAZEBO SIMULATION` 또는 `SIM-VERIFIED`를 계속 표시한다.

---

## 11. 선택 촬영 C — 안전 기능

### 가장 가치 있는 장면

`move_to init` 실행 중 이전 실물 검증과 같은 방식으로 약 1.2초 시점에 cancel을 보낸다.

촬영해야 할 증거:

- 움직이던 팔이 목표까지 가지 않고 중간에서 감속 정지
- Action status `CANCELED`
- `ErrorCode.CANCELED=9`

### 실행 원칙

- 기존에 검증한 `cancel_goal_async` 절차 또는 스크립트만 사용한다.
- ROS CLI에서 `Ctrl+C`만 누르는 것을 물리 cancel 검증으로 간주하지 않는다.
- pick/place cancel은 지원하지 않으므로 시도하지 않는다.
- 스크립트가 즉시 준비되지 않으면 새 코드를 만들지 않고 검증 로그를 end card에 사용한다.

### 영상보다 문서가 나은 안전 항목

- stale scene gate: 팔이 움직이지 않는 것이 성공이라 영상 전달력이 낮음
- single-flight: 두 번째 goal 거부는 terminal evidence가 핵심
- watchdog: 강제로 1초 예산을 설정하는 재현은 이미 검증했으므로 촬영을 위해 반복할 필요가 없음
- 안전 종료: place 후 이미 home이면 시각적 변화가 적음

이 항목들은 대표 영상의 결과 카드나 별도 기술 설명에서 다룬다.

---

## 12. 기존 Gazebo 영상 재사용 계획

기존 `/media` 영상은 기능 설명은 좋지만 세로로 좁고 모두 simulation이다. 실물 master 뒤에 짧게 붙인다.

| 영상 | 사용할 구간 | 목적 | 권장 길이 |
|---|---|---|---:|
| [`T3_자가정리.mp4`](media/edited/T3_자가정리.mp4) | 방치 감지→자가 명령→초록 이동 | 자가 작업 생성 | 5~8초 |
| [`T4_강탈_복구.mp4`](media/edited/T4_강탈_복구.mp4) | 물체 상실→GRIPPER_EMPTY→REGRASP | 복구 상태 머신 | 6~8초 |
| [`T5_예산소진_직원호출.mp4`](media/edited/T5_예산소진_직원호출.mp4) | ABORTED_WAIT와 ignored | 무한 복구 방지 | 3~5초 |

전체 2분 39초 [`demo_v2.mp4`](media/edited/demo_v2.mp4)를 대표 영상으로 그대로 사용하지 않는다. 필요한 장면만 사용하되 simulation 표시는 가리지 않는다.

실물에서 물체를 빼앗는 복구 장면을 다시 만들지 않는다. 현재 주장은 “recovery state machine은 Gazebo에서 검증, 실물은 부분 검증”이다.

---

## 13. 최종 60~75초 편집안

| 시간 | 화면 | 자막·전달 내용 |
|---:|---|---|
| 0~4초 | 실물 셀 전경 | `ROS 2 Real Manipulator Application/System Integration` |
| 4~9초 | 카메라 debug image | `RGB perception → /scene_state` |
| 9~35초 | 실물 red pick/place 연속 | `C++ Action → analytic IK → MoveIt 2 → ros2_control` |
| 35~43초 | release, retreat, home, success | `명시적 Action result · false success 방지` |
| 43~52초 | cancel clip 또는 안전 결과 카드 | `cancel · stale gate · single-flight · watchdog` |
| 52~61초 | T3 또는 T4 Gazebo 짧은 장면 | `SIM-VERIFIED bounded recovery / autonomous cleanup` |
| 61~70초 | 실측 결과 카드 | 100Hz, 30Hz, red 3회 연속, arm 1~6mm |
| 70~75초 | 한계와 링크 | fixed cell · supervised · sim/real 범위 구분 |

편집본과 별도로 실물 master 무편집 원본 링크를 보존한다. 면접관이 원하면 전체 동작을 확인할 수 있게 한다.

---

## 14. 자막 문구

### 시작 제목

```text
ROS 2 기반 실물 매니퓰레이터 Application/System Integration
Perception → Planning → Control → DYNAMIXEL Hardware
```

작은 태그라인:

```text
다국어 지시와 셀 정책으로 정리 작업을 수행하는 AI 로봇 알바 프로토타입
```

### 실물 master 자막

```text
REAL ROBOT · supervised single-client
RGB camera에서 red_block 위치·yaw 검출
camera-derived pose → C++ /pick Action
gripper joint feedback으로 파지 확인
/place Action → counter → home
명시적 success result
```

### simulation 자막

```text
GAZEBO SIMULATION · recovery logic verification
GRIPPER_EMPTY → REGRASP → fresh scene → resume
```

### 결과 카드

```text
Real hardware evidence
· /joint_states ~100 Hz
· /scene_state ~30 Hz
· red-block camera pick 3 consecutive trials at varied positions
· arm target error 1–6 mm
· camera-based placement observation error 24–51 mm
```

### 한계 카드

```text
Scope
· fixed RGB camera / fixed table / known objects
· supervised research prototype, not industrial safety certified
· full recovery flow: Gazebo verified; real fault injection partial
```

---

## 15. 주장 가능·조건부·금지 표현

| 분류 | 표현 |
|---|---|
| 가능 | ROS 2 perception-planning-control-hardware integration |
| 가능 | camera-derived red pick 3 consecutive trials at varied positions |
| 가능 | arm target error 1~6mm와 camera observation error 24~51mm를 분리 측정 |
| 가능 | cancel, single-flight, watchdog, stale scene gate 실물 발동 검증 |
| 가능 | LLM은 제한된 고수준 스킬 계획만 생성 |
| 가능 | 자가정리·bounded recovery 상태 머신 Gazebo 검증 |
| 조건부 | 초록 블록 실물 자가정리 — 촬영된 전체 성공 증거가 있을 때만 |
| 조건부 | 파란 링 pick/place — 반복 성공률 없이 사례로만 |
| 금지 | 범용 객체 인식 |
| 금지 | 3D/6D pose estimation 또는 정식 extrinsic calibration 완료 |
| 금지 | 산업 안전 시스템 또는 무인 운전 |
| 금지 | 실물 자가복구 전체 완료 |
| 금지 | 성공률 100% |
| 금지 | LLM이 로봇 모터를 직접 제어 |

---

## 16. 테이크 기록표

촬영할 때 아래 표를 복사해 기록한다.

| 항목 | 기록 |
|---|---|
| 날짜·시각 | |
| take ID | REAL_MASTER_01 |
| Git commit 또는 diff 상태 | |
| 시리얼 포트 | `/dev/ttyACM__` |
| 카메라 장치 | `/dev/video__` |
| launch 옵션 | `stage:=hold agent:=false` |
| 물체 | `red_block` |
| 시작 위치 | work 중앙 / 대략 좌표 |
| 목적지 | counter |
| pick 결과 | success / code / 시간 |
| place 결과 | success / code / 시간 |
| gripper 관절값 | |
| phone 파일명 | |
| screen 파일명 | |
| 이상 소리·진동 | 없음 / 내용 |
| 최종 판정 | KEEP / RETAKE / DISCARD |

파일명 예시:

```text
real_master_red_counter_take01_phone.mp4
real_master_red_counter_take01_screen.webm
real_cancel_move_to_take01_phone.mp4
sim_T3_autocleanup_excerpt.mp4
portfolio_demo_v1.mp4
```

원본 파일을 `final.mp4`, `new.mp4`처럼 의미 없는 이름으로 덮어쓰지 않는다.

---

## 17. 촬영 후 즉시 할 일

1. 휴대폰 원본과 PC 화면 원본을 두 위치에 복사한다.
2. 처음부터 끝까지 재생해 실제로 끊기지 않았는지 본다.
3. 터미널 결과가 읽히는지 확인한다.
4. 그리퍼가 물체를 실제로 들었는지 프레임 단위로 확인한다.
5. phone과 screen 파일의 take ID가 같은지 확인한다.
6. master가 성공했으면 불필요한 실물 반복을 중단한다.
7. 편집본에도 REAL과 GAZEBO 표시가 명확한지 확인한다.
8. README와 one-pager에 넣을 링크는 최종 파일이 확정된 뒤 추가한다.

---

## 18. 촬영 완료 판정

다음이 모두 충족되면 촬영을 종료한다.

- [ ] 실물 red pick/place 무편집 성공 원본이 있다.
- [ ] 같은 take의 PC 화면 녹화가 있다.
- [ ] perception과 Action 결과가 읽힌다.
- [ ] master 원본이 두 위치에 백업됐다.
- [ ] simulation 장면에는 명확한 표시가 있다.
- [ ] 실측 결과와 한계 카드가 준비됐다.
- [ ] 60~75초 대표 편집본이 있다.
- [ ] 영상의 모든 주장이 handoff의 VERIFIED/PARTIAL 범위와 일치한다.

초록 자가정리, 파란 링, 다국어 실물 실행, watchdog 재현을 모두 촬영하지 못해도 종료할 수 있다. 이 프로젝트의 촬영 완료 조건은 기능 백과사전이 아니라 **실물 ROS 2 통합의 신뢰할 수 있는 대표 증거**다.

## 관련 문서

- 전체 프로젝트 소개와 토론: [`PROJECT_OVERVIEW_FOR_DISCUSSION.md`](PROJECT_OVERVIEW_FOR_DISCUSSION.md)
- 현재 프로젝트 기준선: [`handoff/MASTER.md`](handoff/MASTER.md)
- 실물 동작 실측: [`handoff/milestones/M6_G0_2026-08-19.md`](handoff/milestones/M6_G0_2026-08-19.md)
- 안전 기능 실물 검증: [`handoff/milestones/M6_SAFETY_2026-08-22.md`](handoff/milestones/M6_SAFETY_2026-08-22.md)
- 채용 담당자용 한 페이지: [`handoff/reference/ONE_PAGER_2026-08-22.md`](handoff/reference/ONE_PAGER_2026-08-22.md)

---

# 부록 A — 장면별 실행 명령어 전서 (2026-08-22 검증 세션 기준)

모든 명령은 sim 컨테이너 안에서 실행한다. 터미널을 열 때마다 아래 진입 블록을 먼저 실행한다.

```bash
# 컨테이너 진입 (둘 중 편한 것)
sudo ./dc730 exec sim bash          # 또는
sg docker -c "docker exec -it singlearm-sim bash"

# 컨테이너 안 공통 환경
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
export ROS_DOMAIN_ID=48
```

## A0. 기동

```bash
# 0-1. 시리얼 포트 먼저 확인한다. 전원을 껐다 켜면 ACM0↔ACM1 이 바뀐다(이번 주 2회 실측).
ls -l /dev/ttyACM*

# 0-2. 전체 스택 shortcut - 빠른 재실행용 참고다.
#      이번 독립 촬영은 이 명령 대신 부록 C4에서 실행 주체를 하나씩 올리는 방법을 사용한다.
#      C4의 standalone launch와 이 명령을 절대 동시에 실행하지 않는다.
#      camera/detector/skills 는 기본 true. agent:=false, port_name 은 0-1 결과로 바꾼다.
ros2 launch arm_bringup real.launch.py stage:=hold agent:=false port_name:=/dev/ttyACM0
```

## A1. 사전점검 (§6.3 의 명령형)

```bash
ros2 node list | grep -E "move_group|skill_server|object_detector|usb_cam"
ros2 action list                                    # /move_to /pick /place 셋
ros2 topic echo /scene_state --once                 # red_block 존재·frame=world 확인
ros2 topic hz /joint_states --window 50             # ~100Hz (Ctrl+C 로 종료)
ros2 topic hz /scene_state --window 20              # ~30Hz
ros2 topic echo /dynamixel_hardware_interface/dxl_state --once   # comm 0, hw 오류 0
# ※ dxl_state 의 torque_state 필드는 초기화 이후 갱신되지 않는 값일 수 있어 합격 판정에 쓰지 않는다.
# ※ controller가 active인 동안 get_dxl_data 직접 레지스터 읽기는 촬영 점검에 사용하지 않는다.
#    2026-08-23 실측에서 3초 service timeout과 연속 async-trigger 경고를 유발했다.
# 작업 반경을 비운 뒤 그리퍼 실동작 확인(0.5 벌렸다가 원위치):
ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand \
  "{command: {position: 0.5, max_effort: 0.0}}"
```

## A2. 화면 녹화 구성

```bash
# 오른쪽 창 : 검출 + 구역 오버레이 (금색 사각형과 이름이 함께 보인다)
ros2 run rqt_image_view rqt_image_view /perception/debug_image
# 아래 창 : 파지 판정·결과 로그 (skills 를 launch 한 터미널이 곧 이 로그다.
#   별도 창으로 보려면 launch 를 리다이렉트해 두고 tail 한다)
```

## A3. 필수 master take (§8)

```bash
# 시작 상태 확정
ros2 action send_goal /move_to arm_interfaces/action/MoveTo "{pose_id: home, attempt: 1}" --feedback

# [녹화 시작 + 동기화 신호 후]
ros2 action send_goal /pick arm_interfaces/action/Pick "{object_id: red_block, attempt: 1}" --feedback
# pick 이 SUCCEEDED 인 것을 확인한 뒤에만:
ros2 action send_goal /place arm_interfaces/action/Place \
  "{object_id: red_block, target_id: counter, attempt: 1}" --feedback
# place 는 놓기 -> 물러남 -> home 복귀까지 스스로 한다. home 도착까지 녹화 유지.
```

## A4. 선택 C — cancel clip (§11)

검증된 절차가 스크립트로 준비되어 있다: [`tools/cancel_demo.py`](tools/cancel_demo.py)

```bash
# init 으로 출발시키고 1.2초 시점에 취소 -> 공중 감속 정지 + 상태 5(CANCELED) 출력
python3 /ws/tools/cancel_demo.py
# 복귀
ros2 action send_goal /move_to arm_interfaces/action/MoveTo "{pose_id: home, attempt: 1}"
```

## A5. 선택 B — 자가정리 clip (§10)

```bash
# 1) master 확보·백업 후에만. agent 를 켠다 (이 순간부터 work 의 물체는 2초 방치되면 자동 정리된다)
ros2 launch arm_bringup agent.launch.py
# 2) 다른 터미널에서 상태 감시
ros2 topic echo /robot_status
# 3) [사람] 초록 블록을 work 가운데 놓고 손을 뺀다 -> 2초 뒤 자가정리 시작
#    기대 로그(agent): "자가 정리 실시 : green_block > bin" -> 시퀀스 완료 -> cleaned 증가
# 4) 촬영 끝나면 agent 종료 (Ctrl+C)
```

## A6. 선택 A — 자연어·다국어 (§9)

```bash
# 0) pod 살아 있는지 (HTTP 200 + 모델 목록에 gemma4:26b)
curl -s $OLLAMA_HOST/api/tags | head -c 300
#    ※ 404 = pod 꺼짐. 같은 pod Resume 이면 URL 유지, 새 pod 면 /ws/.env 의 OLLAMA_HOST 갱신 후
#      컨테이너를 재시작해야 환경변수가 반영된다.
# 1) agent 가 켜진 상태에서 (A5 의 1 참조):
ros2 topic pub --once /command std_msgs/msg/String "{data: '빨간 블록을 카운터에 놓아줘'}"
ros2 topic pub --once /command std_msgs/msg/String "{data: '赤いブロックをカウンターに置いて'}"
# 2) 화면용 평가 수치 재현(로봇 무관, 저장된 CSV 로 표만 출력):
cd /ws/src/arm_agent && python3 -m agent.compare_eval ../../eval_results/eval_*allmodels-k3*.csv
```

## A7. 종료 (안전 종료 데모를 겸한다)

```bash
# skills 를 띄운 터미널에서 Ctrl+C 한 번
# -> 팔이 스스로 "동작 중단 -> home 이동 -> 토크 유지" 로그를 남기며 종료된다.
# 전원 차단은 팔을 받친 뒤 물리 스위치로(사람).
```

---

# 부록 B — 본문 사실 검증 노트 (2026-08-22, 전체 코드 대조)

본문을 저장소 현재 상태와 대조한 결과다. 본문 문장은 보존하고 달라진 사실만 여기 적는다.

1. **§4·§10 "bin 은 가장자리" 서술은 낡았다.** 2026-08-22 오후 bin 을 (0.060, −0.225) 구석에서 **(0.054, −0.134) 안쪽**으로 옮겼다(팔이 실제로 놓는 자리 실측 기준, 4개 사본+정합성 테스트 갱신). 도달 반경 r=0.145, 카메라 시야 중앙권이라 초록 자가정리 실물 촬영의 리스크는 본문 평가보다 낮다. 다만 같은 날 bin 착지가 경계에서 1~2mm 로 갈린 실측이 있으니 §10 의 "최종 리허설" 조건은 유지한다.
2. **복구 실물 증거가 본문보다 두꺼워졌다.** 2026-08-22 링 자가정리 중 `GRIPPER_EMPTY → REGRASP(홈·재인지·재시도) → 복구 예산 소진 → ABORT` 전체 사이클이 실물에서 자연 발생으로 1회 완주했다(M6_SAFETY 원장). 본문의 보수적 주장("Gazebo 검증, 실물 부분")은 그대로 써도 되고, 원하면 "실물 1회 자연 발생 확인"까지는 말할 수 있다.
3. **§6.2 한 줄 기동은 유효하다.** `real.launch.py` 의 camera/detector/skills 인자 기본값이 전부 true 임을 launch 파일에서 확인했다. 단 `port_name` 기본은 `/dev/ttyACM0` 이고 전원 재인가 시 번호가 바뀐 실측이 이번 주 2회 있다 - 부록 A0 의 포트 확인을 반드시 거친다.
4. **§11 이 요구한 cancel 스크립트를 신설했다**: `tools/cancel_demo.py` (실물 검증 절차 그대로, PASS/FAIL 자동 판정).
5. **자가정리 방치 판정은 2.0초다**(`LOITER_SEC=2.0`). agent 가 켜진 동안 work 에 물체를 두는 순간 즉시 정리가 시작된다 - master take 는 반드시 agent OFF 로(§8.1 과 일치).
6. 링 관련: 어젯밤 사용자 지시로 링 하강을 수직 스텝에서 "한 번에"로 바꿨고 **미검증**이다. 본문이 링을 촬영 범위에서 뺀 판단(§3)과 정합한다.

---

# 부록 C — 채용 공고 역산 독립 촬영 라이브러리 (2026-08-23)

이 부록은 “로봇이 움직이는 예쁜 영상 한 편”이 아니라, 채용 담당자와 실무 면접관이 각 역량을 따로 확인할 수 있는 **증거 묶음**을 만드는 계획이다. 현재 공고에서 반복되는 요구인 ROS 2 구조화, 센서·액추에이터 통합, C++/Python, 실물 bring-up, motion planning, 장애 분석, test·logging·문서화를 역산했다.

핵심 촬영 규칙은 두 가지다.

> **한 클립에는 주장 하나만 담는다.**
>
> **클립 수는 많이 확보하되, 실제 팔의 pick/place 횟수는 늘리지 않는다.**

실물 master 한 번을 광각 휴대폰, 그리퍼 근접 카메라, 화면 녹화로 동시에 기록하면 `pick`, `grasp feedback`, `place`, `home`, Action result 등 여러 독립 클립을 안전하게 잘라낼 수 있다. 코드, 구조, 로그, 테스트, simulation, 설명 장면은 팔을 움직이지 않고 얼마든지 추가 촬영한다.

아래 라이브러리는 총 **100개 독립 클립 후보**다. 하드웨어 12개, 순차 bring-up 16개, perception 12개, 기구학·조작 16개, 안전·복구 15개, agent·LLM 13개, 코드·운영·본인 설명 16개로 나눴다.

## C1. 채용용 산출물 구조

| 산출물 | 길이 | 보는 사람 | 구성 |
|---|---:|---|---|
| 15초 teaser | 12~18초 | 채용 페이지를 빠르게 넘기는 사람 | 실물 lift, perception overlay, system title |
| 대표 영상 | 60~90초 | 채용 담당자 | 문제→실물 master→안전→수치·한계 |
| 기술 설명 영상 | 2~4분 | 로봇 SW 실무자 | architecture, bring-up, perception, Action, failure contract |
| deep dive | 5~8분 | 기술 면접관 | 코드 경계, IK test, sim-real, 장애 분석까지 설명 |
| 무편집 실물 원본 | master 전체 | 증거 확인자 | phone+screen 동기화, Action 시작부터 결과까지 |
| 독립 클립 폴더 | 장면당 4~20초 | 맞춤 지원·면접 | 아래 ID 단위의 짧은 B-roll과 terminal proof |
| 30~45초 자기 설명 | 30~45초 | 면접 시작 | 내가 만든 것·프레임워크를 쓴 것·한계 구분 |

대표 영상 하나에 모든 내용을 밀어 넣지 않는다. 지원 회사별로 독립 클립을 다시 조합할 수 있게 원본을 남기는 것이 목적이다.

## C2. 위험 등급과 촬영 원칙

| 등급 | 의미 | 허용 예시 | 원칙 |
|---|---|---|---|
| `GREEN` | 팔을 움직이지 않는 촬영 | 셀·배선, 코드, topic, test, log, Gazebo, 설명 | 필요한 만큼 반복 가능 |
| `AMBER` | 검증된 실물 동작 | 무부하 gripper, home, red master, 검증된 `move_to` cancel | 작업 반경 통제 후 성공 1회 확보 시 종료. 안정 세션에서도 항목당 최대 2회 시도 |
| `RED` | 촬영을 위해 재현하지 않는 위험 | 동작 중 serial 분리, 물체 강탈, 강제 watchdog, pick/place cancel, 링·가장자리·새 좌표 | 기존 log·Gazebo·도식으로만 설명 |

추가 원칙:

`GREEN`은 “동작 명령을 보내지 않는다”는 뜻이지, 통전된 팔이 무위험하다는 뜻은 아니다. arm이 켜진 뒤에는 등급과 무관하게 작업 반경을 비우고 물리 전원 스위치에 접근 가능한 상태를 유지한다.

1. `REAL ROBOT`, `SCREEN RECORDING`, `GAZEBO SIMULATION`을 영상 좌상단에 항상 구분한다.
2. 모든 독립 클립 시작에 1초짜리 slate를 넣는다: `ID / 날짜 / REAL|SIM|SCREEN / 주장`.
3. 각 동작 전후 5초의 정지 화면을 남겨 편집 여유를 만든다.
4. 실물 동작 중에는 촬영자도 팔 작업 반경에 들어가지 않는다.
5. 장애가 난 take는 지우지 않는다. 원인과 terminal result가 보이면 훌륭한 디버깅 증거가 될 수 있다.
6. 성공을 위해 조건을 바꿨다면 take sheet에 즉시 기록한다. 기록 없는 튜닝 take는 수치 증거로 쓰지 않는다.
7. 촬영 때문에 소스, 좌표, timeout, 속도, controller 설정을 즉흥적으로 바꾸지 않는다.

## C3. 촬영 세션 순서

각 세션은 다음 순서로 진행한다. 한 단계의 통과 화면을 찍은 뒤에만 다음 실행 주체를 올린다.

```text
0. 코드·장치 기준선 기록                              GREEN
1. 팔/robot_state_publisher/controllers               AMBER
2. RGB camera                                          GREEN
3. camera TF                                           GREEN
4. perception                                          GREEN
5. MoveIt 2                                            GREEN
6. C++ skill server                                    GREEN
7. direct Action master                                AMBER
8. agent/LLM                                           master 확보 뒤에만
9. Gazebo·test·log·설명 장면                           GREEN
10. 역순 종료, 팔은 마지막에 사람이 받친 상태로 종료
```

중간 중단 조건은 다음과 같다.

- 장치 포트가 예상과 다르거나 같은 노드가 두 개 이상 보임
- `comm_state != 0`, hardware error, controller inactive
- joint/scene rate가 현저히 낮거나 timestamp가 멈춤
- TF가 없거나 scene 좌표가 유한하지 않음
- 비정상 소리·진동·LED, 케이블 간섭, 카메라 이동
- 한 번의 명령이 terminal result 없이 남아 있음

중단 후에는 새 goal을 보내지 말고 해당 단계까지만 원인을 분리한다.

## C4. 실행 주체를 하나씩 올리는 촬영 절차

### C4.1 공통 준비

각 실행 주체는 **서로 다른 터미널**에서 띄우고, 터미널 제목을 `01_ARM`, `02_CAMERA`처럼 바꾼다. 모든 터미널에서 다음 환경을 먼저 적용한다.

```bash
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
export ROS_DOMAIN_ID=48
```

호스트에서 먼저 실제 장치를 기록한다.

```bash
ls -l /dev/ttyACM* /dev/ttyUSB* /dev/video*
lsusb
sg docker -c "./dc730 ps --all"
```

컨테이너 안에서 기존 실행 주체가 남아 있지 않은지 확인한다. 결과에 기존 real stack이 있으면 새로 겹쳐 띄우지 않는다.

```bash
ps -eo pid,ppid,stat,args | grep -E \
  "ros2 launch|ros2 run|move_group|skill_server|object_detector|usb_cam|ros2_control_node|agent" \
  | grep -v grep
```

### C4.2 01_ARM — 팔과 controller

사람이 팔을 안전한 자세로 받치고, 물리 전원 스위치에 즉시 접근할 수 있을 때만 실행한다. `stage:=hold`는 현재 자세를 잡는 controller까지 올린다.

```bash
ros2 launch arm_bringup real_arm.launch.py \
  stage:=hold port_name:=/dev/ttyACM0 start_rviz:=false
```

별도 확인 터미널:

```bash
ros2 control list_controllers
ros2 topic hz /joint_states --window 50
ros2 topic echo /dynamixel_hardware_interface/dxl_state --once
```

통과 기준은 `joint_state_broadcaster`, `arm_controller`, `gripper_controller`가 active이고, joint state가 약 100Hz이며, DYNAMIXEL 통신·hardware 오류가 없는 것이다. `/dev/ttyACM0`은 고정 사실이 아니므로 당일 확인값으로 바꾼다.

### C4.3 02_CAMERA — RGB camera

```bash
ros2 launch arm_bringup real_camera.launch.py video_device:=/dev/video0
```

```bash
ros2 topic echo /camera/camera_info --once
ros2 topic hz /camera/image_raw --window 30
```

640×480, 약 30Hz, `K`의 초점값이 0이 아님을 화면에 담는다. `/dev/video1`처럼 같은 장치의 metadata node를 캡처 장치로 잘못 선택하지 않는다.

### C4.4 03_TF — 카메라 좌표계

```bash
ros2 launch arm_bringup camera_tf.launch.py use_sim_time:=false
```

```bash
ros2 run tf2_ros tf2_echo world camera_optical_frame
```

`world → camera_link → camera_optical_frame`이 연결되고 값이 연속적으로 출력되는 5초를 기록한다. 이 값은 정식 6D extrinsic calibration 완료 주장이 아니라 현재 고정 셀의 실측·역산값이다.

### C4.5 04_PERCEPTION — 물체 검출

```bash
ros2 launch arm_bringup perception.launch.py use_sim_time:=false
```

```bash
ros2 topic echo /scene_state --once
ros2 topic hz /scene_state --window 20
ros2 run rqt_image_view rqt_image_view /perception/debug_image
```

red block을 work 중앙에 둔 정지 상태에서 ID, world 좌표, yaw, frame, 구역 overlay를 따로 촬영한다. detector 출력이 안정될 때까지 팔 명령은 보내지 않는다.

### C4.6 05_MOVEIT — motion planning

`move_group`은 `/robot_description`이 먼저 있어야 하므로 반드시 01_ARM 통과 뒤에 실행한다.

```bash
ros2 launch arm_bringup moveit.launch.py use_sim:=false start_rviz:=false
```

```bash
ros2 node list | grep move_group
ros2 action list | grep -E "move_action|execute_trajectory"
```

여기까지는 사용자 정의 `/pick`, `/place`, `/move_to`가 아직 없어야 정상이다. 표준 planning/execution 계층과 프로젝트 skill 계층이 분리되어 있음을 보여주는 장면으로 쓴다.

### C4.7 06_SKILLS — 사용자 정의 C++ Action

세션에서 검증한 trim을 그대로 사용한다. 아래 값은 문서 작성 시점 `real.launch.py`의 기본값을 standalone `skills.launch.py`에 풀어쓴 예시일 뿐이며, 그 자체를 새 성공값으로 간주하지 않는다. 촬영 당일 take sheet와 현재 launch 값을 먼저 대조한다.

```bash
ros2 launch arm_bringup skills.launch.py use_sim_time:=false \
  pick_center_y_trim:=0.045 \
  pick_counter_x_trim:=0.015 \
  pick_counter_y_trim:=0.020 \
  pick_shelf_x_trim:=0.0 pick_shelf_y_trim:=0.0 \
  pick_bin_x_trim:=0.0 pick_bin_y_trim:=0.0
```

```bash
ros2 action list | grep -E "^/(move_to|pick|place)$" | sort
```

세 Action이 보이는 화면을 기록하되 아직 목표를 보내지 않는다. skill server terminal에는 analytic IK, MoveIt, stale scene, single-flight, gripper feedback의 결과가 모이므로 화면 녹화 대상으로 유지한다.

### C4.8 07_AGENT — 판단·자가정리 주체

agent는 실물 master가 백업된 후, work 구역을 비운 상태에서 마지막으로 켠다. 켜진 동안 물체가 work에 2초 머무르면 자가정리가 시작될 수 있다.

```bash
ros2 launch arm_bringup agent.launch.py
```

```bash
ros2 topic echo /robot_status
```

처음에는 `IDLE`과 빈 작업 구역만 촬영한다. agent 실행 중 direct Action client를 동시에 사용하지 않는다.

### C4.9 중복 기동 금지와 종료 순서

- 위 독립 launch들이 실행 중일 때 `real.launch.py`를 추가 실행하지 않는다.
- 같은 `real_arm`, camera, TF, detector, MoveIt, skills, agent를 두 번 띄우지 않는다.
- 종료는 `agent → skills → MoveIt → perception → TF → camera → arm`의 역순이다.
- 실물 goal이 terminal state인지 확인하고 arm을 `home`으로 보낸 뒤 종료한다.
- `skills` 종료는 safe-park 동작이 생길 수 있으므로 작업 반경을 계속 비워 둔다.
- arm launch를 끌 때 토크가 풀려 무너질 수 있으므로 마지막에는 반드시 사람이 팔을 받친다.

## C5. 독립 촬영 목록 — 셀·하드웨어 (`H01~H12`)

| 우선 | ID | 한 클립의 주장 | 촬영 내용 | 방식·위험 |
|---|---|---|---|---|
| P0 | H01 | 실제 장비 프로젝트다 | 전원 OFF 상태의 팔·카메라·work/shelf/counter/bin 전체 5초 | phone, GREEN |
| P0 | H02 | sensor와 actuator를 직접 연결했다 | RGB camera에서 OpenRB, DYNAMIXEL 체인까지 천천히 한 번 훑기 | phone, GREEN |
| P1 | H03 | 셀 좌표는 물리 구역과 대응한다 | 네 구역 이름이 적힌 종이와 debug overlay를 컷 전환으로 비교 | phone+screen, GREEN |
| P1 | H04 | 하드웨어 안전을 고려했다 | 접근 가능한 물리 전원 스위치와 비어 있는 작업 반경 | phone, GREEN |
| P0 | H05 | Linux가 장치를 인식한다 | `lsusb`, `/dev/ttyACM*`, `/dev/video*`를 한 줄씩 보여주기 | screen, GREEN |
| P1 | H06 | container device 경계를 다룬다 | host와 container에서 같은 tty/video 장치가 보이는 비교 | screen, GREEN |
| P1 | H07 | 전원 재인가 시 포트 변경을 운영 절차로 관리한다 | take sheet의 실제 ACM 번호와 launch 인자를 같은 화면에 표시 | screen, GREEN |
| P1 | H08 | 카메라는 고정 셀 가정이다 | 삼각대, 카메라 광축, 테이블의 상대 배치 | phone, GREEN |
| P2 | H09 | 그리퍼도 상태 피드백이 있는 관절이다 | 전원 OFF 근접 정지 화면 + URDF joint 이름 자막 | phone+screen, GREEN |
| P2 | H10 | 케이블 간섭을 사전에 제거한다 | 정리 전/후를 짧게 비교하되 위험한 상태에서 전원은 켜지 않음 | phone, GREEN |
| P2 | H11 | 실제 물체의 크기·형상이 제한 조건이다 | red/green block과 blue ring을 자 옆에 정지 촬영 | phone, GREEN |
| P1 | H12 | 범용 로봇이 아닌 supervised fixed cell이다 | 전체 셀 정지 화면에 적용 범위와 한계 3줄 자막 | phone, GREEN |

## C6. 독립 촬영 목록 — 순차 bring-up (`B01~B16`)

| 우선 | ID | 한 클립의 주장 | 촬영 내용 | 방식·위험 |
|---|---|---|---|---|
| P0 | B01 | 기존 중복 프로세스 없이 시작한다 | 시작 전 process list가 비어 있는 화면 | screen, GREEN |
| P0 | B02 | 하드웨어 계층부터 먼저 올린다 | `01_ARM` launch 시작과 controller active 결과 | screen+wide, AMBER |
| P0 | B03 | 관절 feedback이 실시간으로 온다 | `/joint_states` 약 100Hz 결과 5초 | screen, GREEN |
| P1 | B04 | 통신과 서보 오류를 분리해 본다 | `dxl_state`의 comm/hardware 상태만 확대 | screen, GREEN |
| P0 | B05 | camera를 독립 노드로 올린다 | `02_CAMERA` launch 후 image topic 약 30Hz | screen, GREEN |
| P1 | B06 | calibration 정보가 입력 계약이다 | CameraInfo `width/height/K` 핵심 필드 | screen, GREEN |
| P0 | B07 | TF를 별도 계층으로 관리한다 | `03_TF` 시작 전 lookup 실패와 시작 후 성공을 비교 | screen, GREEN |
| P0 | B08 | perception은 camera와 TF 위에 올라간다 | `04_PERCEPTION` 뒤 `/scene_state`가 생기는 순간 | screen, GREEN |
| P0 | B09 | 로봇 모델이 먼저 있어야 MoveIt이 산다 | arm 이후 `05_MOVEIT`, `/move_group` 확인 | screen, GREEN |
| P1 | B10 | 표준 실행 Action과 custom skill을 구분한다 | skills 전 action list의 MoveIt action만 표시 | screen, GREEN |
| P0 | B11 | custom Action 세 개를 마지막 계층에 추가한다 | `06_SKILLS` 뒤 `/move_to`, `/pick`, `/place` 출현 | screen, GREEN |
| P1 | B12 | agent는 motor driver가 아니라 orchestration 계층이다 | `07_AGENT` 뒤 `/robot_status=IDLE` | screen, GREEN |
| P0 | B13 | 전체 시스템을 node graph로 설명할 수 있다 | 단계가 모두 뜬 뒤 `rqt_graph` 또는 간단한 고정 diagram | screen, GREEN |
| P1 | B14 | launch parameter가 실물 조건을 명시한다 | `use_sim_time=false`, port, camera, trim 인자 화면 | screen, GREEN |
| P1 | B15 | 종료도 설계 대상이다 | agent부터 역순 종료하고 마지막에 arm을 받치는 장면 | phone+screen, AMBER |
| P2 | B16 | 한 줄 launch와 독립 launch는 목적이 다르다 | `real.launch.py` 조립 구조를 코드에서 설명하되 실제 중복 실행 안 함 | screen+narration, GREEN |

## C7. 독립 촬영 목록 — perception·좌표계 (`P01~P12`)

| 우선 | ID | 한 클립의 주장 | 촬영 내용 | 방식·위험 |
|---|---|---|---|---|
| P0 | P01 | RGB 원본이 perception의 입력이다 | `/camera/image_raw`의 정지 셀 5초 | screen, GREEN |
| P0 | P02 | detector가 ID와 형상을 찾는다 | debug image의 red contour, 중심, 긴 축 | screen, GREEN |
| P0 | P03 | 픽셀을 world 좌표로 바꾼다 | 같은 물체의 화면 중심과 `/scene_state` x/y/yaw를 나란히 | screen, GREEN |
| P0 | P04 | 구역 규칙이 시각적으로 검증된다 | work/shelf/counter/bin overlay와 실제 종이 경계 비교 | phone+screen, GREEN |
| P1 | P05 | camera intrinsics와 TF가 모두 필요하다 | CameraInfo K→광선→TF→table plane 도식 설명 | screen+narration, GREEN |
| P1 | P06 | 좌표 frame과 timestamp를 확인한다 | `SceneState.header`의 frame/stamp 확대 | screen, GREEN |
| P1 | P07 | 검출률과 위치 정확도는 다르다 | “검출됨” 화면 옆에 관측 오차 24~51mm 카드 | screen, GREEN |
| P1 | P08 | 팔 오차와 camera 오차를 분리했다 | arm target 1~6mm vs camera observation 24~51mm 비교 카드 | screen, GREEN |
| P1 | P09 | 가림은 입력 품질 문제다 | 팔이 없는 정지 상태와 기존 가림 사례 이미지를 비교. 새 동작 불필요 | screen, GREEN |
| P2 | P10 | 잘못된 CameraInfo를 fail-closed 처리한다 | `fx > 0` guard 코드와 설명. 실카메라 설정은 훼손하지 않음 | screen, GREEN |
| P2 | P11 | 고정 셀에 맞춰 단순한 CV를 선택했다 | HSV mask→contour→`minAreaRect` 코드 3단계 | screen+narration, GREEN |
| P2 | P12 | calibration 한계를 공개한다 | `calibration_current.jpg`, `distortion_current.jpg`와 known-limit 자막 | screen, GREEN |

## C8. 독립 촬영 목록 — 기구학·MoveIt·실물 동작 (`M01~M16`)

`M05~M13`은 가급적 **같은 red master 한 번**을 여러 소스에서 잘라 만든다. 각각을 얻기 위해 pick/place를 다시 실행하지 않는다.

| 우선 | ID | 한 클립의 주장 | 촬영 내용 | 방식·위험 |
|---|---|---|---|---|
| P0 | M01 | analytic IK/FK를 직접 구현했다 | branch, joint limit, backfold guard가 보이는 코드 | screen, GREEN |
| P1 | M02 | 수학 결과를 독립 oracle로 검증한다 | IK→FK round-trip과 KDL oracle test 결과 | screen, GREEN |
| P1 | M03 | 도달 불가는 실행 전에 거부한다 | unit test 또는 Gazebo에서 `UNREACHABLE`; 실물 새 좌표 금지 | screen/SIM, GREEN |
| P0 | M04 | 계획 성공과 실행 성공을 분리한다 | skill code의 plan/execute 분기와 result code | screen, GREEN |
| P0 | M05 | camera pose가 실제 pick goal이 된다 | debug overlay→`/pick red_block` terminal→팔 접근 | master phone+screen, AMBER |
| P0 | M06 | 그리퍼가 물체를 실제로 잡는다 | 손가락 닫힘과 블록 lift를 근접 원본에서 6~10초 | master close-up, AMBER |
| P0 | M07 | command 성공이 아니라 feedback으로 파지 판정한다 | gripper joint 값·판정 log와 같은 시점 근접 화면 | master screen+close-up, AMBER |
| P0 | M08 | 물체를 운반 중 유지한다 | lift 후 counter까지의 이동 6~10초 | master wide, AMBER |
| P0 | M09 | place가 release와 retreat를 포함한다 | counter 착지→open→retreat | master wide+close-up, AMBER |
| P0 | M10 | 작업 종료 상태가 명시적이다 | `/place` success result와 counter의 물체 | master phone+screen, AMBER |
| P0 | M11 | 작업 뒤 안전 자세로 복귀한다 | retreat부터 `home` 정지까지 | master wide, AMBER |
| P0 | M12 | 전체 계층이 끊기지 않고 이어진다 | red `pick→place counter→home` 무편집 master | phone+screen, AMBER |
| P1 | M13 | 같은 계약이 화면과 물리 결과에서 일치한다 | Action feedback 타임라인과 실제 팔을 동기 분할 화면 | edit from master, GREEN |
| P1 | M14 | 이름 기반 자세도 동일 Action으로 실행한다 | 검증된 `/move_to home` 한 번과 결과. master 시작/복귀 장면 재사용 우선 | phone+screen, AMBER |
| P2 | M15 | sim과 real이 같은 `/pick` 계약을 쓴다 | Gazebo와 실물의 goal/result 형식을 병렬 표시 | edit, GREEN |
| P2 | M16 | 수치가 조건을 가진다 | “red varied positions 3 consecutive trials”와 측정 조건 카드 | screen, GREEN |

## C9. 독립 촬영 목록 — 안전·실패·복구 (`S01~S15`)

| 우선 | ID | 한 클립의 주장 | 촬영 내용 | 방식·위험 |
|---|---|---|---|---|
| P0 | S01 | 실패를 구체적인 code로 표현한다 | `ErrorCode.msg`의 주요 7개 코드와 recovery mapping | screen, GREEN |
| P0 | S02 | 동시에 두 goal을 실행하지 않는다 | 기존 검증 log에서 두 번째 goal의 terminal rejection | screen, GREEN |
| P0 | S03 | 낡은 scene으로 움직이지 않는다 | stale gate test/log와 “no motion is success” 자막 | screen, GREEN |
| P1 | S04 | `move_to` cancel은 물리 정지까지 포함한다 | 기존 검증 스크립트로 1회만: 감속 정지+`CANCELED=9` | phone+screen, AMBER |
| P1 | S05 | API 지원 범위를 명시한다 | pick/place cancel reject 정책을 code/test로 설명. 실물 실행 금지 | screen, GREEN |
| P1 | S06 | 무한 실행을 watchdog이 막는다 | 기존 실물 log의 timeout·stop 결과. 새 timeout 주입 금지 | screen, GREEN |
| P1 | S07 | 종료 시 safe park를 시도한다 | 이미 검증된 종료 log 또는 안정 세션 마지막 1회 | screen+wide, AMBER |
| P1 | S08 | 통신 장애를 controller 문제와 분리했다 | 기존 serial-disconnect log timeline. 케이블 재분리 금지 | screen, GREEN |
| P1 | S09 | `GRASP_FAILED`와 `GRIPPER_EMPTY`는 다르다 | before-grasp vs in-transport 도식과 코드 | screen, GREEN |
| P0 | S10 | 물체 상실 뒤 제한된 재파지를 한다 | T4의 `GRIPPER_EMPTY→REGRASP` 구간, SIM 상시 표기 | SIM, GREEN |
| P0 | S11 | 복구 예산이 소진되면 포기한다 | T5의 `ABORTED_WAIT`, SIM 상시 표기 | SIM, GREEN |
| P1 | S12 | 실물에서도 자연 발생 복구 흐름을 관측했다 | 2026-08-22 원장 타임라인. “1회 자연 발생” 명시 | screen, GREEN |
| P1 | S13 | 장애 재현과 실제 검증 범위를 구분한다 | REAL VERIFIED / SIM VERIFIED / NOT REPRODUCED 표 | screen, GREEN |
| P2 | S14 | torque 표시를 맹신하지 않는다 | stale `torque_state` 주의와 무부하 동작 점검 절차 | screen+narration, GREEN |
| P2 | S15 | 진단 자체도 시스템에 영향을 줄 수 있다 | 2026-08-23 직접 register read timeout 사례와 개선 조치 | screen+narration, GREEN |

## C10. 독립 촬영 목록 — agent·LLM·자가정리 (`A01~A13`)

| 우선 | ID | 한 클립의 주장 | 촬영 내용 | 방식·위험 |
|---|---|---|---|---|
| P0 | A01 | LLM은 고수준 계획만 제안한다 | 한국어 문장→JSON `pick/place`; motor angle이 없음을 확대 | screen, GREEN |
| P0 | A02 | 계획은 schema와 whitelist를 통과해야 한다 | validator의 허용 skill·target과 통과 결과 | screen, GREEN |
| P1 | A03 | scene에 없는 물체를 거부한다 | test 또는 agent dry path에서 unknown object rejection | screen, GREEN |
| P1 | A04 | 허용되지 않은 skill을 거부한다 | 악성/오류 JSON fixture와 validation result | screen, GREEN |
| P1 | A05 | 모델 실패에 결정론적 fallback이 있다 | 같은 명령의 retry/fallback 경로를 코드와 test로 설명 | screen, GREEN |
| P0 | A06 | 다국어 능력을 시험지로 측정했다 | 10개 언어 60개 중 59개 성공 표와 조건 | screen, GREEN |
| P0 | A07 | 모델 평가는 로봇 성공률과 별개다 | 220/242 plan 정확도 옆에 “physical success rate 아님” 표시 | screen, GREEN |
| P1 | A08 | agent가 상태를 외부에 공개한다 | `/robot_status`의 `IDLE→RUNNING→IDLE` 기록 | screen, master 재사용 |
| P1 | A09 | 자가정리는 코드에 명시된 셀 정책이다 | `green_block→bin`, `red/blue→shelf` mapping 코드 | screen, GREEN |
| P1 | A10 | 방치 사건이 작업을 만든다 | T3의 2초 loiter→자동 goal, SIM 상시 표기 | SIM, GREEN |
| P2 | A11 | 실물 자가정리는 조건부 증거다 | master 백업 뒤 green 실물 리허설이 통과한 경우에만 1회 | phone+screen, AMBER |
| P1 | A12 | 복구는 원인별로 다르다 | error→RETRY/RESCAN/REGRASP/ABORT mapping | screen, GREEN |
| P2 | A13 | 사람 개입을 요청할 줄 안다 | `ABORTED_WAIT` 뒤 새 작업을 받지 않는 상태 | SIM+screen, GREEN |

## C11. 독립 촬영 목록 — 코드·테스트·운영·본인 설명 (`Q01~Q16`)

| 우선 | ID | 한 클립의 주장 | 촬영 내용 | 방식·위험 |
|---|---|---|---|---|
| P0 | Q01 | 패키지 경계를 설계했다 | `arm_interfaces/kinematics/skills/perception/agent/bringup` tree | screen, GREEN |
| P0 | Q02 | C++와 Python이 interface로 연결된다 | C++ skill과 Python agent가 같은 Action을 사용하는 두 코드 화면 | screen, GREEN |
| P1 | Q03 | 설정 사본의 drift를 test로 막는다 | cell layout consistency test와 YAML | screen, GREEN |
| P1 | Q04 | 수학·계약·AI를 다른 test 층으로 나눴다 | GTest, pytest, eval script를 3단 분할 | screen, GREEN |
| P1 | Q05 | 실행 환경을 재현한다 | Docker compose/service, ROS distro, device mapping 설명 | screen, GREEN |
| P1 | Q06 | 현상을 데이터로 다시 볼 수 있다 | rosbag `info`와 기록 topic 목록 | screen, GREEN |
| P1 | Q07 | 장애를 시간순으로 좁힌다 | 증상→가설→측정→수정→회귀 원장 한 사례 | screen+narration, GREEN |
| P1 | Q08 | 성공 조건과 분모를 숨기지 않는다 | 3 consecutive trials는 보여주되 성공률로 바꾸지 않는 카드 | screen, GREEN |
| P1 | Q09 | 직접 만든 것과 연동한 것을 구분한다 | custom / integrated framework / unused 3열 카드 | screen+narration, GREEN |
| P1 | Q10 | known limitation을 제품 범위로 설명한다 | fixed camera/table/known object/supervised 카드 | screen+narration, GREEN |
| P2 | Q11 | 버전과 take를 추적한다 | commit hash, diff 여부, 날짜, launch args가 있는 take sheet | screen, GREEN |
| P2 | Q12 | 문서가 재현 절차 역할을 한다 | `handoff` milestone에서 명령·결과·판정이 이어지는 화면 | screen, GREEN |
| P0 | Q13 | 본인이 전체 architecture를 설명할 수 있다 | 45초 화이트보드 설명: camera→Action→controller→motor | face+diagram, GREEN |
| P0 | Q14 | 설계 선택의 이유를 설명할 수 있다 | “왜 LLM은 motor를 안 만지는가?” 30초 답변 | face, GREEN |
| P0 | Q15 | 가장 어려운 실물 문제를 설명할 수 있다 | serial/controller/gripper 문제 하나를 STAR 방식 45초 | face, GREEN |
| P1 | Q16 | 다음 단계도 현실적으로 안다 | CI, 3D perception, industrial HW, repeatability 중 우선순위 30초 | face, GREEN |

전체 목록은 100% 촬영 의무가 아니라 **지원처별 조합이 가능한 원본 라이브러리**다. 먼저 P0를 확보하고, 팔이 안정된 세션에서는 P1의 `GREEN` 항목을 많이 찍는다. P2는 시간이 남을 때 추가한다.

## C12. 실물 master 한 번으로 여러 클립을 만드는 방법

### 동시 녹화 배치

1. 휴대폰 A: 셀 전체 광각, 고정
2. 휴대폰 B가 있으면: 그리퍼와 물체 근접, 작업 반경 밖에서 고정
3. PC: 왼쪽 debug image, 오른쪽 skill terminal, 하단 시각 표시
4. 녹화 시작 뒤 손뼉 한 번과 `REAL_MASTER_01` 음성 slate
5. red block의 정지 화면 5초
6. direct `pick → place counter → home` 한 번
7. 결과 화면과 최종 물체를 5초 더 기록

### 한 master에서 파생할 파일

```text
M05_camera_to_pick.mp4
M06_gripper_lift_closeup.mp4
M07_grasp_feedback_split.mp4
M08_transport_wide.mp4
M09_release_retreat.mp4
M10_action_success.mp4
M11_home_return.mp4
M12_real_master_unedited.mp4
M13_action_physical_sync.mp4
recruiter_15s_teaser.mp4
```

이 방식이면 영상은 많이 남지만 서보와 그리퍼에 가하는 실제 작업 횟수는 한 번이다. 다른 각도가 필요하다는 이유만으로 동작을 재실행하지 않는다.

## C13. 화면 증거용 안전 명령 모음

아래 명령은 상태를 읽는 용도다. `topic hz`와 `tf2_echo`는 필요한 5~10초만 촬영하고 `Ctrl+C`로 끝낸다.

```bash
ros2 node list | sort
ros2 action list | sort
ros2 control list_controllers
ros2 topic echo /scene_state --once
ros2 topic echo /camera/camera_info --once
ros2 topic echo /dynamixel_hardware_interface/dxl_state --once
ros2 topic hz /joint_states --window 50
ros2 topic hz /scene_state --window 20
ros2 run tf2_ros tf2_echo world camera_optical_frame
ros2 topic echo /robot_status
ros2 bag info /ws/media/rosbags/<bag-directory>
```

주의:

- `dxl_state`의 `torque_state`만으로 실제 torque on/off를 판정하지 않는다.
- controller가 active인 촬영 세션에서 `get_dxl_data` 직접 레지스터 query를 하지 않는다. 2026-08-23 실측에서 service timeout과 반복 async-trigger 경고를 유발했다.
- 토크·그리퍼 확인은 오류 없는 comm/hardware state와 작업 반경을 비운 무부하 gripper 동작을 함께 본다.
- graph CLI가 오래 멈추면 goal을 보내지 말고 기존 process와 ROS domain부터 확인한다.

## C14. 공고별 추천 클립 조합

| 지원 방향 | 첫 30초에 쓸 클립 | 뒤에 붙일 증거 | 빼거나 뒤로 보낼 것 |
|---|---|---|---|
| 로보티즈 시스템 SW | H02, H05, B02~B04, B11, M12 | S02~S08, Q07, Q12 | LLM benchmark를 첫 장면에 두지 않음 |
| ROS/카메라 application | B05~B08, P02~P08, M05, M12 | Q02, Q03, Q10 | 범용 vision처럼 표현하지 않음 |
| Deployment/현장 통합 | H05~H07, B01~B15, Q06~Q12 | S08, S13, 실제 원인분석 설명 Q15 | 화려한 simulation만 길게 쓰지 않음 |
| Manipulator/MoveIt | M01~M16, S01~S05 | P03, B09~B11, IK 수치 | MoveIt 사용을 planner 자체 개발로 표현하지 않음 |
| Robotics platform | Q01~Q07, B07~B16, S01~S08 | Docker, logging, docs, sim-real | 대규모·real-time 경험으로 과장하지 않음 |
| Physical AI/agent | A01~A13 뒤에 M12 | schema, fallback, bounded recovery | LLM이 motor를 직접 제어한다고 표현하지 않음 |

## C15. 편집 레시피

### 15초 teaser

```text
0~3초   H01 실물 셀 + 제목
3~8초   P02 debug overlay → M06 실제 lift
8~12초  M09 place → M10 success
12~15초 Q09 custom/integrated 카드 + 저장소 링크
```

### 60~90초 채용 담당자용

```text
0~6초    문제와 셀 전경
6~14초   B05~B11 계층이 하나씩 뜨는 빠른 montage
14~45초  M12 실물 master 핵심 구간
45~58초  S02~S04 안전 contract
58~70초  A01/A06 또는 T3 simulation 차별점
70~82초  실측 수치·known limits
82~90초  맡고 싶은 직무와 repository 링크
```

### 2~4분 기술 설명용

```text
문제/범위 20초 → architecture 25초 → 순차 bring-up 35초
→ perception/TF 30초 → IK/MoveIt/Action 40초 → 실물 master 35초
→ failure/safety 30초 → simulation recovery 20초 → 수치·한계 15초
```

### 5~8분 deep dive

대표 영상의 장면을 늘리는 방식이 아니라 Q01~Q16을 사용해 interface 설계, code ownership, test, 실물 장애 한 사례, sim-real 경계, 다음 개선 순서까지 본인이 설명한다.

## C16. 파일명과 take 원장

폴더 예시:

```text
shoot_2026-08-23/
  00_take_sheet/
  01_phone_wide/
  02_phone_close/
  03_screen/
  04_sim/
  05_interview/
  06_exports/
```

파일명 형식:

```text
<ID>_<REAL|SCREEN|SIM>_<claim>_takeNN_<KEEP|FAIL|REFERENCE>.<ext>
```

예시:

```text
B03_SCREEN_joint_state_100hz_take01_KEEP.webm
M12_REAL_red_pick_place_master_take01_KEEP.mp4
S08_SCREEN_serial_fault_timeline_REFERENCE.webm
Q15_SCREEN_field_debug_story_take02_KEEP.mp4
```

각 take마다 다음을 적는다.

| 필드 | 기록 내용 |
|---|---|
| ID·take·시각 | `M12 / take01 / 14:32:10 KST` |
| 코드 기준선 | commit hash, dirty 여부와 관련 diff |
| 장치 | 실제 tty, video device, 전원 상태 |
| 실행 주체 | 현재 켠 launch 01~07 목록 |
| 인자 | stage, port, video, trim, agent 여부 |
| 입력 조건 | 물체, 시작 좌표/구역, 조명, camera 이동 여부 |
| 결과 | Action terminal state, error code, 소요 시간 |
| 물리 관찰 | lift, slip, 착지, 이상 소리·진동 |
| 파일 | phone wide, close, screen의 정확한 이름 |
| 사용 판정 | KEEP / 실패 분석용 KEEP / RETAKE / 폐기 사유 |

## C17. 촬영하지 않고 기존 증거를 쓰는 항목

다음은 영상 개수를 늘리기 위해 실물에서 다시 만들지 않는다.

1. 동작 중 USB/serial cable 분리
2. 집은 물체를 사람이 빼앗는 fault injection
3. 실행 예산을 인위적으로 줄인 watchdog timeout
4. pick/place goal cancel
5. 두 개의 실제 client로 동시 goal 경쟁
6. blue ring과 새 하강 방식
7. work·bin·shelf·counter 가장자리 또는 새 좌표
8. raw joint trajectory나 controller topic 직접 발행
9. controller active 중 DYNAMIXEL register 직접 조회
10. 10개 언어를 각각 실물 동작으로 반복

이 항목은 기존 검증 log, rosbag, unit/integration test, Gazebo 영상, 도식으로 제시한다. “실물에서 재현 장면을 못 찍었다”와 “기능을 검증하지 않았다”는 같은 뜻이 아니며, 매체와 검증 범위를 명시하면 된다.

## C18. 촬영 완료 우선순위

### P0 — 지원 전에 반드시 확보

- [ ] H01, H02, H05: 실물 셀·센서·장치
- [ ] B02, B03, B05, B08, B09, B11: 실행 주체를 하나씩 올린 증거
- [ ] P02, P03, P04: 검출·world pose·구역
- [ ] M01, M04: 직접 구현과 plan/execute 경계
- [ ] M05~M13: 실물 master에서 파생한 동작·결과
- [ ] S01~S03, S10, S11: 실패 contract와 안전한 recovery 증거
- [ ] A01, A02, A06, A07: LLM 역할과 평가 범위
- [ ] Q01, Q02, Q13~Q15: architecture와 본인 설명

### P1 — 직무 맞춤 포트폴리오를 강하게 만드는 것

- [ ] 순차 bring-up 전 과정 B01~B15
- [ ] perception 오차와 가정 P05~P12
- [ ] cancel 1회 또는 기존 실물 증거 S04
- [ ] 실물 장애 원장과 sim-real 경계 S06~S13
- [ ] 운영·test·문서 Q03~Q12

### P2 — 시간이 남을 때

- [ ] 추가 hardware B-roll
- [ ] 조건을 통과한 green 실물 자가정리 1회
- [ ] 면접 답변의 다른 길이 버전
- [ ] 회사별 30초 재편집본

종료 기준은 “모든 ID를 찍음”이 아니다. **P0가 읽히고, real/sim 범위가 표시되고, 무편집 master와 take 원장이 보존되었으며, 추가 실물 반복 없이도 목표 공고별 60~90초 편집본을 만들 수 있는 상태**면 촬영을 끝낸다.
