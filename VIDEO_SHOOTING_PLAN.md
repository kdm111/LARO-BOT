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
- 초록 블록과 형상은 같지만, 초록의 목적지인 `bin`이 셀 가장자리에 있다.

### counter

- 현재 작업 공간에서 비교적 안정적인 `-y` 방향 목적지다.
- `bin`은 카메라 시야와 팔 도달 범위의 가장자리에 가깝다.
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

대표 실물 촬영은 에이전트를 끈 direct Action 모드로 시작한다.

```bash
sudo ./dc730 exec sim bash -lc '
  source /opt/ros/jazzy/setup.bash
  source /ws/install/setup.bash
  export ROS_DOMAIN_ID=48
  ros2 launch arm_bringup real.launch.py stage:=hold agent:=false
'
```

launch 직후 바로 움직이지 말고 다음을 확인한다.

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
- [ ] ID 16을 포함한 팔·그리퍼 torque state가 기대 상태다.
- [ ] 그리퍼가 `0.5` 시험 명령에서 실제로 움직였던 현재 세션 상태를 유지한다.
- [ ] 실행 중인 client는 하나뿐이다.

### 즉시 중단 조건

- 그리퍼만 힘이 없거나 torque state가 false
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
| 그리퍼가 안 움직임 | ID 16 torque state, 전원, joint state |
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

자가정리는 “로봇이 스스로 작업을 만든다”는 프로젝트의 가장 독특한 시나리오다. 그러나 초록 블록의 목적지 `bin`은 현재 셀의 카메라·도달 한계에 가깝다.

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
