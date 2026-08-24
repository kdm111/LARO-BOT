# 프로젝트 전체 소개와 토론 문서

> 프로젝트명: **ROS 2 기반 실물 매니퓰레이터 Application/System Integration**  
> 쉬운 표현: **다국어 지시를 받아 물건을 옮기고, 방치된 물건을 스스로 정리하는 AI 로봇 알바 프로토타입**  
> 대상 독자: 로봇, ROS 2, 인공지능을 처음 접하는 사람  
> 문서 목적: 이 프로젝트가 무엇인지 소개하고, 각 기능과 기술을 만들며 사용자가 무엇을 얻었는지 함께 토론하기 위한 공통 지도

## 1. 이 문서의 역할

이 저장소에는 목적이 서로 다른 문서가 있다.

- `README.md`: 사용자가 개발 과정에서 이해한 내용을 직접 쌓은 학습 기록
- `handoff/`: 다음 AI가 현재 상태, 실측 결과, 남은 위험을 정확히 이어받기 위한 작업 문서
- **이 문서**: 처음 보는 사람에게 프로젝트 전체를 설명하고, 사용자와 설계·성과·취업 방향을 토론하기 위한 소개서

따라서 이 문서는 코드를 전혀 모르는 사람도 읽을 수 있게 설명하되, 검증한 것과 아직 검증하지 못한 것을 숨기지 않는다.

---

## 2. 30초 소개

이 프로젝트는 탁자 위의 로봇 팔이 카메라로 물체를 보고, 사람의 지시를 제한된 작업 계획으로 바꾼 뒤, 물체를 집어서 정해진 장소에 놓는 시스템이다.

예를 들어 사람이 “빨간 블록을 카운터에 놓아줘”라고 말하면 시스템은 다음 순서로 동작한다.

1. 카메라가 빨간 블록의 위치와 방향을 찾는다.
2. 언어 모델은 문장을 `빨간 블록 집기 → 카운터에 놓기`라는 두 단계로 바꾼다.
3. 검증기가 로봇이 실제로 할 수 있는 명령인지 확인한다.
4. 로봇 팔이 해당 위치에 닿을 수 있는 관절 각도를 계산한다.
5. 충돌을 피하는 이동 경로를 계획한다.
6. 실물 모터와 그리퍼가 경로를 실행한다.
7. 결과를 성공 또는 구체적인 실패 코드로 보고한다.

또한 작업 구역에 물체가 일정 시간 방치되면 사람이 명령하지 않아도 로봇이 정리 작업을 만든다. 초록 블록은 이 셀에서 “불량품”으로 정해져 있어 수거함으로 보낸다.

이 프로젝트의 핵심은 로봇 팔 한 동작이 아니라 다음 전체 연결이다.

> **카메라 → 인지 → 작업 판단 → 기구학 → 경로 계획 → ROS 2 Action → 모터 실행 → 결과 검증과 복구**

---

## 3. 사람에 비유한 시스템 구성

| 사람의 역할 | 프로젝트 구성 | 하는 일 |
|---|---|---|
| 눈 | RGB 카메라 | 탁자 위 영상을 30Hz로 보낸다 |
| 시각 인지 | OpenCV 물체 검출기 | 색과 형상을 이용해 물체 ID, 위치, 방향을 찾는다 |
| 상황판 | `/scene_state` | 현재 어떤 물체가 어디에 있는지 공유한다 |
| 작업 관리자 | Python agent | 사람 명령과 자가정리 정책을 실행 순서로 만든다 |
| 통역사 | Ollama 언어 모델 | 여러 언어의 문장을 제한된 로봇 스킬로 번역한다 |
| 작업 규정 | 계약·화이트리스트·검증기 | 허용된 스킬과 물체, 장소만 통과시킨다 |
| 팔 자세 계산 | 해석 IK | 목표 위치를 만들 관절 각도를 계산한다 |
| 이동 경로 담당 | MoveIt 2 | 목표까지 갈 수 있는 경로를 계획하고 실행한다 |
| 신경계 | ROS 2와 ros2_control | 노드, 메시지, Action, 컨트롤러를 연결한다 |
| 근육과 감각 | DYNAMIXEL 모터 | 팔과 그리퍼를 움직이고 현재 관절 상태를 돌려준다 |
| 현장 관리자 | 상태 머신과 안전 gate | 실패를 분류하고, 재시도하거나 멈추고 사람을 부른다 |

중요한 경계가 하나 있다. 언어 모델은 모터 각도나 궤적을 직접 만들지 않는다. 언어 모델은 `pick`, `place`, `move_to` 중 무엇을 어떤 순서로 실행할지만 제안한다. 실제 동작 가능 여부와 실행은 결정론적인 로봇 소프트웨어가 담당한다.

---

## 4. 전체 프로세스

```text
사람의 다국어 명령 ───────────────┐
                                   ▼
작업 구역 방치 감지 ──> 자가정리 정책 ──> 작업 계획
                                   │
                         계약·씬·화이트리스트 검증
                                   │
                 ┌─────────────────┴─────────────────┐
                 ▼                                   ▼
            /pick Action                         /place Action
                 │                                   │
카메라 영상 → 물체 검출 → /scene_state               │
                 │                                   │
                 └──> 해석 IK → MoveIt 2 → ros2_control
                                             │
                                      DYNAMIXEL 팔·그리퍼
                                             │
                          성공 / 실패 코드 / 취소 / 시간 초과
                                             │
                          재시도·재인지·중단·사람 확인 요청
```

### 4.1 사람이 일을 시키는 경우

예시 명령은 “빨간 블록을 카운터에 놓아줘”다.

- 언어 모델이 `pick(red_block)`과 `place(red_block, counter)`로 번역한다.
- 검증기는 물체가 현재 씬에 있는지, 스킬과 장소가 계약에 있는지 확인한다.
- `pick`은 최신 카메라 프레임에서 물체 위치와 yaw를 읽는다.
- 팔은 접근, 하강, 그리퍼 닫기, 파지 판정, 들어 올리기를 순서대로 실행한다.
- `place`는 운반 중 물체를 계속 쥐고 있는지 확인하고, 목표 구역에서 손을 연 뒤 물러난다.
- 작업이 끝나면 팔은 `home`으로 돌아간다.

### 4.2 로봇이 스스로 일을 만드는 경우

에이전트는 작업 구역에 물체가 2초 이상 머무는지 본다.

- 빨간 블록: 창고(`shelf`)로 정리
- 파란 링: 창고(`shelf`)로 정리하도록 정의되어 있으나 실물 반복성은 제한적
- 초록 블록: 불량품으로 정의하고 수거함(`bin`)으로 이동

이 자가정리는 언어 모델이 임의로 목적을 발명하는 구조가 아니다. 물체별 목적지는 코드에 명시된 셀 운영 정책이며, 에이전트가 그 정책에 따라 `pick → place` 작업을 생성한다.

### 4.3 문제가 생기는 경우

시스템은 모든 실패를 같은 “실패”로 뭉개지 않는다.

- 물체가 보이지 않음: `OBJECT_NOT_FOUND`
- 팔이 물리적으로 닿지 못함: `UNREACHABLE`
- 경로를 만들지 못함: `PLANNING_FAILED`
- 물체를 처음부터 잡지 못함: `GRASP_FAILED`
- 잡은 뒤 운반 중 떨어뜨림: `GRIPPER_EMPTY`
- 실행 시간이 예산을 넘음: `EXECUTION_TIMEOUT`
- 사용자가 이동을 취소함: `CANCELED`

이 구분 덕분에 “다시 계획”, “다시 보기”, “다시 집기”, “즉시 중단”처럼 원인에 맞는 대응을 고를 수 있다. 실물 고장 주입을 포함한 전체 복구 시나리오는 아직 완성 범위가 아니며, 복구 상태 머신의 전체 흐름은 Gazebo에서 검증했다.

---

## 5. 개발 단계와 사용자가 얻은 것

### M1. 팔의 위치를 관절 각도로 바꾸기

첫 단계는 목표 좌표 `(x, y, z)`를 받았을 때 각 관절을 몇 도 움직여야 하는지 계산하는 것이었다. 5축 팔의 해석 역기구학(IK)을 구현하고, 팔꿈치 방향이 다른 두 해, 관절 한계, 도달 불가를 구분했다. 계산 결과는 순기구학(FK)으로 다시 좌표를 복원해 검증했다.

**사용자가 얻은 것**

- 공식을 코드로 옮기는 데서 끝내지 않고 역계산으로 검증하는 방법
- 하나의 목표에 여러 관절 해가 존재한다는 로봇 기구학의 핵심 개념
- “계산 가능”과 “실물 관절 한계 안에서 실행 가능”이 다르다는 이해
- 단위 테스트를 수학 구현의 안전망으로 사용하는 경험

### M2. 계산을 실제 로봇 스킬로 만들기

다음으로 `pick`, `place`, `move_to`를 ROS 2 Action으로 만들었다. 스킬 서버는 해석 IK와 MoveIt 2를 연결하고, 각 단계의 성공과 실패를 구조화된 결과로 돌려준다.

**사용자가 얻은 것**

- ROS 2 node, topic, message, action의 역할 차이
- 오래 걸리는 로봇 동작에 Action이 적합한 이유
- 인터페이스를 먼저 고정하면 mock, simulation, real robot을 같은 호출 방식으로 바꿀 수 있다는 설계 경험
- 계획 성공과 실제 실행 성공을 분리해 보고해야 한다는 시스템 관점

### M3. 카메라가 본 물체를 팔이 집게 만들기

RGB 이미지에서 색 영역을 찾고, 물체의 중심 픽셀과 긴 축을 구했다. 카메라 내부 파라미터와 TF를 이용해 픽셀을 테이블 평면의 로봇 좌표로 바꿔 `/scene_state`로 발행했다.

**사용자가 얻은 것**

- OpenCV의 HSV 마스크, contour, 중심점, `minAreaRect` 사용 경험
- 카메라 픽셀 좌표와 로봇 월드 좌표가 서로 다른 좌표계라는 이해
- CameraInfo, TF, optical frame을 연결하는 ROS 2 인지 파이프라인 경험
- 검출이 된다는 사실과 위치가 정확하다는 사실은 다르며, 오차를 직접 재야 한다는 교훈
- 딥러닝이 없어도 고정 환경에서는 단순한 방법이 더 설명 가능하고 빠를 수 있다는 판단 경험

### M4. 다국어 명령을 안전한 작업 계획으로 제한하기

Ollama 모델이 자연어를 JSON 스킬 계획으로 만들게 했다. 출력은 스키마, 스텝 수, 스킬 이름, 필수 인자, 물체 존재 여부, 장소 화이트리스트를 통과해야 실행된다. 모델 실패 시 재질문과 결정론적 문자열 파서가 폴백으로 남는다.

**사용자가 얻은 것**

- LLM 출력을 곧바로 실행하지 않고 계약과 검증기 뒤에 두는 방법
- 구조화 출력, JSON schema, 화이트리스트, fallback 설계 경험
- 모델 정확도뿐 아니라 거부 능력과 지연을 함께 측정해야 한다는 이해
- “AI가 로봇을 제어한다”보다 “AI는 제한된 고수준 계획만 제안한다”가 더 안전하고 설명 가능하다는 판단
- 한국어·영어 및 10개 언어 시험지를 직접 설계하고 모델을 비교한 평가 경험

### M5. 실패를 감추지 않고 복구하거나 포기하기

로봇이 물체를 놓치거나 물체 위치가 바뀌었을 때 재시도, 재인지, 재파지를 수행하는 상태 머신을 만들었다. 같은 실패를 무한 반복하지 않도록 스텝별 복구 예산과 자가정리 시도 예산을 두고, 소진하면 `ABORTED_WAIT`에서 사람을 기다리게 했다.

**사용자가 얻은 것**

- 정상 동작보다 실패 상태와 전이가 시스템 설계에서 더 중요할 수 있다는 이해
- 무한 재시도를 막는 bounded recovery와 retry budget 개념
- 실패 코드와 복구 전략을 분리해 설계하는 방법
- 로봇이 “포기할 줄 아는 것”도 자율성의 일부라는 운영 관점
- 재현하기 어려운 실패를 Gazebo fault injection으로 반복 검증하는 경험

### M6. 시뮬레이션을 실물 장비로 옮기기

마지막 단계에서 RGB 카메라, OpenRB/DYNAMIXEL, MoveIt 2, ros2_control, 팔과 그리퍼를 실물에서 연결했다. 시리얼 단절, 컨트롤러 상태, 그리퍼 토크 상실, 잘못된 목표, 카메라 보정 표류를 계층별로 진단했다.

**사용자가 얻은 것**

- 실물 bring-up은 코드를 실행하는 일이 아니라 전원, USB, 권한, 통신, controller, TF, launch 순서를 함께 다루는 일이라는 경험
- 로그, 관절값, 서보 레지스터, 반복 측정을 이용해 문제 계층을 좁히는 디버깅 능력
- 시뮬레이션에서 통과한 상수와 가정이 실물에서는 깨질 수 있다는 sim-to-real 이해
- 잘못된 목표를 거부하고, 낡은 인지로 움직이지 않고, 동시 명령을 막는 fail-closed 설계 경험
- 기능을 계속 늘리는 대신 검증 범위를 좁히고 known limitation을 공개하는 프로젝트 마감 판단

---

## 6. 기능·기술별 정리

| 기능 또는 기술 | 이 프로젝트에서 한 일 | 사용자에게 남은 능력과 경험 |
|---|---|---|
| ROS 2 Jazzy | 여러 패키지와 node를 topic, service, action으로 연결 | 분산 로봇 애플리케이션의 경계와 통신 방식을 설계하는 능력 |
| 사용자 정의 인터페이스 | `Pick`, `Place`, `MoveTo`, `SceneState`, `FailureReport`, `RobotStatus` 정의 | 구현보다 먼저 계약을 고정하고 컴포넌트를 독립시키는 경험 |
| C++ Action skill server | 실시간 실행에 가까운 팔 동작과 MoveIt 연동 | 비동기 goal, result, cancel, single-flight를 다루는 경험 |
| Python agent | 명령 해석, 상태 관리, 자가정리, 복구 orchestration | 여러 Action을 하나의 작업으로 묶는 응용 계층 설계 경험 |
| 해석 IK/FK | 5축 팔의 관절 해 계산과 역검증 | 수학, 관절 제한, 도달성 판단을 실제 코드와 테스트로 연결한 경험 |
| MoveIt 2 | IK 결과를 목표로 받아 plan과 execute 수행 | “목표가 가능함”, “경로가 있음”, “실제로 실행됨”을 구분하는 능력 |
| ros2_control | MoveIt과 실제 DYNAMIXEL controller 연결 | 상위 계획과 하위 하드웨어 제어 계층의 경계를 이해한 경험 |
| DYNAMIXEL/OpenRB | 관절 상태, 토크, 시리얼 통신, 그리퍼 제어 | 하드웨어 통신 장애와 소프트웨어 오류를 구분하는 bring-up 경험 |
| RGB perception | HSV 검출, 형상 필터, 중심·yaw 계산 | 고정 환경용 인지 파이프라인을 직접 구성하고 튜닝한 경험 |
| CameraInfo와 TF | 픽셀 광선을 로봇 기준 테이블 평면으로 변환 | 좌표계, 프레임, timestamp가 로봇 통합에서 중요한 이유를 체득 |
| 셀 구역 모델 | `work`, `shelf`, `counter`, `bin`을 실제 좌표로 정의 | 물리 도달 범위와 카메라 시야를 애플리케이션 규칙으로 바꾸는 경험 |
| 그리퍼 파지 판정 | 명령 성공이 아니라 실제 관절 위치로 빈 손·파지·낙하 구분 | actuator feedback을 이용해 거짓 성공을 막는 경험 |
| 다국어 LLM planning | 자연어를 최대 2단계 스킬 계획으로 변환 | 생성형 AI를 로봇에 제한적으로 접목하고 평가하는 능력 |
| 계약 검증과 폴백 | 스키마, 화이트리스트, 씬 대조, 재질문, 문자열 폴백 | 확률적인 모델과 결정론적인 실행 시스템 사이 경계를 만드는 경험 |
| 자가정리 | 방치된 물체를 감지해 물체별 목적지로 작업 생성 | 이벤트 기반 자율 작업과 사람 명령을 같은 실행 계약으로 통합한 경험 |
| 복구 상태 머신 | RETRY, RESCAN, REGRASP, ABORT 및 예산 관리 | 실패 원인에 따라 제한된 복구를 설계하는 경험 |
| stale scene gate | goal 이후 들어온 새 프레임만 사용하고 없으면 무동작 실패 | 낡은 센서 정보보다 멈춤을 선택하는 fail-closed 사고방식 |
| single-flight | 하나의 팔에 동시에 두 goal이 실행되지 않도록 원자적으로 거부 | 공유 하드웨어의 동시성 문제를 실제 위험과 연결한 경험 |
| cancel과 watchdog | `move_to` 취소 시 정지, 시간 초과 시 정지와 오류 보고 | API가 “취소됨”이라고 말할 때 물리적으로도 멈춰야 한다는 계약 이해 |
| 안전 종료 | SIGINT/SIGTERM에서 정지 후 home 복귀, 토크 유지 | 중력으로 무너지는 팔에서 종료와 전원 차단이 다른 문제임을 이해 |
| Gazebo simulation | 정상 작업, 자가정리, 낙하, 재인지, 예산 소진 반복 | 위험하거나 반복 비용이 큰 실패를 simulation에서 검증하는 경험 |
| Docker 환경 | ROS 2, Gazebo, 장치 권한과 실행 환경을 컨테이너로 관리 | 개발 환경 재현과 Linux device/cgroup 문제 해결 경험 |
| 단위·통합 테스트 | IK/FK, 계약, 셀 좌표 사본, LLM planner 시험 | 수학·설정·AI 출력을 서로 다른 테스트 층으로 나누는 경험 |
| 모델 평가 | 정확도, 거부, 중앙 지연, 언어별 결과를 CSV로 측정 | 모델의 인상 대신 시험지와 수치로 선택하는 경험 |
| 실측 문서화 | 증상→가설→측정→수정→회귀 기록 | 팀원이 없어도 다음 사람이 판단을 재현할 수 있게 근거를 남기는 능력 |

---

## 7. 대표 설계 선택과 그 대가

### 7.1 딥러닝 인식 대신 색 기반 인식

- 선택 이유: 고정 카메라, 고정 조명, 정해진 색의 물체라는 프로젝트 범위에서는 빠르고 설명 가능하다.
- 얻은 것: 적은 계산량, 디버깅 가능한 검출 과정, RunPod와 무관한 실시간 인지
- 대가: 임의 물체, 같은 색의 여러 물체, 조명 변화에는 약하다.
- 사용자가 얻은 판단: 가장 복잡한 기술이 아니라 현재 문제와 검증 비용에 맞는 기술을 선택하는 법

### 7.2 단안 RGB와 테이블 평면 가정

- 선택 이유: 깊이 카메라 없이도 탁자 위 물체의 위치를 계산할 수 있다.
- 얻은 것: 간단한 하드웨어로 camera-to-robot 연결 완성
- 대가: 높이와 3D 자세를 일반적으로 복원하지 못하고 렌즈 왜곡과 카메라 이동에 민감하다.
- 사용자가 얻은 판단: 가정을 명시하면 단순한 방법도 제품 범위 안에서는 유효하지만, 가정 밖으로 일반화해 주장하면 안 된다는 것

### 7.3 해석 IK와 MoveIt 2를 함께 사용

- 해석 IK: 목표의 도달 가능성과 관절 해를 직접 계산한다.
- MoveIt 2: 현재 상태에서 목표까지 갈 궤적을 계획하고 실행한다.
- 얻은 것: 수학적으로 설명 가능한 목표 생성과 표준 실행 스택을 함께 사용
- 대가: 로봇 모델, 실제 관절 한계, controller 상태가 모두 일치해야 한다.
- 사용자가 얻은 판단: 직접 구현과 프레임워크 사용은 경쟁 관계가 아니라 계층별 역할 분담이 될 수 있다는 것

### 7.4 LLM을 고수준 계획에만 사용

- 선택 이유: 자연어의 다양성은 LLM이 잘 처리하지만 모터 제어는 결정론적이어야 한다.
- 얻은 것: 다국어 명령과 유연한 표현을 제한된 스킬 계약에 연결
- 대가: 모델이 올바른 JSON을 내도 의미가 틀릴 수 있어 별도의 의미 검증이 필요하다.
- 사용자가 얻은 판단: AI 모델의 능력과 전체 시스템의 안전성을 같은 것으로 보지 않는 법

### 7.5 넓은 작업 공간 대신 안전한 고정 셀

- 선택 이유: 팔은 가운데에서는 안정적이지만 양끝에서는 높이와 관절 여유가 빠르게 줄어든다.
- 얻은 것: 반복 가능한 작업 구역과 명시적인 place 목적지
- 대가: 범용 작업 공간이나 6개 구역 같은 확장은 포기했다.
- 사용자가 얻은 판단: 하드웨어의 실제 작업 가능 영역을 측정해 애플리케이션 범위로 바꾸는 법

### 7.6 자동 torque-off 대신 home 복귀 후 토크 유지

- 선택 이유: 이 팔은 토크를 끄면 중력으로 무너진다.
- 얻은 것: 종료 시 팔을 home으로 옮기고 자세를 유지
- 대가: 최종 전원 차단은 사람이 물리 스위치로 담당한다.
- 사용자가 얻은 판단: “전원을 끄면 안전하다” 같은 일반론보다 실제 기구의 실패 형태를 먼저 봐야 한다는 것

---

## 8. 지금까지 실제로 확인한 결과

### 실물에서 확인

- 팔 `/joint_states`: 약 100Hz
- 카메라와 `/scene_state`: 약 30Hz
- RGB 카메라, 팔, 그리퍼, MoveIt, controller 통합 launch
- 카메라가 측정한 서로 다른 위치의 빨간 블록 pick 3회 연속: 12.1초, 13.1초, 13.0초
- `counter`, `bin`, `shelf` 세 목적지에서 place 단계 실행
- 별도 6점에서 팔 목표 위치 오차 1~6mm
- 카메라 기반 place 관측 오차 24~51mm
- plan 성공과 execute 실패를 분리해 시리얼 단절을 거짓 성공으로 보고하지 않음
- `move_to` cancel 시 중간 정지와 CANCELED 보고
- 실행 중 두 번째 goal의 single-flight 거부
- watchdog 발동 시 정지와 `EXECUTION_TIMEOUT` 보고
- 인지 중단 시 낡은 좌표로 움직이지 않고 `OBJECT_NOT_FOUND` 보고
- 종료 신호에서 home 복귀 후 토크 유지

팔 오차 1~6mm와 카메라 오차 24~51mm는 서로 다른 측정이다. 이 프로젝트는 카메라가 정밀하다고 주장하지 않는다. 빨간 블록 파지가 가능했던 것은 넓은 그리퍼 허용 범위가 남은 인지 오차를 흡수했기 때문이다.

### Gazebo에서 확인

- 사람 명령을 받아 빨간 블록 서빙
- 초록 블록을 명령 없이 자가정리
- 운반 중 물체 상실 후 `GRIPPER_EMPTY → REGRASP → 재인지 → 작업 재개`
- 반복 실패 시 복구 예산 소진, `ABORTED_WAIT`, 사람 확인 요청
- `RESUME` 이후 작업 재개

### 언어 모델 평가에서 확인

현재 비교 정본은 기존 182개 한·영 시험에 10개 언어 60개를 붙인 총 242개 시험이다.

| 모델 | 전체 결과 | 중앙 지연 | 해석 |
|---|---:|---:|---|
| `gemma4:26b` | 220/242, 90.9% | 3.57초 | 정확도 1위, 다국어 59/60 |
| `qwen3.5:9b` | 211/242, 87.2% | 1.88초 | 정확도와 지연의 균형 |
| `exaone3.5:7.8b` | 191/242, 78.9% | 1.14초 | 완주 모델 중 가장 빠름 |

이 수치는 모델이 시험 문장을 스킬 계획으로 옮긴 결과다. 로봇 전체 작업 성공률이나 “10개 언어 안전 제어 완료”를 뜻하지 않는다. 언어별 표본도 6개뿐이다.

---

## 9. 정직한 현재 한계

- 고정 카메라, 고정 테이블, 정해진 색 물체에 맞춘 프로토타입이다.
- 카메라는 단안 평면 매핑이며 일반적인 3D/6D pose estimation이 아니다.
- 카메라 렌즈 왜곡을 정식 보정하지 않아 위치별 잔차가 남는다.
- 실물의 대표 범위는 빨간 블록과 감독되는 한 명의 client다.
- 파란 링은 pick/place 사례가 있지만 연속 반복 성공률을 확정하지 못했다.
- 그리퍼 서보 전원 순단의 근본 원인은 완전히 닫지 못했다.
- pick/place 시퀀스 중간 cancel은 지원하지 않으며 요청을 정직하게 거부한다.
- 전체 recovery 상태 머신은 Gazebo 검증이며 실물 고장 주입은 부분 검증이다. 다만 2026-08-22 실물에서 `GRIPPER_EMPTY → 재파지 → 복구 예산 소진 → 중단`의 전체 사이클이 자연 발생으로 1회 확인되었다(계획된 반복 주입은 아니다).
- Planning Scene에 주변 물체와 든 물체를 자동 반영하는 기능은 없다.
- 현재 LLM 검증기는 스텝별 계약은 검사하지만 모든 2단계 계획이 반드시 `같은 물체의 pick → place`인지 보장하는 의미 불변식은 추가 보강 여지가 있다.
- CI는 전체 핵심 패키지를 아직 모두 포괄하지 않으며, 최신 전체 테스트를 깨끗한 한 장의 결과로 마감하는 작업이 남아 있다.
- 연구용 안전 gate를 구현한 것이며 산업 안전 인증을 받은 시스템이 아니다.

이 한계는 프로젝트가 실패했다는 뜻이 아니다. 프로젝트가 실제로 증명한 범위를 정하고, 다음 사람이 같은 조건에서 결과를 재현할 수 있게 만드는 경계다.

---

## 10. 이 프로젝트를 통해 사용자가 최종적으로 얻은 것

### 10.1 ROS 2 애플리케이션 통합 능력

사용자는 단일 노드를 작성한 것이 아니라 센서, 인지, 기구학, planning, controller, actuator, agent를 하나의 실행 흐름으로 연결했다. 이는 ROS 2 Application/Systems Integration 직무와 가장 직접적으로 연결되는 결과다.

### 10.2 실물 로봇 디버깅 능력

팔이 움직이지 않을 때 무조건 코드를 수정하지 않고 다음 계층을 나눠 확인하는 경험을 얻었다.

```text
전원 → USB/시리얼 → 하드웨어 인터페이스 → controller
→ joint state → MoveIt plan → trajectory execute → 실제 관절·그리퍼 반응
```

특히 그리퍼만 토크가 없던 사건에서 `/dxl_state`, torque state, 전원 재기동, 실제 0.5 위치 명령을 통해 하드웨어 상태를 분리한 경험은 실무적인 강점이다.

### 10.3 실패를 계약으로 표현하는 설계 능력

“안 됐다”가 아니라 물체 없음, 도달 불가, 계획 실패, 파지 실패, 운반 중 낙하, 시간 초과, 취소를 서로 다른 코드로 표현했다. 이 덕분에 호출자는 원인에 맞는 행동을 결정할 수 있다.

### 10.4 측정하고 주장을 제한하는 능력

사용자는 팔의 위치 오차와 카메라 관측 오차를 분리했다. 성공 사례가 있어도 분모가 없으면 성공률을 만들지 않았고, simulation 결과를 실물 결과로 포장하지 않았다. 이는 기술 자체만큼 중요한 엔지니어링 태도다.

### 10.5 AI를 시스템 안에 제한해서 사용하는 능력

LLM을 “모든 것을 결정하는 두뇌”로 두지 않고, 고수준 계획 번역기로 제한했다. 모델 비교도 정확도 하나가 아니라 정상 명령, 거부 명령, 지연, 언어별 차이로 나눠 측정했다.

### 10.6 범위를 줄이고 프로젝트를 끝내는 능력

모바일 베이스, RGB-D, VLA/RL, 범용 인식, 6개 구역, 정밀 캘리브레이션 같은 확장을 멈췄다. 취업 목표에 맞춰 실물 red pick/place, 안전 gate, 측정, 영상 증거를 남기는 방향으로 전환했다.

---

## 11. 이 프로젝트가 취업에서 보여주는 정체성

가장 강한 정체성은 다음이다.

> **ROS 2 Application / Robot Application / Robotics Systems Integration 개발자**

그 근거는 다음과 같다.

- ROS 2 package, node, launch, parameter, topic, action을 실제 시스템으로 구성
- RGB camera와 DYNAMIXEL 팔·그리퍼의 sensor-actuator integration
- 해석 IK, MoveIt 2, ros2_control을 연결한 manipulation application
- simulation에서 만든 인터페이스를 실물에서도 유지
- bring-up, serial, controller, TF, perception, gripper 문제를 계층별로 디버깅
- 실측 수치, 실패 코드, 안전 gate, 운영 한계를 문서화

AI와 다국어는 좋은 차별점이지만 첫 번째 직무 정체성은 아니다. “AI 로봇 알바”는 사람의 관심을 끄는 이야기이고, 그 안을 지탱하는 채용 증거는 ROS 2 통합과 실물 디버깅이다.

이 프로젝트만으로 다음 분야를 주력으로 주장하는 것은 효율적이지 않다.

- 순수 perception/SLAM 연구
- VLA/RL 연구
- 동역학·힘 제어·whole-body control
- 산업 안전 인증 시스템
- 범용 자율 로봇 제품

### 11.1 2026-08-23 채용 공고 검색으로 다시 확인한 결론

아래 표는 **2026-08-23에 공개 페이지가 열려 있거나 채용 목록에서 현재 노출되는 공고**를 기준으로 작성했다. `상시`, `채용 시 마감` 공고도 예고 없이 닫힐 수 있으므로 실제 지원 직전 48시간 안에 원문을 다시 확인한다. 여기서 `A`, `B`는 합격 가능성 예측이 아니라 **이 프로젝트가 이미 가진 증거와 JD의 거리**다.

#### A 레인 — 지금 가진 증거로 설명이 바로 되는 공고

| 우선 | 공고 | 공고가 요구하는 핵심 | 이 프로젝트의 직접 증거 | 지원 시 정직하게 말할 간극 |
|---:|---|---|---|---|
| 1 | [로보티즈 · 휴머노이드 시스템 소프트웨어 엔지니어](https://robotisrecruiter.ninehire.site/job_posting/aE8A4gYI) | ROS 2 시스템 SW, 센서·액추에이터·제어기 연동, DYNAMIXEL, ros2_control, 실로봇 이슈 분석. 경력 무관·채용 시 마감 | OpenRB/DYNAMIXEL 실기, ros2_control controller 3종, C++/Python, Linux·serial·USB 디버깅, MoveIt 연동 | 벤더 hardware interface를 **연동·진단**했지 새 HAL/driver를 처음부터 작성한 것은 아님 |
| 2 | [어드밴텍 · Linux & Robot Application Engineer (ROS)](https://www.peoplenjob.com/jobs/6241248) | Linux application, ROS 1/2 robot application, camera interface, USB/MIPI debugging, OpenCV. 제목은 신입·채용 시 | USB RGB camera, `usb_cam`, CameraInfo·TF, OpenCV detector, ROS 2 application, C++/Python | MIPI와 상용 vision product 경험은 없음. 페이지의 제목은 신입이지만 경력 필드가 혼재하므로 지원 전 확인 |
| 3 | [로보티즈 · 상시 인턴 Physical AI 제품 기능/RX 시스템](https://robotisrecruiter.ninehire.site/job_posting/iuFSE1yT) | ROS 2 제어·센서·통신 통합, sim-real 연동, 실패 분석, 예제·매뉴얼, MoveIt/Gazebo | 이 저장소의 전체 구조와 거의 일치. 실물 팔, Gazebo, 실패 원장, 재현 명령, 촬영 계획까지 제시 가능 | Isaac Sim과 학습 기반 정책은 이 프로젝트의 주력 증거가 아님 |
| 4 | [리얼월드 · Robotics Deployment 인턴](https://www.wanted.co.kr/wd/322082) | 로봇·센서 HW/SW setup, 현장 테스트·디버깅·성능 검증, 데이터·기술 문서, ROS 2/C++/Python | 단계별 bring-up, 장치 권한·serial·controller·TF 진단, 실측표, rosbag, 마일스톤 문서 | 고객사 배포·3D scanner·학습 데이터 운영은 미경험 |
| 5 | [Riibotics · 소프트웨어 엔지니어 인턴](https://riibotics.career.greetinghr.com/ko/o/122388) | ROS 2 package/architecture, sensor driver 연동, Linux, Gazebo/Isaac 기반 시험 | 8개 로컬 패키지 경계, launch/config, camera 연동, Gazebo와 실물의 동일 Action 계약 | LiDAR·IMU·Nav2 및 Isaac Sim 경험은 없음 |
| 6 | [엑스와이지 · 로봇 제어 엔지니어](https://xyz.career.greetinghr.com/ko/o/198921) | ROS 2 기반 제어 시스템, 센서·모터 node/interface 연동. 채용 목록상 경력 무관 | sensor→Action→controller→motor 전체 연결, 그리퍼 feedback, 오류 코드, 실물 안정화 | 저수준 motor control과 동역학 제어를 직접 개발했다고 주장하면 안 됨 |

이 중 **로보티즈 시스템 SW**는 단순히 ROS 2라는 단어만 겹치는 공고가 아니다. `DYNAMIXEL`, `ros2_control`, 센서·액추에이터·제어기 연동, 실로봇 이슈 분석까지 프로젝트의 실제 문제와 같은 언어로 적혀 있다. 따라서 현재 가장 강한 1순위 증거는 LLM 데모가 아니라 **실물 bring-up과 하드웨어 경계 디버깅**이다.

#### B 레인 — 핵심은 맞지만 연구·경력·산업 깊이의 간극이 있는 공고

| 공고 | 맞는 부분 | 현재 간극 | 지원 판단 |
|---|---|---|---|
| [피트인 · 매니퓰레이터 제어 엔지니어](https://www.wanted.co.kr/wd/358808) | 실물 매니퓰레이터, FK/IK, OMPL/MoveIt, vision-to-control, 반복 실험 | 동역학·진동 제어, 산업용 arm, planner 커스터마이징 경험 부족 | 조작 직무 B 레인으로 도전. analytic IK와 실물 오차 분리를 전면에 둠 |
| [LiOps · Robotics Engineer](https://www.wanted.co.kr/wd/318704) | ROS/ROS 2 manipulation, 실물 HW 통합·디버깅, motion planning, Gazebo, fail-safe | 3D point cloud, AMR/Nav, 산업용 arm, 100ms 최적화, 현장 PoC 부족 | 공고의 핵심 요건 중 ROS와 실물 HW 두 축으로 도전하되 3D/AMR 간극 공개 |
| [로아이 · Motion Planning Engineer](https://www.wanted.co.kr/wd/275399) | kinematics, C++/Python, ROS 2, sim-real planning 검증 | 경력 1~5년, Isaac Sim, RRT/PRM 자체 구현·최적화, dynamics 부족 | MoveIt **사용 경험**을 planner 알고리즘 개발로 부풀리지 않는 조건부 도전 |
| [투모로 · Robotics Systems & Platform SW](https://tommoro.ai/career/robotics-systems-platform-software-engineer) | package/node/launch/config, sensor-actuator-AI interface, serial, Docker, test, 문서, 실로봇 안정화 | 석사 또는 학사+2년/동등 역량, HAL·network·telemetry·CI/CD·성능 분석 깊이 | 직무 방향은 매우 정확한 상향 지원. 시스템 증거 묶음으로 `동등 역량`을 설득 |
| [위로보틱스 · 로봇 소프트웨어 엔지니어](https://rih.wirobotics.com/careers/robot-software) | Linux/ROS 2/DDS, logging·monitoring, Gazebo/Isaac, CI integration test | real-time Linux, multithreading, network, 대규모 시스템과 고성능 최적화 부족 | 현재 정체성의 다음 단계. 당장 주력보다 성장 방향을 보여주는 공고 |

공고들을 함께 보면 회사와 로봇 형태가 달라도 다음 요구가 반복된다.

1. `package/node/launch/parameter/config`로 시스템을 구성하는 ROS 2 구조화 능력
2. camera·sensor·actuator·controller·AI 사이의 명시적인 interface
3. C++와 Python, Linux 환경에서의 build·run·debug
4. 실제 하드웨어 bring-up, serial/USB/network 문제 분리, field debugging
5. kinematics·motion planning·simulation 결과를 실물에 적용하고 검증한 경험
6. logging·bag·test·문서·재현 절차를 통해 한 번의 성공을 운영 가능한 증거로 바꾸는 능력

이 반복 요구 때문에 이 프로젝트의 취업용 이름은 그대로 유지한다.

> **ROS 2 기반 실물 매니퓰레이터 Application/System Integration**
>
> 부제: **perception·planning·control·hardware를 같은 계약으로 연결하고 실패를 측정한 작업 셀**

`Physical AI`, `다국어`, `LLM`은 관심을 끄는 두 번째 문장이다. 첫 문장에서 AI를 앞세우면 실제로 가장 잘 맞는 시스템·응용·deployment 공고의 핵심 증거가 뒤로 밀린다.

### 11.2 공고 요구와 저장소 증거의 정확한 연결

| 반복되는 JD 문장 | 저장소에서 보여줄 증거 | 실물·수치 증거 | 촬영해야 할 장면 |
|---|---|---|---|
| ROS 2 package/node/interface 설계 | [`src/arm_interfaces`](src/arm_interfaces), [`src/arm_bringup`](src/arm_bringup), Action 3종과 상태 message | mock→Gazebo→real에서 같은 goal/result 계약 유지 | package tree, Action 정의, `node/action list`, 단계별 bring-up |
| C++/Python 통합 | C++ `arm_kinematics`·`arm_skills`, Python `arm_perception`·`arm_agent` | Python scene이 C++ Action goal로 이어져 실물 실행 | 코드 두 화면과 실제 결과를 같은 clip에서 연결 |
| sensor·actuator·controller 연동 | `usb_cam`→TF→detector→MoveIt→ros2_control→DYNAMIXEL | `/scene_state` 약 30Hz, `/joint_states` 약 100Hz, controller 3종 active | 카메라 debug, TF, controller, 실제 팔을 순서대로 촬영 |
| kinematics·motion planning | [`src/arm_kinematics/src/ik.cpp`](src/arm_kinematics/src/ik.cpp), branch·joint-limit·FK oracle, MoveIt plan/execute 분리 | 팔 6점 목표 오차 1~6mm, red pick 3회 연속 | IK round-trip test, unreachable 거부, MoveIt 계획, 실물 pick |
| 실로봇 bring-up·debug | [`real_arm.launch.py`](src/arm_bringup/launch/real_arm.launch.py), Docker device/cgroup, 계층별 원장 | ACM 번호 변경, serial 단절, controller, gripper torque 문제를 실측으로 분리 | USB 장치→controller→topic→motion을 하나씩 올리는 화면 |
| 실패·안전 처리 | [`ErrorCode.msg`](src/arm_interfaces/msg/ErrorCode.msg), single-flight, stale gate, watchdog, cancel, safe park | cancel·single-flight·watchdog·stale·종료 실물 발동 확인 | 재현 위험이 낮은 terminal 증거 + 이미 검증된 cancel 1회 |
| simulation과 실물 검증 | Gazebo scene과 real launch가 동일 Action 계층 사용 | 정상 동작은 실물, 위험한 recovery 반복은 Gazebo | 같은 `/pick` 계약의 sim/real 병렬 편집 |
| AI와 결정론적 실행의 경계 | schema·화이트리스트·scene validation·fallback | 242개 계획 변환 평가, 최고 220/242; 로봇 성공률과 분리 | 자연어→JSON→검증→Action을 단계별 화면으로 촬영 |
| test·logging·문서 | GTest, pytest, CSV 평가, rosbag, `handoff/milestones` | 수치의 조건·실패·한계까지 원장으로 추적 가능 | 테스트 결과, bag info, 실측표, known limitations 카드 |

이 표에서 중요한 점은 “사용했다”와 “개발했다”를 구분하는 것이다.

- 직접 설계·구현한 것: 사용자 정의 interface, analytic IK/FK, C++ skill orchestration, OpenCV perception adapter, agent/validator/recovery, launch/config 경계, 실물 시험과 측정 절차
- 프레임워크를 구성·연동·디버깅한 것: MoveIt 2, ros2_control, Gazebo, DYNAMIXEL hardware interface, `usb_cam`
- 사용하지 않은 것: Isaac Sim, LiDAR/point cloud, Nav2/SLAM, EtherCAT/CAN, force/impedance control, industrial robot language

면접에서 두 번째 항목을 첫 번째처럼 말하지 않는 편이 오히려 강하다. 프레임워크 내부 알고리즘을 만들지 않았어도, 서로 다른 계층이 실제 장비에서 올바른 결과와 실패를 주고받게 만든 통합 경험은 별도의 실무 역량이다.

### 11.3 이 프로젝트가 단순 튜토리얼보다 강한 이유

튜토리얼은 보통 정상 경로 한 번에서 끝난다. 이 프로젝트의 채용 가치는 다음 다섯 경계를 실제 문제로 통과했다는 데 있다.

1. **센서와 로봇의 경계**: 픽셀을 TF와 평면 가정으로 world 좌표로 바꾸고 오차를 별도 측정했다.
2. **계획과 실행의 경계**: MoveIt plan 성공과 DYNAMIXEL execute 실패를 분리해 거짓 성공을 막았다.
3. **시뮬레이션과 실물의 경계**: camera pitch, TCP 높이, 중력·마찰, USB 권한처럼 sim에 없던 오차를 다시 측정했다.
4. **확률 모델과 결정론적 제어의 경계**: LLM은 최대 2단계 고수준 계획만 제안하고 validator와 Action server가 실행 권한을 갖는다.
5. **기능과 운영의 경계**: cancel, stale data, single-flight, watchdog, 종료, 복구 예산과 known limitation을 명시했다.

따라서 대표 영상도 “블록을 집었다”만 보여주면 프로젝트 가치의 절반 이상을 버린다. **장치가 하나씩 살아나는 과정, 실패를 올바르게 보고하는 장면, 측정값과 로그**가 함께 있어야 공고가 찾는 실무 증거가 된다.

### 11.4 지원 우선순위와 포트폴리오 구성

지원은 프로젝트가 더 완벽해질 때까지 미루지 않는다.

1. 로보티즈 시스템 SW와 어드밴텍 Robot Application처럼 현재 증거가 JD 문장과 직접 겹치는 정규직 공고
2. 로보티즈·리얼월드·Riibotics처럼 실물 프로젝트가 강한 구분자가 되는 인턴·전환형 공고
3. 피트인·LiOps처럼 조작·통합 핵심은 맞고 산업 장비·3D·경력 간극이 있는 도전 공고
4. 투모로·위로보틱스처럼 system platform의 다음 성장 방향을 보여주는 상향 공고
5. pure VLA/RL, perception research, dynamics/force control 전담은 이 프로젝트 하나로 억지 지원하지 않음

제출 묶음은 다음 순서가 좋다.

```text
1페이지 요약
→ 60~90초 대표 영상
→ 2~4분 기술 증거 영상
→ 무편집 real master와 Git 저장소
→ 실측·실패 원장 링크
```

채용 담당자는 처음부터 30분짜리 영상을 보지 않는다. 짧은 영상으로 관심을 얻고, 기술 면접관이 원하면 독립 clip과 원장으로 깊이를 확인하게 한다.

### 11.5 이력서·면접에 바로 쓸 핵심 문장

**이력서 제목**

> ROS 2 기반 실물 매니퓰레이터 시스템 통합 — RGB perception, analytic IK, MoveIt 2, ros2_control, DYNAMIXEL

**bullet 후보**

- ROS 2 Action을 경계로 RGB camera, 5축 analytic IK, MoveIt 2, ros2_control, DYNAMIXEL arm/gripper를 연결하고 Gazebo와 실물에서 동일한 `pick/place/move_to` 계약을 유지했습니다.
- 실물 `/joint_states` 약 100Hz와 `/scene_state` 약 30Hz를 확인하고, 서로 다른 위치의 camera-derived red pick 3회 연속과 3개 목적지 place 단계를 검증했습니다.
- plan 성공과 execute 성공을 분리해 serial 단절의 거짓 성공을 차단하고, stale scene·single-flight·watchdog·cancel·안전 종료를 명시적 결과로 만들었습니다.
- LLM을 motor command 경로에서 분리하고 schema·화이트리스트·scene validation·deterministic fallback으로 제한했으며, 10개 언어를 포함한 242개 계획 변환 시험으로 모델을 비교했습니다.

한 지원서에는 2~3개만 선택한다. 로보티즈 시스템 SW에는 1·3번, 어드밴텍에는 1·2번과 camera 디버깅, deployment에는 2·3번과 실측 문서를 우선한다.

**면접 첫 45초**

> “저는 ROS 2에서 센서, 계획, controller, actuator 사이의 계약을 설계하고 실물에서 안정화하는 쪽을 목표로 합니다. 이 프로젝트에서는 RGB 카메라의 scene state를 C++ Action skill로 연결하고, analytic IK와 MoveIt 2를 거쳐 ros2_control과 DYNAMIXEL 팔을 움직였습니다. 특히 plan 성공과 실제 execute 실패를 분리하고 stale data, 동시 goal, timeout, cancel을 명시적인 실패로 만들었습니다. AI는 모터 제어가 아니라 검증 가능한 고수준 계획에만 제한했습니다.”

### 11.6 현재 공백과 다음 학습 방향

| 공고에서 반복되지만 현재 약한 영역 | 현재 사실 | 면접에서의 정직한 답 | 프로젝트를 지금 확장할지 |
|---|---|---|---|
| hardware interface/driver 자체 개발 | DYNAMIXEL interface를 연동·진단했지만 vendor 구현이 기반 | “상위 계약과 controller 연동, 장치·통신 진단까지 했고 HAL 신규 작성은 미경험” | 같은 요구가 지원 피드백에서 반복될 때 작은 mock driver 과제로 분리 |
| DDS·network·real-time·latency profiling | ROS_DOMAIN_ID와 DDS 통신을 운용했지만 middleware 튜닝·RT 분석은 얕음 | “기능 통합 경험은 있고 정량 latency/RT 최적화는 다음 학습 영역” | 현재 대표 영상보다 우선하지 않음 |
| CI/CD·diagnostics·telemetry·replay | Docker, test, rosbag, 문서는 있으나 제품 수준 자동화는 아님 | “재현·기록 기반은 만들었고 CI 범위와 telemetry는 한계로 공개” | 문서·영상 마감 후 작은 CI 보강만 검토 |
| 3D sensor·Planning Scene | 단안 RGB+평면 가정, 주변 물체 collision scene 자동 반영 없음 | “고정 셀 가정으로 범위를 제한했고 3D 일반화는 주장하지 않음” | 이 프로젝트에는 추가하지 않음 |
| dynamics·force/impedance·trajectory optimization | 위치 제어와 OMPL 사용 수준 | “IK와 실행 통합은 했지만 동역학·힘 제어 알고리즘 개발은 미경험” | 별도 학습 프로젝트가 필요한 영역 |
| Isaac Sim·industrial arm·field deployment | Gazebo와 소형 DYNAMIXEL arm 실기 | “sim-real 문제 분리 경험은 있지만 해당 플랫폼·고객 현장은 미경험” | 취업 후 또는 특정 과제 요구 시 전환 학습 |

이 공백을 모두 지금 채우려 하면 정체성이 다시 흐려진다. 먼저 현재 강점의 증거를 촬영하고 지원한다. 이후 여러 공고나 면접 피드백에서 같은 한 가지 공백이 반복될 때만 다음 프로젝트를 고른다.

### 11.7 취업용 증거의 우선순위

| 등급 | 반드시 남길 증거 | 이유 |
|---|---|---|
| P0 | 실물 red `pick → place → home` 무편집 원본, camera 화면, Action result | 시스템이 실제로 닫힌 루프로 작동한다는 최소 증거 |
| P0 | 하드웨어부터 agent까지 하나씩 bring-up하고 각 층을 확인하는 화면 | 시스템 SW·application·deployment 공고에 가장 직접적 |
| P0 | 30~45초 본인 설명 | 코드를 이해하고 의사결정을 소유한다는 증거 |
| P1 | cancel, stale, single-flight, safe shutdown, plan/execute 분리 | 정상 데모를 넘어 실패를 설계했다는 차별점 |
| P1 | IK test, controller, TF, `/scene_state`, rosbag·실측표 | 기술 면접에서 질문을 견디는 근거 |
| P2 | 다국어 계획, 자가정리, Gazebo recovery | 관심을 끄는 차별점이지만 실물 통합보다 뒤에 배치 |
| P3 | ring 사례, 긴 benchmark, 모든 로그 | 요청받았을 때만 제공하는 보조 자료 |

구체적인 독립 촬영 목록과 안전한 순서는 [`VIDEO_SHOOTING_PLAN.md`](VIDEO_SHOOTING_PLAN.md)의 부록 C에 정리한다.

---

## 12. 토론할 질문

이 문서를 바탕으로 사용자와 다음 항목을 하나씩 결정할 수 있다.

### 프로젝트 정체성

1. 첫 문장을 “ROS 2 실물 통합”과 “AI 로봇 알바” 중 어디에서 시작할 것인가?
2. 취업용 설명에서 AI·다국어를 어느 정도 비중으로 둘 것인가?
3. 사용자 자신은 기구학, 시스템 통합, 하드웨어 디버깅, 에이전트 중 무엇을 가장 자신 있게 설명할 수 있는가?

### 대표 기능

4. 빨간 블록 실물 pick/place를 유일한 대표 시나리오로 둘 것인가?
5. 초록 블록 자가정리는 실물로 보여줄 만큼 안정적인가, 아니면 Gazebo 증거로 제한할 것인가?
6. 안전 cancel 장면과 자가정리 장면 중 60초 영상에 무엇을 우선할 것인가?

### 기술 설명

7. 해석 IK를 직접 구현한 이유를 사용자가 1분 안에 설명할 수 있는가?
8. 카메라 오차 24~51mm인데도 pick이 가능했던 이유를 설명할 수 있는가?
9. LLM이 모터를 직접 제어하지 않는 구조가 왜 중요한지 설명할 수 있는가?
10. `GRASP_FAILED`와 `GRIPPER_EMPTY`를 나눈 이유를 실물 사례로 설명할 수 있는가?

### 정직한 한계와 마감

11. 파란 링과 양끝 작업 구역을 포트폴리오에서 완전히 빼도 되는가?
12. CI와 최신 테스트 상태를 지원 전에 어디까지 정리할 것인가?
13. 사용자에게 남은 가장 큰 불안은 코드, 실물 반복성, 설명, 영상 중 무엇인가?
14. 이 프로젝트를 더 개발하지 않고 지원으로 전환할 종료 기준에 동의하는가?

---

## 13. 처음 보는 사람을 위한 용어 사전

| 용어 | 쉬운 설명 |
|---|---|
| ROS 2 | 로봇의 여러 프로그램이 메시지를 주고받게 하는 소프트웨어 기반 |
| Node | 카메라, 인지, 팔 제어처럼 한 가지 역할을 맡은 실행 프로그램 |
| Topic | 상태나 센서값을 계속 방송하는 통신 방식 |
| Action | 시간이 걸리는 작업을 요청하고 진행·결과·취소를 다루는 통신 방식 |
| IK | 손끝 목표 위치로부터 필요한 관절 각도를 구하는 계산 |
| FK | 관절 각도로부터 실제 손끝 위치를 구하는 계산 |
| MoveIt 2 | 로봇 팔의 경로 계획과 실행을 돕는 ROS 2 프레임워크 |
| ros2_control | 상위 로봇 명령과 실제 또는 가상 하드웨어 controller를 연결하는 프레임워크 |
| TF | 카메라, 로봇 베이스, 손끝처럼 서로 다른 좌표계의 관계를 관리하는 기능 |
| DYNAMIXEL | 위치, 속도, 전류 상태를 통신할 수 있는 스마트 서보 모터 |
| Gazebo | 실제 장비 없이 로봇과 환경을 시험하는 물리 시뮬레이터 |
| LLM | 자연어를 이해하고 생성하는 대규모 언어 모델 |
| Watchdog | 작업이 정해진 시간 안에 끝나지 않으면 멈추게 하는 감시 장치 |
| Fail-closed | 상태를 확신하지 못할 때 추측해 움직이지 않고 실패로 멈추는 원칙 |
| Bring-up | 전원, 장치, driver, controller, 센서, 로봇 모델을 순서대로 실제 동작 상태로 올리는 과정 |

---

## 14. 더 자세한 근거

- 현재 상태와 취업 전환 기준: [`handoff/MASTER.md`](handoff/MASTER.md)
- 프로젝트 종료 판단: [`handoff/CLOSEOUT_ROS2_APPLICATION_2026-08-19.md`](handoff/CLOSEOUT_ROS2_APPLICATION_2026-08-19.md)
- 실물 pick/place와 오차 측정: [`handoff/milestones/M6_G0_2026-08-19.md`](handoff/milestones/M6_G0_2026-08-19.md)
- 실물 안전 기능 검증: [`handoff/milestones/M6_SAFETY_2026-08-22.md`](handoff/milestones/M6_SAFETY_2026-08-22.md)
- 채용 담당자용 한 페이지 초안: [`handoff/reference/ONE_PAGER_2026-08-22.md`](handoff/reference/ONE_PAGER_2026-08-22.md)
- 사용자의 전체 학습 기록: [`README.md`](README.md)
- 자가정리 영상: [`media/edited/T3_자가정리.mp4`](media/edited/T3_자가정리.mp4)
- 낙하 후 복구 영상: [`media/edited/T4_강탈_복구.mp4`](media/edited/T4_강탈_복구.mp4)
- 복구 예산 소진 영상: [`media/edited/T5_예산소진_직원호출.mp4`](media/edited/T5_예산소진_직원호출.mp4)

## 마지막 요약

이 프로젝트에서 사용자가 만든 것은 “말을 듣고 블록 하나를 옮기는 팔”만이 아니다.

> **불확실한 자연어와 카메라 입력을 제한된 계약으로 받아들이고, 표준 ROS 2 실행 계층을 거쳐 실물 하드웨어를 움직이며, 실패를 구분하고 멈추거나 복구할 수 있는 작은 로봇 애플리케이션 시스템**을 만들었다.

그리고 사용자가 얻은 가장 큰 결과는 기능 개수가 아니라 다음 세 가지다.

1. 여러 로봇 기술을 하나의 실제 동작으로 연결하는 통합 능력
2. 실물에서 발생하는 문제를 계층별 측정으로 좁히는 디버깅 능력
3. 검증한 범위와 한계를 구분해 설명하고 프로젝트를 마감하는 엔지니어링 판단
