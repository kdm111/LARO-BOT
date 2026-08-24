# 실물 로봇 전체 행동 촬영 목록

> 기준일: 2026-08-23  
> 촬영 대상: **실물 로봇만**  
> Gazebo: 최종 목록에서 제외  
> 원본 저장 위치: `media/portfolio`

이 목록의 완료 기준은 대표 동작 몇 개만 고르는 것이 아니다. 현재 프로젝트 소스에서 **서로 다른 팔·그리퍼 동작 분기**가 실제로 실행된다면 원본 영상으로 남긴다. 같은 동작을 좌표만 조금 바꿔 무한 반복하지는 않되, 구역·파지 방식·행동 주체·복구 전략이 다르면 별도 영상으로 취급한다.

## 1. 촬영 원칙

1. 모든 영상은 실물 팔과 결과 terminal을 동시에 담는다.
2. Agent 영상은 자연어 입력, 채택한 계획, 상태 전이, 실물 움직임, 최종 결과까지 끊지 않는다.
3. 한 번에 하나의 행동만 촬영하고 저장한 뒤 다음 행동으로 넘어간다.
4. gripper 단독 open/close는 찍지 않는다. 실제 pick/place 안에서 촬영한다.
5. 실패 복구도 실물로 남기되, 사람 손으로 운반 중 물체를 빼앗거나 케이블을 분리하지 않는다.
6. `cancel`, 강제 watchdog, serial 단절처럼 물리 정지 수단이 필요한 실험은 현재 장비 조건에서 재현하지 않는다.
7. 자연스럽게 파지 실패나 낙하가 발생하면 녹화를 끄지 않고 Agent의 복구가 끝날 때까지 계속 찍는다.

## 2. 이름 자세와 기본 이동

| ID | 상태 | 실물 동작 | 반드시 보일 것 |
|---|---|---|---|
| R01 | 완료 | `home → init` | `/move_to init`, 실제 관절 이동, Action success |
| R02 | 완료 | `init → home` | `/move_to home`, 실제 복귀, Action success |

`move_to`를 반복해서 더 찍지는 않는다. 이름 자세 이동은 R01·R02 두 개로 끝낸다.

## 3. 구역별 실물 pick/place 순환

구역마다 실제 좌표와 pick trim이 다르므로 work 한 곳만 찍고 끝내지 않는다. 같은 red block을 네 구역으로 순환시키면 사람 손으로 중간 재배치하지 않고 모든 구역 접근을 남길 수 있다.

| ID | 상태 | 실물 동작 | 코드에서 달라지는 부분 |
|---|---|---|---|
| R03 | 완료 | red `work → counter` | work pick, counter place, retreat, home. 기존 `pick.webm`+`place.webm` |
| R04 | 완료 | red `counter → shelf` | 일본어 자연어 Agent 명령, counter 구역 pick trim, +y shelf 접근·보정 |
| R05 | 완료 | red `shelf → bin` | 영어 자연어 Agent 명령, shelf 구역 pick trim, bin place |
| R06 | 제외 | red `bin → work` | `work`는 Agent 자연어 target whitelist 밖이며 놓은 직후 자가정리가 재출발함 |

각 영상은 다음 전체 사이클을 포함한다.

```text
새 scene 대기
→ 접근
→ 하강
→ 그리퍼 닫기와 파지 판정
→ lift와 재확인
→ 저속 운반
→ place 접근
→ release
→ retreat
→ home
```

## 4. 물체별 다른 파지 동작

red와 green은 같은 크기의 블록이라 pinch 동작이 같다. blue ring은 반벌림 진입, hook 플래그, 별도 높이·판정값을 사용하므로 반드시 별도 행동으로 남긴다.

| ID | 상태 | 실물 동작 | 촬영 의미 |
|---|---|---|---|
| R07 | R11로 확보 | green block `work → bin` | 불량 블록 pick과 수거함 폐기 동작 |
| R08 | 조건부 신규 | blue ring `work → shelf` | 반벌림 진입, 링 파지, lift, 낮은 release, shelf 복귀 |

R08은 현재 소스 주석상 최신 hook 하강 방식의 재검증이 남아 있다. 따라서 앞선 블록 동작이 모두 정상이고 camera pose가 안정적일 때 마지막에 진행한다. 첫 시도에서 비정상 접근이 보이면 반복하지 않는다.

## 5. Agent가 발생시키는 실물 행동

직접 Action과 Agent 행동은 팔 궤적이 같아도 실행 주체와 상태 전이가 다르므로 둘 다 남긴다.

### R09. 사람 자연어 명령 전체 사이클

- 상태: 촬영 완료 — 실물 pick/place/home 성공, 최종 camera verification 실패(`served: 0`, `aborted: 1`)
- 명령 예시: `빨간 블록을 카운터에 놔줘`
- 실물 동작: red `work → counter → home`
- 한 영상에 포함:
  - 자연어 `/command`
  - `LLM 계획 채택: pick red_block, place counter`
  - `IDLE → RUNNING`, `source: human`
  - 실제 pick/place/home
  - 각 Action `success=True, code=0`
  - `VERIFYING`
  - camera가 counter의 red block을 재확인
  - `IDLE`, `served` 증가

### R10. red block 자가 정리

- 상태: 완료
- 시작: red block을 work에 놓고 사람 명령을 보내지 않는다.
- 실물 동작: 2초 방치 감지 → red `work → shelf → home`
- 한 영상에 포함:
  - `IDLE`
  - `자가 정리 실시: red_block > shelf`
  - `source: self`
  - 실제 pick/place/home
  - `VERIFYING → IDLE`
  - `cleaned` 증가

### R11. green block 자가 폐기

- 상태: 완료
- 시작: green block을 work에 놓고 사람 명령을 보내지 않는다.
- 실물 동작: 2초 방치 감지 → green `work → bin → home`
- 촬영 의미: 물체별 목적지 정책이 단순 자막이 아니라 실제 다른 팔 경로로 실행됨

### R12. blue ring 자가 정리

- 상태: 파지 실패·자동 재시도 실사 확보, 정상 완료 영상은 미완료
- 시작: blue ring을 work에 놓고 사람 명령을 보내지 않는다.
- 실물 동작: 2초 방치 감지 → blue ring `work → shelf → home`
- 촬영 의미: 특수 파지와 Agent 정책이 한 시퀀스에서 연결됨

## 6. 실물 행동 복구

복구는 포트폴리오의 핵심이므로 Gazebo로 대체하지 않는다. 다만 안전하게 만들 수 있는 복구와 물리 고장 주입이 필요한 복구를 구분한다.

### R13. `OBJECT_NOT_FOUND → RESCAN → home → 재인지 → 재실행`

- 상태: 신규, 소프트웨어 방식으로 준비
- 방법 원칙: cable을 뽑지 않고 perception 발행만 일시 정지한다.
- 시작 자세: `init`
- 보일 동작:
  1. pick이 goal 이후 새 scene을 기다리다 `OBJECT_NOT_FOUND` 반환
  2. Agent가 `RESCAN` 선택
  3. 상태 `RECOVERING`
  4. 실물 팔 `init → home`으로 물러나 시야 확보
  5. perception 복원 후 fresh scene 수신
  6. 같은 물체를 새 좌표로 pick/place
  7. camera 재확인 후 종료
- 안전 조건: 자동 재실행 타이밍을 사전에 구성한 뒤 시작하며, 작업 중 사람 손은 셀 안에 넣지 않는다.

### R14. `GRASP_FAILED → REGRASP`

- 상태: 완료 — blue ring 파지 실패 후 자동 재시도 실사 확보
- 보일 동작:
  - 빈 파지 판정
  - 물체 위에서 안전 높이로 retreat
  - home 복귀
  - Agent `REGRASP`
  - fresh scene으로 다시 접근·파지
- 의도적 재현 금지:
  - 움직이는 팔 앞에서 물체를 치우지 않는다.
  - calibration 값을 고의로 틀어 테이블을 향하게 하지 않는다.
- 확보 방법: R07·R08·R10~R12 정상 촬영 중 자연스럽게 발생하면 녹화를 유지한다.

### R15. `GRIPPER_EMPTY → REGRASP`

- 상태: 자연 발생 시 촬영 완료로 전환
- 보일 동작:
  - lift 또는 transfer 중 파지 상실 판정
  - home 복귀
  - Agent가 복구 스텝을 삽입
  - 재인지 → repick → 원래 place 재개
- 의도적 재현 금지: 운반 중 물체를 손으로 빼앗지 않는다.

### R16. `PLANNING_FAILED/EXECUTION_TIMEOUT → RETRY`

- 상태: 자연 발생 시 촬영 완료로 전환
- 보일 동작: 실패한 동일 goal을 `attempt=2`로 다시 계획하고 실행
- 의도적 재현 금지: watchdog 예산을 촬영용으로 낮추거나 장애물을 갑자기 넣지 않는다.

### R17. 복구 예산 소진 → `ABORTED_WAIT → RESUME`

- 상태: R13 완료 후 별도 준비
- 안전한 실패 원인: perception 일시 정지로 stale scene을 만들고, 물리 간섭 없이 실패 예산을 소진한다.
- 보일 내용:
  - 현재 설정 `MAX_ATTEMPTS=2`, `MAX_RECOVERY=1`
  - 반복 실패 후 `ignored` 등록
  - `ABORTED_WAIT`에서 자동 행동이 더 나오지 않음
  - perception 복원
  - 사람이 `RESUME`
  - 실물 팔 home 복귀
  - fresh scene을 읽고 정상 작업 재개
- 주의: 과거 T5의 3회/복구 2회 영상은 현재 설정 증거로 사용하지 않는다.

## 7. 종료 행동

### R18. skill server 안전 종료

- 상태: 사용자 결정으로 촬영 제외
- 조건: gripper가 비어 있고 주변이 정리된 상태
- 실물 동작: 진행 동작 stop → home 이동 → torque 유지
- 보일 것: `종료 절차 : home 도달 - 토크 유지한 채 종료`
- 이 영상 뒤에는 같은 세션에서 추가 goal을 보내지 않는다.

## 8. 이미 있는 무동작 안전 영상

다음은 로봇이 움직이지 않는 것이 성공인 장면이다. 이미 촬영했으므로 다시 찍지 않는다.

| 원본 | 내용 |
|---|---|
| `media/portfolio/invalid_pose.webm` | `UNDEFINED_POSE`, 팔 무동작 |
| `media/portfolio/S02.webm` | `OBJECT_NOT_FOUND`, 팔 무동작 |
| `S03_agent_unsupported_purple_refusal.webm` | 지원하지 않는 purple block 자연어 명령을 Agent가 실행 전 거부, 팔 무동작 |

## 9. 촬영하지 않는 실물 고장 주입

- 운반 중 물체를 손으로 빼앗기
- 움직이는 팔 앞에서 물체 위치 바꾸기
- USB/serial cable 분리
- watchdog 시간을 강제로 낮춰 팔 정지시키기
- pick/place 중 cancel
- 두 client로 동시 goal 경쟁

이 항목들은 기능이 없어서 제외하는 것이 아니다. 현재 셀에 독립된 물리 비상정지 장치가 없으므로 포트폴리오 촬영을 위해 새 위험을 만들지 않는 것이다.

## 10. 실제 촬영 순서

```text
R01 → R02
→ R09                           사람 Agent: work → counter
→ R04 → R05 → R06              counter → shelf → bin → work 구역 순환
→ R10                           red 자가 정리
→ R07 → R11                     green 직접 동작과 자가 폐기
→ R13                           안전한 RESCAN 복구
→ R17                           예산 소진과 RESUME
→ R08 → R12                     링은 마지막 조건부
→ R18                           안전 종료
```

R14·R15·R16은 위 정상 행동을 촬영하는 동안 자연 발생하면 별도 파일로 보존한다. 일부러 실패를 만들기 위해 정상 물체나 하드웨어에 손대지 않는다.

현재 필수 촬영은 모두 끝났다. R08/R12 정상 ring 성공,
R13 RESCAN, R15 운반 낙하, R16 planning retry, R17 ABORTED_WAIT는 조건부다.
blue ring `GRASP_FAILED → REGRASP` 실사를 이미 확보했으므로 포트폴리오를 위해
나머지 실패를 억지로 만들지는 않는다. 실제 안내는 **한 항목씩** 제공한다.
