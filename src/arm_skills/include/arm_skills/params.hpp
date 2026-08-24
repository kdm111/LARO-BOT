#pragma once

#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <utility>

// 실물 서보가 EEPROM으로 강제하는 관절 한계.
// sim에는 이 한계가 없고 gz_ros2_control이 gpio 파라미터를 읽지 않는다.
// ★ 2026-08-20 motion.cpp의 파일 지역 상수였던 것을 여기로 올렸다. trace_zone이
//   같은 한계를 봐야 하는데, 안전 한계를 복사하면 언젠가 한쪽만 바뀐다.
static constexpr double kRealJointMin[5] = {-M_PI, -1.868, -1.571, -M_PI, -M_PI};
static constexpr double kRealJointMax[5] = {M_PI, 1.658, 1.675, M_PI, M_PI};

// 그리퍼 목표값 둘. 2026-08-20 밤, ID16(그리퍼) 토크 상실 두 번(전원 보호 추정)의 대책.
// gripper_controller 는 마지막 목표를 100Hz 로 계속 쓰므로, 도달할 수 없는 목표가
// 걸려 있으면 서보는 그 자리를 무기한 민다. "미는 상태를 만들어 두지 않는다"가 원칙.
//   kGripperOpen : 벌림 목표. SRDF "open"(=1.0)은 기구 끝점(실측 0.9925) 너머라
//     열어 둔 내내 끝점을 밀었다. 끝점 안쪽이면 도달한 뒤 무부하로 쉰다.
//   kGripPreload : 쥔 뒤 남겨 두는 조임량. grasp_check 가 닫아서 잰 관절값에서
//     이만큼 더 조인 값을 새 목표로 되건다(gripper.cpp 주석) - close(0.0)를 문 채
//     운반하면 위치 오차 0.2~0.3 을 향해 내내 전류 한계로 간다.
//     ⚠️ 0.03 은 실물 검증 전이다(2026-08-20 밤). 쥔 힘이 약해 미끄러지면 올릴 것 -
//     운반 낙하 임계가 이 값에 걸려 있으니(skill_place.cpp) 같이 봐야 한다.
static constexpr double kGripperOpen = 0.95;
static constexpr double kGripPreload = 0.03;
// ★ 2026-08-21 유지 목표의 하한. 0.0 으로 클램프하면 얇은 것(링 벽, 쥔 값 0.017)
//   에서 조임 오차가 kGripPreload 가 아니라 0.017 로 줄어 절반 힘이 된다 - 실제로
//   운반 중 링을 떨어뜨렸다(사용자 관찰 "꽉 잡자"). 빈 손 과닫힘 실측이 -0.0184
//   까지 갔으므로 -0.02 는 물리적으로 도달 가능한 범위다 - 여기까지 열어 두면
//   어떤 두께든 조임 오차가 kGripPreload 로 일정해진다.
static constexpr double kGripCloseFloor = -0.02;

// ★ 2026-08-21 실측 : hook 진입 벌림(0.5)에서 손끝 사이 간격 = 6.5cm (사용자 자
//   실측, 관절 0.4955 에서). §8.5 환산(0.55 ≈ 4.1cm)은 이 실측과 어긋난다.
static constexpr double kHookHalfSpan = 0.0325;
// ★ hook 파지점 오프셋 = 벽의 한가운데, (내경 1.75 + 외경 3.0) / 2 = 2.375cm.
//   TCP 를 여기 두면 두 손끝(±kHookHalfSpan)이 구멍 안(-0.875cm 지점)과 링 밖
//   (+5.625cm 지점)에 서고, 닫힐 때 안벽(+1.75)·바깥벽(+3.0)에 "동시에" 닿는다
//   (양쪽 이동 거리 2.6cm 로 같다) - 링을 끌지 않고 벽 1.25cm 를 반듯하게 문다.
//   ※ 처음엔 "안쪽 손끝을 구멍 정중앙에"(오프셋 3.25cm)로 했다가 바꿨다 -
//     접촉 시점이 어긋나 닫히며 링을 끌었고, 비스듬히 물려(관절 0.3022 잼)
//     들 때 빠졌다(2026-08-21 실측 : "잡기는 했는데 들지는 못했다").
//   허용 오차 : 오프셋이 ±0.875cm 틀어져도 안쪽 손끝이 구멍 안이다.
// ★ 2026-08-22 미사용 전환 : 감싸기 모드(skill_pick.cpp 파지점 주석)로 바꾸며
//   조준에서 뺐다. 벽 물음으로 되돌릴 때를 위해 값과 유도는 남긴다.
static constexpr double kHookPinchOffset = 0.02375;
// ★ 2026-08-21 밤 : hook 하강 위치의 x 트림. 육안 관찰("로봇 쪽으로 못 미친다,
//   +x 로 25mm")로 0.025 를 넣었다가 한 번 시도 후 사용자 지시로 롤백했다(0.0).
//   +2.5cm 로도 빈 손이었다 - x 밀림 가설은 이 한 번으로는 확정도 반증도 아니다.
static constexpr double kHookXTrim = 0.0;
// ★ 2026-08-21 밤 : hook y 트림. 이중 실측으로 확정했다 -
//   ① 0.25 벌림의 실측 간격 4.0cm(자) -> 기하상 손끝이 벽 양쪽 1.4cm 여유여야 함
//   ② 그런데 바깥쪽 손끝이 바깥벽 위에 "딱" 닿아 있었다(사용자 육안)
//   -> 실물 그리퍼가 모델 대비 -y 로 ~1.375cm 비껴 내려온다(손끝 비대칭 또는
//      TCP 원점 치우침으로 추정, 원인층 미확정). 조준에 +y 로 같은 양을 얹어
//      상쇄한다. 잡기·놓기 양쪽에 함께 건다(같은 기하가 거울로 반복되므로).
// ★ 0.014 -> 0.018 -> 0.014 -> 0.0 (2026-08-22 새벽). "비대칭 1.4cm" 실측은
//   당시 검출 편향(시대마다 흘러 다니는 ring_dy)이 섞인 허상이었을 가능성이
//   드러났다 - 전원 리셋 후 poke 로 검출이 정확해진 상태에서 조준이 정확히
//   +1.4cm 만큼 왼쪽으로 치우쳤다. 0 으로 놓고 재검증한다. 진짜 기계적
//   비대칭이 있다면 이번엔 검출이 맞는 상태에서 다시 나타날 것이다.
static constexpr double kHookYTrim = 0.0;


// 놓을 자리(base_link 기준). ★ 같은 좌표가 세 곳에 있다 - 하나만 고치면 조용히 어긋난다:
//   ① scene_cell.sdf 의 zone_* 모델 pose  ② cell.py 의 ZONE(구역 판정)  ③ 여기(팔이 실제로 가는 곳)
//   counter = 카운터(zone_counter, 주문의 종착지), bin = 수거함(zone_bin, 불량품)
//   ★ 2026-08-20 저녁 셋 다 자리가 바뀌었다. shelf·counter 는 종이 위(5x3cm),
//     bin 만 종이 밖 오른쪽 구석이다 - 버리는 자리를 작업면에 두지 않는다.
//   shelf = 창고. 2026-08-19 실물에서 shelf_block/shelf_ring 을 하나로 합쳤다.
//   ※ eval 시험지 182케이스가 "bin"을 놓는 자리로 쓴다. 그 값은 계약 어휘로 살아 있으므로
//     파싱 시험은 그대로 유효하다 - 다만 의미는 "수거함"으로 바뀌었다.
// ★ 2026-08-19 실물 반영: x 0.085 -> 0.140 (근거는 cell_layout.yaml 머리 주석).
// ★ 2026-08-20 작업 구역이 놓는 자리가 됐고, 같은 날 낮에 두 칸으로 갈렸다가
//   저녁에 다시 하나로 합쳐졌다(사용자 결정). work 는 이제 실물 A4 석 장 중
//   가운데 한 장을 안쪽으로 들인 것이다. 중심 x 0.157 은 그 종이 중심(blue_ring 실측)
//   이고, y 는 저녁에 좌우를 다르게 줄이면서 0.015 -> 0.010 으로 옮겨졌다.
//   ⚠️ 합치면서 "한 점이면 블록 둘을 되돌릴 때 겹친다"는 문제가 되살아났다.
//      두 칸으로 갈랐던 이유가 그것이었다. 근거와 남은 문제는 cell_layout.yaml.
// ★ 2026-08-22 shelf y 0.153 -> 0.123, work 중심 0.010 -> -0.005 (사용자 결정).
//   shelf place 착지가 목표보다 오른쪽(-y)에 앉는 +y 영역 팔 편향의 대응 -
//   조준 트림 대신 구역 자체를 팔이 실제로 놓는 자리로 3cm 옮겼다.
static const std::map<std::string, std::pair<double, double>> kTargets = {
  {"counter", {0.154, -0.156}},
  {"bin", {0.054, -0.134}},
  {"shelf", {0.140, 0.123}},
  {"work", {0.157, -0.00525}}
};

// ★ 2026-08-22 : +y 영역의 팔-카메라 y 편향 보정(pick·place 공용).
//   실측 3점이 원점 통과 직선에 맞는다 :
//     조준 y 0.153 -> 착지 -6.2cm  (0.43*0.153 = 0.066)
//     조준 y 0.123 -> 착지 -5.3cm  (0.43*0.123 = 0.053)
//     중심부 y~0.01 -> 편차 ~0
//   즉 +y 로 갈수록 팔이 명령 좌표보다 오른쪽(-y)에 선다. shelf place 착지 미달과
//   shelf 쪽 pick 헛손질(카메라 좌표는 맞는데 팔이 비켜 섬 - 링의 왼쪽 실패 포함)이
//   같은 원인이다. 원인층은 왜곡계수 0 인 카메라 매핑 의심 - 정식 캘리브 전까지의
//   실측 보정. -y 쪽은 적용하지 않는다(counter 는 보정 없이 명중해 왔다).
inline double arm_y_bias(double y)
{
  return y > 0.0 ? std::min(0.43 * y, 0.07) : 0.0;
}

// work 구역의 y 경계. 진실은 arm_bringup/config/cell_layout.yaml 이고,
// kTargets와 마찬가지로 스킬이 실시간 조준에 쓰는 사본이다.
inline constexpr double kWorkPickYMin = -0.0885;
inline constexpr double kWorkPickYMax = 0.0780;

struct PickZoneRect
{
  double x_min;
  double x_max;
  double y_min;
  double y_max;
};

inline constexpr PickZoneRect kWorkPickZone{0.1295, 0.1845, -0.0885, 0.0780};
inline constexpr PickZoneRect kCounterPickZone{0.1290, 0.1790, -0.1710, -0.1410};
inline constexpr PickZoneRect kShelfPickZone{0.1150, 0.1650, 0.1080, 0.1380};
inline constexpr PickZoneRect kBinPickZone{0.0290, 0.0790, -0.1490, -0.1190};

inline constexpr bool in_pick_zone(const PickZoneRect & zone, double x, double y)
{
  return x >= zone.x_min && x <= zone.x_max && y >= zone.y_min && y <= zone.y_max;
}

enum class PickZone
{
  WORK,
  COUNTER,
  SHELF,
  BIN,
  NONE
};

inline constexpr PickZone pick_zone_of(double x, double y)
{
  return in_pick_zone(kWorkPickZone, x, y) ? PickZone::WORK :
         in_pick_zone(kCounterPickZone, x, y) ? PickZone::COUNTER :
         in_pick_zone(kShelfPickZone, x, y) ? PickZone::SHELF :
         in_pick_zone(kBinPickZone, x, y) ? PickZone::BIN : PickZone::NONE;
}

// 실물 카메라 중앙 트림은 work 전체에서 연속이어야 한다. 예전 |y| <= 0.06
// 조건은 work 안쪽에 불연속을 만들어 경계를 1mm 넘는 순간 조준이 4.5cm
// 오른쪽으로 뛰었다. shelf/counter/bin은 이 y 범위 밖이라 영향을 받지 않는다.
// ★ 2026-08-23 실물 최종: trim=0.045에서 green_block을 work 오른쪽
//   y≈-0.061, 중앙 y≈-0.005, 왼쪽 y≈+0.044 세 점에서 집기·lift 성공했고,
//   세 번 모두 사용자가 손가락이 블록 중앙을 정확히 물었다고 육안 확인했다.
inline constexpr double work_pick_y_trim(double x, double y, double trim)
{
  return pick_zone_of(x, y) == PickZone::WORK ? trim : 0.0;
}

inline constexpr double zone_pick_x_trim(
  PickZone zone, double counter_trim, double shelf_trim, double bin_trim)
{
  return zone == PickZone::COUNTER ? counter_trim :
         zone == PickZone::SHELF ? shelf_trim :
         zone == PickZone::BIN ? bin_trim : 0.0;
}

inline constexpr double zone_pick_y_trim(
  PickZone zone, double counter_trim, double shelf_trim, double bin_trim)
{
  return zone == PickZone::COUNTER ? counter_trim :
         zone == PickZone::SHELF ? shelf_trim :
         zone == PickZone::BIN ? bin_trim : 0.0;
}

static_assert(work_pick_y_trim(0.157, kWorkPickYMin, 0.045) == 0.045);
static_assert(work_pick_y_trim(0.157, kWorkPickYMax, 0.045) == 0.045);
static_assert(work_pick_y_trim(0.157, kWorkPickYMin - 0.0001, 0.045) == 0.0);
static_assert(work_pick_y_trim(0.157, kWorkPickYMax + 0.0001, 0.045) == 0.0);
static_assert(pick_zone_of(0.154, -0.156) == PickZone::COUNTER);
static_assert(pick_zone_of(0.140, 0.123) == PickZone::SHELF);
static_assert(pick_zone_of(0.054, -0.134) == PickZone::BIN);
static_assert(zone_pick_x_trim(PickZone::COUNTER, 0.025, 0.0, 0.0) == 0.025);
static_assert(zone_pick_y_trim(PickZone::COUNTER, 0.010, 0.0, 0.0) == 0.010);

// 물체별 파지 파라미터. 어떻게 잡을 것인가의 주체는 skill이다.
//   depth   : 인지가 준 물체 중심 z에서 TCP(손가락 끝)를 얼마나 더 내릴 것인가.
//             손가락은 TCP에서 위로 뻗으므로 중심에 두면 윗절반만 물려 미끄러진다.
//   place_z : 놓을 때의 TCP 높이. 물체 높이가 다르면 놓는 높이도 다르다.
//   hold_eps: 파지 판정 임계. 문 두께가 다르면 손가락 관절값이 자릿수째로 다르다.
//             빈 손은 어느 물체든 0에 가깝다(낙하 순간 0.32 -> 0.0001 실측).
// ★ 2026-08-20 offset 필드 삭제(사용자 결정 : "벽 노리는거 다 빼라").
//   속이 빈 물체의 "벽"을 겨냥해 파지점을 중심에서 옆으로 옮기던 값이다.
//   벽은 두께 1.25cm 인데 카메라 오차가 1~2cm 라 표적이 오차보다 작았다 -
//   원리상 맞을 수 없는 조준이었고, 08-19 부터 오늘까지 링 파지 실패의 뿌리였다.
//   중심을 겨냥하면 표적이 바깥지름 6cm 가 되어 허용 오차가 5배가 된다.
//   실측이 그대로 말해 준다 : 벽 물음 0.0138 vs 전체 물음 0.3267 (24배).
//   필드 자체를 지운다 - 아무도 안 쓰는 값을 남겨 두면 언젠가 또 켜진다.
struct GraspSpec
{
  double depth;
  double place_z;
  double hold_eps;
  // 내려가기 전에 벌려 둘 폭(그리퍼 관절값).
  // ★ 2026-08-20 : 이제 전부 kGripperOpen(활짝)이다. 물체 전체를 가로질러 물기 때문이다.
  //   1.00 이었다가 같은 날 밤 kGripperOpen 으로 - 1.0 은 기구 끝점 너머다(위 주석).
  //   구 주석은 "링은 덜 벌려야 한다"고 했는데, 그건 벽을 물던 시절의 이야기다
  //   (안쪽 손가락을 구멍에 넣어야 해서 개구부를 좁혀야 했다). 벽 조준을 버린 지금은
  //   링도 블록과 같다 - 물체보다 넓게 벌리고 통째로 문다.
  //   ※ 구 주석의 "관절 1.35 ≈ 10cm, 0.55 ≈ 4.1cm" 환산은 믿지 말 것.
  //     그 환산이 맞다면 0.50 으로 바깥지름 6cm 링을 감쌀 수 없어야 하는데
  //     실제로는 감쌌다(2026-08-20). 아직 다시 재지 않았다.
  double open_pos;
  // ★ 2026-08-19 추가: 물체 위 몇 m 에서 접근할 것인가.
  //   블록(높이 4cm)용 0.06 을 링(높이 1.2cm)에 그대로 쓰면 6cm 위에서 내려오는데,
  //   그 여정에서 MoveIt 이 계획한 궤적이 관절공간에서 휘며 손가락이 링을 스쳐
  //   밀어낸다(2026-08-19: pick 은 다섯 번 다 실패했는데, 같은 좌표·같은 폭으로
  //   home 에서 곧장 내려가 닫으니 한 번에 잡혔다 - 관절 0.0138).
  //   납작한 물체는 낮게 접근해야 안전하다.
  double approach_dz;
  // ★ 2026-08-20 추가: 들고(lift)/놓고(place-retreat) 물러나는 최소 높이.
  //   전역 kRetreatZ 였던 것을 물체별로 옮겼다. 운반 충돌 때문에 블록은 0.12 로
  //   올려야 하는데, 링은 0.10 으로 파지·완주가 검증된 상태고(2026-08-19)
  //   "높이가 크게 바뀌면 IK 가 다른 손목 해를 골라 j5 가 뒤집힌다"는 관측이 있어
  //   전역으로 같이 올리면 검증을 잃는다. 그래서 블록만 올린다.
  //   place_z + approach_dz 로만 잡으면 납작한 물체(링)에서 6.6cm 밖에 안 올라가고,
  //   home 복귀에서 팔이 방금 놓은 물체를 쓸고 간다(2026-08-14 실측 9cm 이탈).
  //   그래서 하한으로 쓴다. 상한은 도달 범위다: 높이를 올릴수록 닿는 반경이 준다
  //   (실측 z=0.14 에서 bin, 0.16 에서 shelf 가 빠진다).
  double retreat_z;
  // ★ 2026-08-21 추가: 파지 방식(사용자 결정 : "링은 훅으로 집자, 0.5만 벌리고").
  //   false = pinch. 활짝 벌려 내려가 밖에서 조인다(블록).
  //   true  = hook. "반만(open_pos) 벌리고" 내려가 닫는다(링). 안쪽 손가락이
  //           구멍(내경 3.5cm)에 들어가면 닫힐 때 벽이 걸리고, 둘 다 밖이면
  //           링 전체를 감싼다 - 어느 쪽이든 쥔다. 벌림이 좁아 닫는 행정이
  //           짧으니 링을 옆으로 쳐내지도 못한다.
  //   활짝 벌린 pinch 는 조준을 실측 보정으로 중심에 맞춘 뒤에도 실패했다
  //   (2026-08-21 다섯 번 : 벽 집기로 밀려남 / 허공 닫힘).
  //   판정은 pinch 와 같은 grasp_check 다 - hold_eps 0.008 이 두 결과를 다 받는다
  //   (벽 물음 0.0138 / 전체 물음 0.3267 실측). 다른 점은 진입뿐 : 벌림을 직접
  //   보내 정착을 확인하고 내려간다(MoveIt 은 반쯤 움직인 채 돌아오는 실측이 있다).
  //   ※ 처음엔 "오므려 넣고 안에서 벌리는" 역방향 훅으로 잘못 구현했다가 바로
  //     되돌렸다(구현 hook_check 는 삭제) - 사용자 의도는 반벌림 진입이었다.
  bool hook;
};
static const std::map<std::string, GraspSpec> kGrasps = {
  // 블록 4x6x4cm. 중심 z=0.02에서 1cm 내려 아랫절반을 문다.
  // ★ place_z = depth를 뺀 파지 높이와 같아야 한다(0.02 - 0.010 = 0.010).
  //   집을 때 TCP가 0.010이면 블록 중심은 TCP보다 1cm 위에 매달린다. 놓을 때
  //   TCP를 0.020으로 두면 중심이 0.030이 되어 바닥에서 1cm 뜬 채로 손을 편다 -
  //   떨어지며 튀어서 2026-08-15 실측 3.3cm 어긋났고(목표 (0.060,-0.145) ->
  //   실제 (0.087,-0.164)) 옆 구역으로 판정됐다. 0.010이면 낙차가 0이다.
  //   구 주석의 "먼 자리(r=0.248)에서 IK 불가"는 옛 구역 배치의 값이다 -
  //   지금 place 목적지는 넷 다 r <= 0.19라 접근 높이에 여유가 있다.
  // ★ retreat_z 2026-08-20 0.10 -> 0.12(빨강·초록 공통). 운반 중 쥔 블록이 테이블 위
  //   블록과 부딪혔다. TCP 는 블록 중심보다 1.5cm 위(depth −0.015)라 쥔 블록 밑면은
  //   TCP−3.5cm 이다. 0.10 이면 밑면 0.065, 테이블 블록 윗면 0.040 -> 여유 2.5cm 뿐이고
  //   MoveIt 경로가 조금만 처져도 닿는다. 0.12 면 여유 4.5cm.
  {"red_block", {-0.015, 0.0350, 0.050, kGripperOpen, 0.060, 0.12, false}},
  // 불량품. red_block과 모양·크기·질량이 같아 값도 같다.
  {"green_block", {-0.015, 0.0350, 0.050, kGripperOpen, 0.060, 0.12, false}},
  // 링 바깥지름 6.0 / 안지름 3.5 / 두께 1.2cm. 중심 z=0.006, 윗면 z=0.012.
  //
  // ★ 2026-08-20 저녁, 링을 블록과 같은 방식으로 다룬다(사용자 결정 :
  //   "벽 노리지 말고 전체를 집는 걸로"). 08-14 부터 이어진 "벽을 문다" 계열을 버린다.
  //   버린 이유는 표적 크기다 - 벽 두께 1.25cm 는 카메라 오차(1~2cm)보다 작아서
  //   원리상 맞출 수 없었다. 중심을 겨냥하면 표적이 바깥지름 6cm 가 된다.
  //   실측 대비 : 벽 물음 0.0138  vs  전체 물음 0.3267 (24배)
  //   블록 4cm 가 0.21 이니 0.3267 은 약 6cm - 링을 통째로 가로질러 문 값이다.
  //
  // ★ depth 를 -0.014 -> -0.004 로 낮췄다(TCP 0.020 -> 0.010).
  //   TCP = obj_z - depth 이므로 옛 값은 손가락 끝이 0.020 에 섰는데, 링 윗면이
  //   0.012 다. 즉 링보다 8mm 위에서 닫고 있었다 - 허공을 물 수밖에 없었다.
  //   블록과 견주면 어긋남이 분명하다 :
  //     블록 TCP 0.035 / 윗면 0.040  -> 윗면보다 5mm 아래(문다)
  //     링   TCP 0.020 / 윗면 0.012  -> 윗면보다 8mm 위(못 문다)
  //   0.010 이면 링 윗면보다 2mm 아래가 되어 블록과 같은 구도가 된다.
  //
  // ★ hold_eps 0.008 은 그대로 둔다. 벽 물음(0.0138) 기준으로 잡은 값인데
  //   전체 물음은 0.32 라 훨씬 위다 - 하한으로서 여전히 유효하다.
  //   ※ 다만 hook(손가락이 구멍 3.5cm 를 통과해 링이 걸려 올라오는 것)은 이 임계로
  //     못 가른다. 그때 그리퍼는 끝까지 닫혀 관절값이 빈 손과 같아지는데 링은 들린다
  //     (2026-08-20 실측: 판정 0.0031 "빈 손" 인데 실제로는 들려 있었다).
  //     depth 를 낮춰 손가락이 링 몸통 높이에서 닫히게 한 것이 hook 자체를 줄인다.
  // ★ retreat_z 는 0.10 유지 - 높이를 올리면 도달·손목 해(j5 뒤집힘)가 다시 흔들린다.
  //
  // ★ 2026-08-21 hook 전환 : depth -0.004 -> 0.000 -> 0.002 (TCP 0.010 -> 0.006
  //   -> 0.004). pinch 시절 TCP 0.010 은 "윗면보다 2mm 아래"의 얕은 걸침이었다.
  //   TCP 0.006(벽 중간)으로 첫 훅 파지에 성공했지만 운반 중 떨어뜨렸다 -
  //   "조금만 더 내려가서 꽉"(사용자)으로 0.004. 테이블 위 4mm 다.
  //   ⚠️ 팔이 실제보다 낮게 서면 손끝이 테이블을 누른다(팔 관절 과부하) -
  //   더 낮추는 것은 실물 확인 후에.
  //   open_pos : 진입 벌림. 0.50(실측 간격 6.5cm) -> 0.30 (2026-08-21 저녁,
  //   사용자 지정). 선형 환산(6.5cm/0.5)으로 간격 ≈ 3.9cm - 아래 손끝이 구멍 안
  //   (중심 +0.4cm), 위 손끝이 링 밖(바깥벽 +1.3cm)에 선다. 닫는 행정이 짧아
  //   내려오며 스치거나 끌 여지가 준다.
  // ★ 0.30 -> 0.15 (2026-08-21 밤). 0.30(간격 3.9cm)도 내려가던 손끝이 링에
  //   부딪혀 바닥까지 못 갔다(사용자 관찰 "거의 다 왔어 - 부딪혀서 못 내려갔다").
  //   0.15 는 선형 환산 간격 ≈ 2.0cm - 두 손끝이 벽(1.25cm) 양옆 3.5mm 틈으로
  //   내려가 안벽 안/바깥벽 밖에 선다.
  // ★ 0.15 -> 0.20 -> 0.25 -> 0.20 -> 0.50 (사용자 지시로 단계 조정).
  //   손목 해 고정으로 "중심부로는 잘 간다"가 확인된 뒤, 위치를 믿고 진입을
  //   다시 활짝(간격 6.5cm 실측)으로 열었다 - 하강 중 벽 충돌을 여유로 피한다.
  //   실측 2점 : 0.25 -> 4.0cm, 0.5 -> 6.5cm.
  // ★ approach_dz 0.025 -> 0.06 (2026-08-21 밤, 사용자 : "조금 더 위에서,
  //   블록과 똑같은 높이에서 내려가자"). 홈 -> approach 도착 이동도 관절 공간이라
  //   옆·아래로 휘는데, 2.5cm 접근은 그 휨이 링 높이를 스친다. 블록과 같은 6cm 면
  //   도착 휨의 여유가 생기고, 거기서부터는 수직 스텝 하강(skill_pick.cpp)이라
  //   높이가 늘어도 조준이 안 흐트러진다. 구 값 0.025 의 근거("납작한 물체는
  //   낮게 접근")는 활짝 벌린 pinch 시절 이야기다.
  // ★ depth 0.002 -> 0.003 -> 0.004 (TCP 4 -> 3 -> 2mm, 2026-08-21 밤 사용자
  //   지정 "조금 더, 방법이 없어"). 수직 스텝 lift 로도 드는 중 낙하가 남았고
  //   고무 패드는 이미 붙어 있어 마찰 카드는 소진 - 물림 깊이 10mm 까지 늘린다.
  //   ⚠️ 테이블 여유 2mm - 바닥이라 경고했으나 사용자가 1mm 를 승인했다
  //   ("1mm 까지 내려가도 될 거 같아"). depth 0.005 = TCP 1mm, 물림 11mm.
  //   하강 중 긁는 소리가 나면 즉시 되돌릴 것.
  // ★ 0.50 에서 "갑자기 잘 집힌다"(사용자) -> 0.55 -> 0.60 으로 한 칸씩(사용자
  //   지시). 보간 간격 ≈ 7.5cm - 링(6cm)을 넉넉히 넘는 벌림이다.
  {"blue_ring", {0.005, 0.0200, 0.008, 0.60, 0.060, 0.10, true}}
};
