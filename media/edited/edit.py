#!/usr/bin/env python3
"""촬영 원본 4개에 자막과 상태 오버레이를 얹는다. 원본은 건드리지 않는다.

자막 시각은 손으로 찍은 것이 아니라 agent 로그의 이벤트 시각에서 계산했다.
  video_t(event) = anchor_video + (log_t(event) - anchor_log)
anchor_video는 ffmpeg freezedetect가 잡은 "정지 화면이 끝나는 순간"(= 팔이 움직이기
시작한 시각)이고 anchor_log는 그 순간의 로그 시각이다. 그래서 자막이 실제 사건과 맞는다.

가제보 창을 통째로 녹화했으므로 툴바(위 145px)와 상태바(아래 31px)를 잘라낸다.
"""
import os
import subprocess
import sys

MEDIA = '/ws/media'
OUT = os.path.join(MEDIA, 'edited')
FONT = 'Noto Sans CJK KR'

# ---- ASS 헬퍼 ----------------------------------------------------------------


def ts(t):
    """초 -> ASS 시각 h:mm:ss.cc"""
    t = max(0.0, t)
    h = int(t // 3600)
    m = int((t % 3600) // 60)
    s = t % 60
    return f'{h}:{m:02d}:{s:05.2f}'


def ass_header(w, h):
    return f"""[Script Info]
ScriptType: v4.00+
PlayResX: {w}
PlayResY: {h}
WrapStyle: 2
ScaledBorderAndShadow: yes

[V4+ Styles]
Format: Name, Fontname, Fontsize, PrimaryColour, SecondaryColour, OutlineColour, BackColour, Bold, Italic, Underline, StrikeOut, ScaleX, ScaleY, Spacing, Angle, BorderStyle, Outline, Shadow, Alignment, MarginL, MarginR, MarginV, Encoding
Style: Sub,{FONT},24,&H00FFFFFF,&H000000FF,&H00000000,&H50000000,0,0,0,0,100,100,0,0,3,5,0,2,26,26,20,1
Style: Hud,{FONT},18,&H0000E5FF,&H000000FF,&H00000000,&H70000000,1,0,0,0,100,100,0,0,3,5,0,7,16,16,14,1
Style: Title,{FONT},34,&H00FFFFFF,&H000000FF,&H00000000,&H90000000,1,0,0,0,100,100,0,0,3,8,0,8,30,30,18,1

[Events]
Format: Layer, Start, End, Style, Name, MarginL, MarginR, MarginV, Effect, Text
"""


def line(style, start, end, text):
    return f'Dialogue: 0,{ts(start)},{ts(end)},{style},,0,0,0,,{text}\n'


def write_ass(path, w, h, subs, hud, titles=()):
    body = ass_header(w, h)
    for s, e, t in titles:
        body += line('Title', s, e, t)
    for s, e, t in hud:
        body += line('Hud', s, e, t)
    for s, e, t in subs:
        body += line('Sub', s, e, t)
    with open(path, 'w', encoding='utf-8') as f:
        f.write(body)


def run(cmd):
    print('$', ' '.join(cmd[:6]), '...', flush=True)
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stderr[-3000:], file=sys.stderr)
        raise SystemExit(f'ffmpeg 실패: {r.returncode}')


def render(src, dst, crop, ass_path, segments=None):
    """crop -> (배속) -> 자막 -> h264. segments=[(시작,끝,배속), ...]"""
    cw, ch, cx, cy = crop
    pre = f'crop={cw}:{ch}:{cx}:{cy},fps=30,format=yuv420p'
    esc = ass_path.replace(':', r'\:')
    if segments:
        n = len(segments)
        # 같은 스트림을 여러 번 읽을 수 없으므로 split으로 갈라 각각 trim 한다.
        chain = f'[0:v]{pre},split={n}' + ''.join(f'[s{i}]' for i in range(n)) + ';'
        chain += ';'.join(
            f'[s{i}]trim={a}:{b},setpts=(PTS-STARTPTS)/{sp}[v{i}]'
            for i, (a, b, sp) in enumerate(segments)) + ';'
        chain += ''.join(f'[v{i}]' for i in range(n))
        chain += f'concat=n={n}:v=1:a=0[cat];'
        chain += f"[cat]ass='{esc}'[out]"
    else:
        chain = f"[0:v]{pre},ass='{esc}'[out]"
    run(['ffmpeg', '-v', 'error', '-i', src, '-filter_complex', chain,
         '-map', '[out]', '-an', '-c:v', 'libx264', '-preset', 'medium',
         '-crf', '20', '-pix_fmt', 'yuv420p', '-movflags', '+faststart',
         dst, '-y'])


# ---- 공통 크롭 ----------------------------------------------------------------
# 창 전체가 아니라 3D 뷰포트에서 실제로 뭔가 일어나는 부분만 남긴다.
# 위 145px은 툴바, 그 아래로도 하늘이 절반 가까이 비어 있었다(첫 렌더에서 실측).
# 아래쪽은 재생/일시정지 버튼과 RTF 표시가 있어 잘라낸다.
CROP_A = (760, 470, 110, 430)   # 987x945 원본 (T1 T3 T4)
CROP_T5 = (760, 470, 105, 430)  # 971x977 원본 (T5 — 창이 더 커서 같은 470을 얻는다)
W_A, H_A = 760, 470
W_5, H_5 = 760, 470


# ---- T1 : 사람 주문 -> 서빙 ---------------------------------------------------
# anchor : 영상 25.8s = 로그 1786789073.25 (pick goal 수락, 팔이 움직이기 시작)
T1_SRC = 'T1_팔이_창고에서_빨간_블록을_집는다.webm'
T1_SUB = [
    (0.5, 8.0, '대기 — 명령을 기다리는 게 아니라 작업 구역을 계속 보고 있다'),
    (8.5, 24.5, '빨강은 블록 창고, 파랑 링은 링 창고. 작업 구역은 비어 있다'),
    (25.5, 32.0, '사람 명령 : "빨간 블록을 카운터에 놔줘"'),
    (32.5, 39.0, 'LLM이 계약 어휘로 옮긴다 → [pick red_block, place counter]'),
    (40.0, 47.0, '물체의 긴 축을 재서 그리퍼 방향을 정하고 접근한다'),
    (48.0, 55.0, '파지 성공 — 그리퍼 관절값으로 판정한다 (빈 손 ≤0.03, 쥠 0.32~0.45)'),
    (60.0, 71.0, '카운터로 운반'),
    (77.0, 83.0, '놓기 완료 — 목표에서 1.05cm'),
    (83.5, 89.0, '액션은 성공을 보고했다. 그래도 카메라로 다시 확인한다 → 통과'),
]
T1_HUD = [
    (0.0, 25.8, '상태  IDLE            서빙 0'),
    (25.8, 83.5, '상태  RUNNING · 사람 명령   서빙 0'),
    (83.5, 85.6, '상태  VERIFYING (씬 재확인)  서빙 0'),
    (85.6, 89.0, '상태  IDLE            서빙 1'),
]
T1_TITLE = [(0.5, 4.5, '① 주문을 받아 서빙한다')]

# ---- T3 : 자가 정리 -----------------------------------------------------------
# 투하 12.1s, anchor : 영상 17.8s = 로그 1786789293.13 (자가 정리 실시)
T3_SRC = 'T3_초록블록_자가_정리.webm'
T3_SUB = [
    (0.5, 11.0, '작업 구역이 비어 있다. 로봇은 아무 명령도 받지 않은 상태다'),
    (12.3, 17.3, '불량품(초록)이 작업 구역에 떨어진다 — 아무도 명령하지 않는다'),
    (17.8, 25.0, '2초 이상 머물면 방치로 판정 → 로봇이 스스로 명령을 만든다'),
    (25.5, 33.0, '자가 명령도 사람 명령과 똑같은 검증 4층을 지난다'),
    (42.0, 49.0, '파지 성공'),
    (55.0, 66.0, '초록의 목적지는 카운터가 아니라 수거함이다 — 물체별 라우팅'),
    (72.0, 79.5, '수거함에 놓기 완료'),
    (80.0, 87.0, "검증 통과 — '스스로 치움' 1건"),
    (88.0, 98.0, '사람이 시킨 일과 스스로 한 일이 다른 칸에 쌓인다'),
]
T3_HUD = [
    (0.0, 17.8, '상태  IDLE            서빙 1 · 정리 0'),
    (17.8, 80.5, '상태  RUNNING · 자가 명령   서빙 1 · 정리 0'),
    (80.5, 82.6, '상태  VERIFYING (씬 재확인)  서빙 1 · 정리 0'),
    (82.6, 104.0, '상태  IDLE            서빙 1 · 정리 1'),
]
T3_TITLE = [(0.5, 4.5, '② 명령 없이 스스로 치운다')]

# ---- T4 : 강탈 -> 복구 --------------------------------------------------------
# anchor : 영상 17.05s = 로그 1786789549.95
T4_SRC = 'T4_pick한물체_강탈이후_재개_확인.webm'
T4_SUB = [
    (0.5, 9.0, '같은 주문을 다시. 이번에는 운반 도중에 사고가 난다'),
    (17.5, 24.0, '사람 명령 : "빨간 블록을 카운터에 놔줘"'),
    (39.5, 45.0, '파지 성공 — 운반 시작'),
    (45.3, 52.0, '운반 중에 물체를 빼앗겼다 (실물에서는 낙하에 해당한다)'),
    (52.7, 59.0, 'GRIPPER_EMPTY 감지 → 전략 REGRASP · LLM이 고르고 검증기가 승인한다'),
    (59.5, 68.0, '복구 자세로 물러난다 — 팔이 시야를 가리면 다시 인지할 수 없다'),
    (70.0, 80.0, '재인지 : 좌표를 새로 읽는다. 옛 좌표로 다시 집지 않는다'),
    (82.0, 89.0, '다시 파지'),
    (112.0, 119.0, '카운터에 놓기 완료'),
    (119.5, 128.0, '포기 0건 — 실패했지만 사람은 아무것도 하지 않았다'),
]
T4_HUD = [
    (0.0, 17.05, '상태  IDLE            서빙 1 · 정리 1 · 포기 0'),
    (17.05, 52.7, '상태  RUNNING · 사람 명령   서빙 1 · 정리 1 · 포기 0'),
    (52.7, 58.2, '상태  RECOVERING · REGRASP 1/2   포기 0'),
    (58.2, 119.3, '상태  RUNNING · 복구 후 재개   서빙 1 · 정리 1'),
    (119.3, 121.4, '상태  VERIFYING (씬 재확인)'),
    (121.4, 130.4, '상태  IDLE            서빙 2 · 정리 1 · 포기 0'),
]
T4_TITLE = [(0.5, 4.5, '③ 실패를 스스로 수습한다')]

# ---- T5 : 예산 소진 -> 직원 호출 -> 재개 --------------------------------------
# 원본 610s. 같은 실패가 9번 반복되므로 가운데를 배속한다(원본은 그대로 남는다).
#   A 0~75   1배   투하 · 1/3 시작 · 첫 강탈 · REGRASP
#   B 75~445 8배   나머지 반복 (1/3 마무리 · 2/3 · 3/3)
#   C 445~540 1배  정리 불가 선언 · 무시 구간 · 재개
#   D 540~610 4배  재개 후 정상 완주
T5_SRC = 'T5.webm'
T5_SEG = [(0, 75, 1), (75, 445, 8), (445, 540, 1), (540, 610, 4)]


def t5_map(t):
    """원본 시각 -> 편집본 시각."""
    if t <= 75:
        return t
    if t <= 445:
        return 75 + (t - 75) / 8
    if t <= 540:
        return 121.25 + (t - 445)
    return 216.25 + (t - 540) / 4


T5_SUB = [
    (0.5, 5.0, '이번에는 실패가 계속된다. 로봇은 언제 포기해야 하는가'),
    (5.6, 12.0, '불량품 투하 — 자가 정리가 시작된다'),
    (14.5, 22.0, '자가 정리 1/3 — 물체마다 시도 예산이 있다'),
    (40.0, 48.0, '들어 올리는 순간마다 빼앗는다'),
    (57.3, 64.0, 'REGRASP 1/2 — 한 시퀀스 안의 복구 예산'),
    (76.0, 84.0, '▶ 여기부터 8배속 — 같은 실패가 아홉 번 반복된다'),
    (86.0, 95.0, '1/3 소진 → 2/3 시작. 시퀀스 예산이 하나 깎였다'),
    (104.5, 113.0, '2/3 소진 → 3/3 시작. 마지막 기회다'),
    (121.5, 128.0, '▶ 다시 실시간'),
    (129.2, 138.0, '정리 불가 : 3회 실패 → 사람 확인 요청'),
    (139.0, 149.0, '팔이 멈춘다. 상태는 ABORTED_WAIT — 사람이 올 때까지 기다린다'),
    (152.0, 165.0, '초록은 아직 작업 구역에 있다. 로봇은 그걸 알고도 손대지 않는다'),
    (166.0, 178.0, '무시 목록(ignored)에 올렸기 때문이다 — 못 봐서가 아니다'),
    (180.0, 195.0, '예산이 없으면 자율 작업은 같은 실패를 영원히 반복한다'),
    (200.8, 209.0, '사람이 "RESUME"을 보낸다 → 예산 원복, 무시 목록 비움'),
    (211.4, 220.0, '자가 정리 1/3 — 처음부터 다시 센다'),
    (222.0, 233.0, '포기 3건은 지워지지 않는다. 재개는 예산을 되돌릴 뿐이다'),
]
T5_HUD = [
    (0.0, 13.9, '상태  IDLE                서빙 2 · 정리 1 · 포기 0'),
    (13.9, 85.5, '상태  RUNNING/RECOVERING · 자가 정리 1/3     포기 0'),
    (85.5, 104.0, '상태  자가 정리 2/3            포기 1'),
    (104.0, 129.1, '상태  자가 정리 3/3            포기 2'),
    (129.1, 200.6, '상태  ABORTED_WAIT · 사람 확인 대기   포기 3 · ignored [green_block]'),
    (200.6, 211.2, '상태  RESUME 수신 → 복귀        포기 3 · ignored []'),
    (211.2, 233.8, '상태  RUNNING · 자가 정리 1/3      포기 3'),
]
T5_TITLE = [(0.5, 4.5, '④ 포기할 줄 알아야 자율이다')]

TAKES = [
    ('T1_주문_서빙', T1_SRC, CROP_A, W_A, H_A, T1_SUB, T1_HUD, T1_TITLE, None),
    ('T3_자가정리', T3_SRC, CROP_A, W_A, H_A, T3_SUB, T3_HUD, T3_TITLE, None),
    ('T4_강탈_복구', T4_SRC, CROP_A, W_A, H_A, T4_SUB, T4_HUD, T4_TITLE, None),
    ('T5_예산소진_직원호출', T5_SRC, CROP_T5, W_5, H_5, T5_SUB, T5_HUD, T5_TITLE, T5_SEG),
]


def main():
    os.makedirs(OUT, exist_ok=True)
    only = sys.argv[1] if len(sys.argv) > 1 else None
    for name, src, crop, w, h, subs, hud, titles, seg in TAKES:
        if only and only not in name:
            continue
        ass_path = os.path.join(OUT, name + '.ass')
        write_ass(ass_path, w, h, subs, hud, titles)
        dst = os.path.join(OUT, name + '.mp4')
        print(f'== {name} ==', flush=True)
        render(os.path.join(MEDIA, src), dst, crop, ass_path, seg)
        print(f'   -> {dst}', flush=True)


if __name__ == '__main__':
    main()
