#!/usr/bin/env python3
"""네 테이크를 2분 40초짜리 데모 한 편으로 합친다.

★ 개별 편집본(T1_*.mp4 등)을 이어붙이지 않고 원본에서 다시 만든다.
  합본은 배속이 들어가는데, 이미 구워진 자막을 배속하면 읽을 수가 없다.
  그래서 자막은 합본 타임라인에 맞춰 새로 쓴다. 원본은 그대로 남는다.

네 원본의 창 크기가 달라 크롭 시작점이 다르다. 결과는 전부 760x470으로 맞춘다.
"""
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from edit import FONT, MEDIA, OUT, line, ts, run  # noqa: E402

W, H = 760, 470

# (원본, 크롭, 시작, 끝, 배속) — 출력 길이 = (끝-시작)/배속
CLIPS = [
    ('T1_팔이_창고에서_빨간_블록을_집는다.webm', (760, 470, 110, 430), 22, 89, 2.6),
    ('T3_초록블록_자가_정리.webm', (760, 470, 110, 430), 10, 100, 3.4),
    ('T4_pick한물체_강탈이후_재개_확인.webm', (760, 470, 110, 430), 14, 130, 4.2),
    ('T5.webm', (760, 470, 105, 430), 8, 70, 3.0),
    ('T5.webm', (760, 470, 105, 430), 70, 450, 20.0),
    ('T5.webm', (760, 470, 105, 430), 450, 545, 3.5),
    ('T5.webm', (760, 470, 105, 430), 545, 610, 5.0),
]


def build():
    # 각 조각의 출력 시작 시각을 미리 계산한다(자막을 여기에 건다).
    starts, t = [], 0.0
    for _, _, a, b, sp in CLIPS:
        starts.append(t)
        t += (b - a) / sp
    return starts, t


STARTS, TOTAL = build()
C1, C2, C3, C4 = STARTS[0], STARTS[1], STARTS[2], STARTS[3]
C4b, C4c, C4d = STARTS[4], STARTS[5], STARTS[6]

TITLES = [
    (0.3, 4.5, 'Self-Recovering Service Cell'),
]
HUD = [
    (C1, C2, '① 주문을 받아 서빙한다'),
    (C2, C3, '② 명령 없이 스스로 치운다'),
    (C3, C4, '③ 실패를 스스로 수습한다'),
    (C4, TOTAL, '④ 포기할 줄 알아야 자율이다'),
]
SUB = [
    (0.4, 5.0, '명령 · 자가 판단 · 실패 복구가 모두 같은 계약을 지나는 로봇 셀'),
    (5.5, 11.0, '사람 명령 : "빨간 블록을 카운터에 놔줘" → LLM이 계약 어휘로 옮긴다'),
    (12.0, 18.0, '물체의 긴 축을 재서 그리퍼 방향을 정한다'),
    (20.0, 25.5, '놓고 나면 카메라로 다시 확인한다 — 액션의 성공 보고를 믿지 않는다'),

    (C2 + 1.0, C2 + 7.0, '불량품이 작업 구역에 떨어진다. 아무도 명령하지 않는다'),
    (C2 + 8.0, C2 + 14.0, '2초 이상 머물면 방치로 판정 → 로봇이 스스로 명령을 만든다'),
    (C2 + 15.0, C2 + 22.0, '자가 명령도 사람 명령과 똑같은 검증 4층을 지난다'),
    (C2 + 23.0, C2 + 29.0, '초록의 목적지는 카운터가 아니라 수거함이다'),

    (C3 + 1.0, C3 + 7.0, '같은 주문. 이번에는 운반 중에 물체를 빼앗긴다'),
    (C3 + 8.5, C3 + 15.0, 'GRIPPER_EMPTY 감지 → 전략 REGRASP'),
    (C3 + 16.0, C3 + 22.0, '복구 자세로 물러나 재인지 — 옛 좌표로 다시 집지 않는다'),
    (C3 + 23.0, C3 + 27.5, '완주. 사람은 아무것도 하지 않았다'),

    (C4 + 1.0, C4 + 7.0, '이번에는 실패가 계속된다. 로봇은 언제 포기해야 하는가'),
    (C4 + 8.0, C4 + 14.0, '들어 올릴 때마다 빼앗는다 — REGRASP는 시퀀스마다 2회까지'),
    (C4b + 1.0, C4b + 8.0, '▶ 20배속 — 같은 실패가 아홉 번 반복된다'),
    (C4b + 9.0, C4b + 16.0, '시도 예산 3회가 차례로 깎인다'),
    (C4c + 1.0, C4c + 8.0, '정리 불가 : 3회 실패 → 사람 확인 요청. 팔이 멈춘다'),
    (C4c + 9.0, C4c + 17.0, '초록은 아직 작업 구역에 있다. 알고도 손대지 않는다 — 무시 목록'),
    (C4c + 18.0, C4c + 25.0, '예산이 없으면 자율 작업은 같은 실패를 영원히 반복한다'),
    (C4d + 0.5, C4d + 6.0, '"RESUME" → 예산 원복. 포기 3건은 지워지지 않는다'),
    (C4d + 7.0, C4d + 12.5, '실패를 숨기지 않는 것이 이 셀의 계약이다'),
]


def main():
    ass_path = os.path.join(OUT, 'demo_v2.ass')
    body = f"""[Script Info]
ScriptType: v4.00+
PlayResX: {W}
PlayResY: {H}
WrapStyle: 2
ScaledBorderAndShadow: yes

[V4+ Styles]
Format: Name, Fontname, Fontsize, PrimaryColour, SecondaryColour, OutlineColour, BackColour, Bold, Italic, Underline, StrikeOut, ScaleX, ScaleY, Spacing, Angle, BorderStyle, Outline, Shadow, Alignment, MarginL, MarginR, MarginV, Encoding
Style: Sub,{FONT},24,&H00FFFFFF,&H000000FF,&H00000000,&H50000000,0,0,0,0,100,100,0,0,3,5,0,2,26,26,20,1
Style: Hud,{FONT},20,&H0000E5FF,&H000000FF,&H00000000,&H70000000,1,0,0,0,100,100,0,0,3,5,0,7,16,16,14,1
Style: Title,{FONT},36,&H00FFFFFF,&H000000FF,&H00000000,&H90000000,1,0,0,0,100,100,0,0,3,8,0,8,30,30,20,1

[Events]
Format: Layer, Start, End, Style, Name, MarginL, MarginR, MarginV, Effect, Text
"""
    for s, e, t in TITLES:
        body += line('Title', s, e, t)
    for s, e, t in HUD:
        body += line('Hud', s, e, t)
    for s, e, t in SUB:
        body += line('Sub', s, e, t)
    with open(ass_path, 'w', encoding='utf-8') as f:
        f.write(body)

    # 입력 파일은 3개(T1/T3/T4/T5)뿐이지만 T5는 네 번 쓰므로 인덱스를 모아둔다.
    files, idx = [], []
    for src, *_ in CLIPS:
        if src not in files:
            files.append(src)
        idx.append(files.index(src))

    cmd = ['ffmpeg', '-v', 'error']
    for f in files:
        cmd += ['-i', os.path.join(MEDIA, f)]

    parts, labels = [], []
    for i, (src, crop, a, b, sp) in enumerate(CLIPS):
        cw, ch, cx, cy = crop
        pad = '' if ch == H else f',pad={W}:{H}:0:{(H - ch) // 2}:black'
        parts.append(
            f'[{idx[i]}:v]trim={a}:{b},setpts=(PTS-STARTPTS)/{sp},'
            f'crop={cw}:{ch}:{cx}:{cy}{pad},fps=30,format=yuv420p[c{i}]')
        labels.append(f'[c{i}]')
    esc = ass_path.replace(':', r'\:')
    chain = ';'.join(parts) + ';' + ''.join(labels)
    chain += f'concat=n={len(CLIPS)}:v=1:a=0[cat];'
    chain += f"[cat]ass='{esc}'[out]"

    dst = os.path.join(OUT, 'demo_v2.mp4')
    print(f'합본 {TOTAL:.1f}초 -> {dst}', flush=True)
    for i, (src, _, a, b, sp) in enumerate(CLIPS):
        print(f'  {i}: {src[:18]:18s} {a:>4}~{b:<4} x{sp:<5} '
              f'-> {STARTS[i]:6.1f}s', flush=True)
    cmd += ['-filter_complex', chain, '-map', '[out]', '-an',
            '-c:v', 'libx264', '-preset', 'medium', '-crf', '20',
            '-pix_fmt', 'yuv420p', '-movflags', '+faststart', dst, '-y']
    run(cmd)
    print('완료', flush=True)


if __name__ == '__main__':
    main()
