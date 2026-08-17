from .llm_contract import ERROR_NAMES, MAX_STEPS, POSE_IDS, TARGET_IDS


def _build_prompt(command, scene_ids):
    """도구 스키마와 씬 정보를 담은 프롬프트를 만든다.

    지시문은 영어다(모델이 더 잘 따른다). 명령 자체는 한국어여도 된다.
    문법으로 형식을 강제해도 모델은 그 문법의 존재를 모르므로,
    스키마를 프롬프트에도 문자열로 적어준다(Ollama 공식 권고).
    """
    scene = ', '.join(scene_ids) if scene_ids else '(unknown)'
    return (
        'You output ONLY a JSON array of steps. No prose, no explanation.\n'
        '\n'
        'Available skills (use EXACTLY these names and fields):\n'
        '  {"skill": "pick",    "object_id": "<object>"}\n'
        '  {"skill": "place",   "object_id": "<object>", "target_id": "<target>"}\n'
        '  {"skill": "move_to", "pose_id": "<name>"}\n'
        '\n'
        f'Valid pose_id values: {", ".join(POSE_IDS)}\n'
        f'Valid target_id values: {", ".join(TARGET_IDS)}\n'
        f'Objects present in the scene: {scene}\n'
        '\n'
        'Rules:\n'
        f'- Output at most {MAX_STEPS} steps.\n'
        '- Do NOT add steps that were not requested.\n'
        '  In particular, never insert move_to before pick or place.\n'
        '- move_to may only appear as a single step on its own.\n'
        '- Use only object_id values from the scene list above.\n'
        # 실측 실패(2026-08-07): "빨간 블록 집어" -> object_id를 명령의 한국어 단어
        # 그대로 복사해 검증에서 거부됐다. 같은 명령이 red_block으로 나온 적도 있어
        # 비결정적이다. 번역이 필요하다는 것을 규칙과 예시로 명시한다.
        '- The command may name objects in Korean. Translate them to the EXACT\n'
        '  English id from the scene list. Example: "빨간 블록" -> "red_block".\n'
        '- NEVER copy a word from the command into object_id. Always pick an id\n'
        '  that appears verbatim in the scene list above.\n'
        # 거부 경로. 2026-08-15에 열었다 - _is_refusal이 검증기 앞에서 []를 가려내므로
        # 이제 재프롬프트로 밀려나지 않는다(그 전에는 넣으면 안 되는 줄이었다).
        # 이 줄이 없으면 모델은 못 하는 명령에도 억지로 씬 목록에서 아무거나 골라
        # 낸다 - 조용히 틀린 일을 하는 것이 아무것도 안 하는 것보다 나쁘다.
        # 거부 조건을 "할 수 없으면"으로 넓게 적었더니 모델이 멀쩡한 명령까지 거부했다
        # (2026-08-16 실측 : 오거부 20건. 'red_block 집어' 3/3 거부, 'RED_BLOCK 집어',
        #  'red_block 집어, 고마워', '홈으로 가' 전부 []. 영어 'pick red_block'은 통과).
        # 짧은 말투·존댓말·대문자·인사말을 "못 하는 일"로 읽은 것이다.
        # -> 거부 조건을 닫힌 목록으로 좁히고, 반대로 해야 하는 경우를 못박는다.
        '- Output [] (empty array, nothing else) ONLY in these cases:\n'
        '    (a) the object named is not in the scene list above,\n'
        '    (b) the target named is not in the target_id list above,\n'
        '    (c) the pose named is not in the pose_id list above,\n'
        '    (d) the action is not one of the three skills above.\n'
        '- Otherwise you MUST output a plan. If the object is in the scene list,\n'
        '  [] is always the wrong answer. Short, casual, polite, uppercase or\n'
        '  punctuated commands are still valid commands - just plan them.\n'
        '- A pick needs no target. "pick X" alone is a complete, valid command.\n'
        '- NEVER substitute a different object or target to make it work.\n'
        '  Asking for something absent is a refusal, not a naming mistake.\n'
        '\n'
        f'Command: {command}'
    )


def _retry_prompt(prompt, raw, error):
    """1차 실패를 이유와 함께 되돌려주는 재프롬프트.

    실측 실패(2026-08-07): 1차가 '씬에 없는 물체'로 거부되자 2차가 []를 냈다.
    거부당했다는 사실만 보고 "할 수 있는 게 없다"로 후퇴한 것이다.
    그래서 의도는 유지하고 틀린 필드만 고치라고 못박는다.

    ★ 2026-08-15 정정. 위 처방으로 「[]를 내지 마라, 거절된 object_id는 씬 목록에서
      가장 비슷한 것으로 갈아끼워라」를 넣었었다. 그때는 []가 그냥 실패였으니 맞는
      말이었는데, []에 '정당한 거부'라는 뜻이 생기면서 전제가 사라졌다. 그 지시가
      남아 있는 동안 실제로 사고가 났다 - 초록 블록을 시켰는데 초록이 그 순간 씬에서
      빠져 1차가 반려되자, 2차가 시킨 대로 red_block으로 갈아끼워 완주하고 served까지
      올렸다(실측). 모델이 틀린 게 아니라 시킨 대로 한 것이다.
      이제는 반대로 못하겠으면 []를 내라고 말한다. 2차의 []도 1차와 똑같이 거부로
      받는다 - 두 시도를 다르게 대할 이유가 없다.
    """
    return (
        f'{prompt}\n'
        '\n'
        'Your previous answer was REJECTED.\n'
        f'Previous answer: {raw}\n'
        f'Reason: {error}\n'
        'Fix ONLY the field that was wrong and keep the original intent.\n'
        'If the object_id you used is simply not in the scene list, then the\n'
        'command cannot be done: output an empty array [] and nothing else.\n'
        'That is the correct answer, not a failure.\n'
        'NEVER swap in a different object or target to make the command work -\n'
        'moving the wrong object is worse than doing nothing.\n'
        'Return only a corrected JSON array.'
    )


def _build_recovery_prompt(code, stage, detail, attempt, recovery_used, max_recovery):
    """실패 보고를 모델이 읽을 수 있는 형태로 옮긴다.

    스키마를 문법으로 강제해도 모델은 문법의 존재를 모른다.
    그래서 출력 형식을 프롬프트에도 문자열로 적는다(Ollama 공식 권고).
    """
    name = ERROR_NAMES.get(code, f'UNKNOWN({code})')
    return (
        'You choose ONE recovery strategy for a robot arm that failed a skill.\n'
        '\n'
        'Strategies:\n'
        '- RETRY   : send the same goal again. Useful only when the failure is random.\n'
        '            Motion planning is sampling based, so a new seed may solve it.\n'
        '- REGRASP : back off to a safe pose, look again, then redo the failed step.\n'
        '            Use when the gripper missed the object or dropped it.\n'
        '- RESCAN  : same motion as REGRASP. Use when the object was not found.\n'
        '- ABORT   : stop the whole command. Use when retrying cannot help.\n'
        '\n'
        'Failure report:\n'
        f'- error          : {name}\n'
        f'- stage          : {stage}\n'
        f'- detail         : {detail}\n'
        f'- attempt        : {attempt}\n'
        f'- recovery used  : {recovery_used} of {max_recovery}\n'
        '\n'
        'Rules:\n'
        '- Answer with one JSON object only: {"strategy": "<ONE OF THE ABOVE>"}\n'
        '- If the arm physically cannot reach the target, retrying changes nothing.\n'
        '- If you are unsure, choose ABORT. Stopping is always safe.\n'
    )
