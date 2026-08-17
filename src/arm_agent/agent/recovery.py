from arm_interfaces.msg import ErrorCode

# 복구 전략
# 전략은 goal 파라미터가 아니라 시퀀스 조작이다.
# REGRASP/RESCAN : 복구 자세로 물러나 재인지한 뒤 실패한 스텝을 다시 밟는다.
RETRY = 'RETRY'
REGRASP = 'REGRASP'
RESCAN = 'RESCAN'
ABORT = 'ABORT'
# 에러 코드 -> 전략. 등록 안된 코드는 기본 abort(멈추는게 가장 안전)
STRATEGY = {
    ErrorCode.PLANNING_FAILED: RETRY,
    ErrorCode.EXECUTION_TIMEOUT: RETRY,
    ErrorCode.GRASP_FAILED: REGRASP,
    ErrorCode.OBJECT_MOVED: RESCAN,
    ErrorCode.OBJECT_NOT_FOUND: RESCAN,
    ErrorCode.GRIPPER_EMPTY: REGRASP,
    ErrorCode.UNREACHABLE: ABORT,
    ErrorCode.UNDEFINED_POSE: ABORT,
    ErrorCode.INTERNAL_ERROR: ABORT,
}

MAX_ATTEMPTS = 2
MAX_RECOVERY = 1
