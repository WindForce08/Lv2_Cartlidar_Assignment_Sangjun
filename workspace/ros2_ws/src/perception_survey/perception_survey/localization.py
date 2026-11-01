"""위치 추정 상태(/localization/status, std_msgs/String)로 측량 관측을 받을지 정한다. ROS 의존 없음 → 단위 시험.
상태 값: OK · SUSPECT · LOST · RECOVERING · CONFIRM · FAILED(재위치 노드 loc_monitor). OK 일 때만 받는다.
상태를 한 번도 못 받았거나 stale_s 넘게 새 값이 없으면 '모름'으로 보고 받는다(재위치 노드 없이 띄운 모의 · 시험 실행 호환)."""

OK = 'OK'


def accept_by_localization(state, age_s, stale_s):
    """state: 마지막 상태(None = 못 받음), age_s: 받은 뒤 지난 시간[s]. 돌려줌: (받음 여부, 거부 이유 | None)"""
    if state is None or not (age_s <= stale_s):
        return True, None
    if state == OK:
        return True, None
    return False, 'localization_' + str(state).lower()
