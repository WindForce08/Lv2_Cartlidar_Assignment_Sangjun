from perception_survey.localization import accept_by_localization


def test_ok_and_unknown_accept():
    assert accept_by_localization('OK', 0.1, 1.0) == (True, None)
    assert accept_by_localization(None, 0.0, 1.0) == (True, None)          # 상태 노드 없음
    assert accept_by_localization('LOST', 1.5, 1.0) == (True, None)        # 1 s 넘게 안 옴 = 모름


def test_not_ok_rejects():
    for s in ('SUSPECT', 'LOST', 'RECOVERING', 'CONFIRM', 'FAILED'):
        ok, why = accept_by_localization(s, 0.2, 1.0)
        assert not ok and why == 'localization_' + s.lower()


def test_nan_age_is_unknown():
    assert accept_by_localization('LOST', float('nan'), 1.0) == (True, None)
