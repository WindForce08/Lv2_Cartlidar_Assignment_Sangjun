import math

from perception_survey.face_yaw import face_yaw, face_yaw_error

D = math.pi / 180


def quat(yaw, roll=0.0):
    cy, sy, cr, sr = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(roll / 2), math.sin(roll / 2)
    return cy * sr, sy * sr, sy * cr, cy * cr


def test_fold_quarter_turn():
    for deg, want in ((0, 0), (17, 17), (44, 44), (46, -44), (90, 0), (135, -45), (-46, 44), (180, 0)):
        y = face_yaw(*quat(deg * D))
        assert abs(y - want * D) < 1e-9, deg
        assert -math.pi / 4 <= y < math.pi / 4


def test_tilt_and_lying():
    assert abs(face_yaw(*quat(17 * D, 3 * D)) - 17 * D) < 1e-9
    s = math.sqrt(0.5)
    assert face_yaw(0, s, 0, s) is None


def test_error_against_reference():
    assert abs(face_yaw_error(19.06 * D, 17.06 * D) - 2 * D) < 1e-12
    assert abs(face_yaw_error(-44 * D, 44 * D) - 2 * D) < 1e-12
