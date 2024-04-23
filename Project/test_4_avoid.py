import pytest


@pytest.mark.parametrize(
    "pose_past,goals_reached,expected",
    [
        ([0, 0, 0], 0, 45),
        ([0, 0, 0], 1, 0),
        ([0, 0, 0], 2, 90),
        ([0, 0, 0], 3, 135),
        ([0, 0, 0], 4, 180),
        ([0, 0, 0], 5, -135),
        ([0, 0, 0], 6, -90),
    ],
)
def test_get_goal_angle(pose_past, goals_reached, expected, mocker):
    mocker.patch("ev3dev2.motor.LargeMotor")
    mocker.patch("ev3dev2.motor.MediumMotor")
    mocker.patch("ev3dev2.sensor.lego.UltrasonicSensor")
    mocker.patch("ev3dev2.button.Button")
    mocker.patch("threading.Thread.start")
    mocker.patch("builtins.open")
    mocker.patch(
        "move.get_x_goal",
        return_value=[1, 1, 0, -1, -1, -1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    )
    mocker.patch(
        "move.get_y_goal",
        return_value=[1, 0, 1, 1, 0, -1, -1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    )
    from goals import get_goal_angle

    assert get_goal_angle(pose_past, goals_reached) == expected


@pytest.mark.parametrize(
    "pose_past,goals_reached, expected",
    [
        ([0, 2, 0], 0, 1),
        ([0, 0, 0], 1, None),
        ([0, 5, 0], 2, 2),
        ([0, 5, 0], 4, 3),
        ([5, 5, 0], 6, 4),
        ([0, 0, 0], 7, None),
        ([0, 0, 0], 17, 0),
    ],
)
def test_check_distance_to_goal(pose_past, goals_reached, expected, mocker):
    mocker.patch("ev3dev2.motor.LargeMotor")
    mocker.patch("ev3dev2.motor.MediumMotor")
    mocker.patch("ev3dev2.sensor.lego.UltrasonicSensor")
    mocker.patch("ev3dev2.button.Button")
    mocker.patch("threading.Thread.start")
    mocker.patch("builtins.open")
    from goals import check_distance_to_goal

    assert check_distance_to_goal(pose_past, goals_reached) == expected
