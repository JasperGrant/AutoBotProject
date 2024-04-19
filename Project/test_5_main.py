import pytest
from math import pi


@pytest.mark.parametrize(
    "expected",
    [(-2), (-1), (0), (-3)],
)
def test_waypoint_follow(mocker, expected):
    mocker.patch("ev3dev2.motor.LargeMotor")
    mocker.patch("ev3dev2.motor.MediumMotor")
    mocker.patch("ev3dev2.sensor.lego.UltrasonicSensor")
    mocker.patch("ev3dev2.button.Button")
    mocker.patch("threading.Thread.start")
    mocker.patch("builtins.open")
    mocker.patch(
        "odometry.get_pose_past",
        side_effect=[(0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, pi)],
    )
    mocker.patch("avoid.get_goals_reached", return_value=0)
    mocker.patch("move.get_x_goal", return_value=[0, 0, 0])
    mocker.patch("move.get_y_goal", return_value=[0, 0, 0])
    mocker.patch("move.get_theta_goal", return_value=[0, 0, 0])
    mocker.patch("move.turn", return_value=-2)

    mocker.patch(
        "move.move_forward",
        side_effect=[
            -2,
            -1,
            0,
            0,
        ],
    )
    mocker.patch("move.get_theta_goal", return_value=[0, 0, 0, 0])
    from main import waypoint_follow

    assert waypoint_follow() == expected


def test_waypoint_follow_priority(mocker):
    mocker.patch("ev3dev2.motor.LargeMotor")
    mocker.patch("ev3dev2.motor.MediumMotor")
    mocker.patch("ev3dev2.sensor.lego.UltrasonicSensor")
    mocker.patch("ev3dev2.button.Button")
    mocker.patch("threading.Thread.start")
    mocker.patch("builtins.open")
    from main import waypoint_follow_priority, WAYPOINT_FOLLOW_CONSTANT_PRIORITY

    assert waypoint_follow_priority()[1] == WAYPOINT_FOLLOW_CONSTANT_PRIORITY


def test_scan_priority(mocker):
    mocker.patch("ev3dev2.motor.LargeMotor")
    mocker.patch("ev3dev2.motor.MediumMotor")
    mocker.patch("ev3dev2.sensor.lego.UltrasonicSensor")
    mocker.patch("ev3dev2.button.Button")
    mocker.patch("threading.Thread.start")
    mocker.patch("builtins.open")
    mocker.patch("main.get_distance_since_last_scan", return_value=1)
    from main import scan_priority, SCAN_SLOPE_PRIORITY

    assert scan_priority()[1] == SCAN_SLOPE_PRIORITY


def test_obstacle_avoid_priority(mocker):
    mocker.patch("ev3dev2.motor.LargeMotor")
    mocker.patch("ev3dev2.motor.MediumMotor")
    mocker.patch("ev3dev2.sensor.lego.UltrasonicSensor")
    mocker.patch("ev3dev2.button.Button")
    mocker.patch("threading.Thread.start")
    mocker.patch("builtins.open")
    mocker.patch("main.get_distance_since_last_scan", return_value=1)
    mocker.patch("detect.get_avoidance_in_progress", return_value=True)
    from main import obstacle_avoid_priority, OBSTACLE_NOT_DETECTED_CONSTANT_PRIORITY

    assert obstacle_avoid_priority()[1] == OBSTACLE_NOT_DETECTED_CONSTANT_PRIORITY
