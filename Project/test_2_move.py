import pytest


def test_turn_object_detected(mocker):
    mocker.patch("ev3dev2.motor.LargeMotor")
    mocker.patch("ev3dev2.motor.MediumMotor")
    mocker.patch("ev3dev2.sensor.lego.UltrasonicSensor")
    mocker.patch("ev3dev2.button.Button")
    mocker.patch("threading.Thread.start")
    mocker.patch("builtins.open")
    mocker.patch(
        "detect.get_avoidance_in_progress",
        side_effect=[True, True] + [False] * 10,
    )
    mocker.patch("move.velocity_controller")
    from move import turn

    assert turn(0, 0, 0) == -2


def test_move_forward_goal_reached(mocker):
    mocker.patch("ev3dev2.motor.LargeMotor")
    mocker.patch("ev3dev2.motor.MediumMotor")
    mocker.patch("ev3dev2.sensor.lego.UltrasonicSensor")
    mocker.patch("ev3dev2.button.Button")
    mocker.patch("threading.Thread.start")
    mocker.patch("builtins.open")
    mocker.patch("move.velocity_controller")
    mocker.patch("move.get_pose_past", return_value=[0, 0, 0])
    from move import move_forward

    assert move_forward(0, 0, 0, 0, 0) == 0


def test_move_forward_goal_within_threshold(mocker):
    mocker.patch("ev3dev2.motor.LargeMotor")
    mocker.patch("ev3dev2.motor.MediumMotor")
    mocker.patch("ev3dev2.sensor.lego.UltrasonicSensor")
    mocker.patch("ev3dev2.button.Button")
    mocker.patch("threading.Thread.start")
    mocker.patch("builtins.open")
    mocker.patch("move.velocity_controller")
    mocker.patch("move.get_pose_past", return_value=[2, 3, 0])
    from move import move_forward

    assert move_forward(0, 0, 0, 0, 0) == 0


def test_move_forward_object_detected(mocker):
    mocker.patch("ev3dev2.motor.LargeMotor")
    mocker.patch("ev3dev2.motor.MediumMotor")
    mocker.patch("ev3dev2.sensor.lego.UltrasonicSensor")
    mocker.patch("ev3dev2.button.Button")
    mocker.patch("threading.Thread.start")
    mocker.patch("builtins.open")
    mocker.patch("move.velocity_controller")
    from move import move_forward

    assert move_forward(0, 0, 0, 0, 0) == -2


def test_move_forward_goal_given_up_on(mocker):
    mocker.patch("ev3dev2.motor.LargeMotor")
    mocker.patch("ev3dev2.motor.MediumMotor")
    mocker.patch("ev3dev2.sensor.lego.UltrasonicSensor")
    mocker.patch("ev3dev2.button.Button")
    mocker.patch("threading.Thread.start")
    mocker.patch("builtins.open")
    mocker.patch("move.velocity_controller")
    mocker.patch(
        "move.get_pose_past",
        side_effect=[[i, i, i] for i in range(5, 200, 5)],
    )
    from move import move_forward

    assert move_forward(0, 0, 0, 0, 0) == -1
