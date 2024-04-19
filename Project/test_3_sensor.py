import pytest

from math import sqrt


@pytest.mark.parametrize(
    "distance,angle,robot_pose,expected",
    [
        (10, 0, [0, 0, 0], [10, 0]),
        (10, 90, [0, 0, 0], [0, 10]),
        (10, 180, [0, 0, 0], [-10, 0]),
        (10, 270, [0, 0, 0], [0, -10]),
        (10, -180, [10, 10, 0], [0, 10]),
        (10, -225, [0, 0, 0], [-7.07, 7.07]),
        (10, -45, [0, 0, 0], [7.07, -7.07]),
    ],
)
def test_transform_distance_to_coordinate(
    mocker, distance, angle, robot_pose, expected
):

    mocker.patch("ev3dev2.motor.MediumMotor")
    mocker.patch("ev3dev2.motor.LargeMotor")
    mocker.patch("ev3dev2.sensor.lego.UltrasonicSensor")
    from sensor import transform_distance_to_coordinate

    transformed_pose = transform_distance_to_coordinate(distance, angle, robot_pose)
    assert (
        sqrt(
            (transformed_pose[0] - expected[0]) ** 2
            + (transformed_pose[1] - expected[1]) ** 2
        )
        < 0.01
    )
