import math

import pytest

from fast_lio_vehicle_adapter.transform_math import compose, invert
from fast_lio_vehicle_adapter.transform_math import normalize_quaternion


def assert_vector_close(actual, expected):
    assert actual == pytest.approx(expected, abs=1.0e-9)


def test_inverse_round_trip():
    translation = (0.31, -0.02, 0.18)
    rotation = normalize_quaternion((0.0, 0.0, 0.3826834324, 0.9238795325))
    inverse_translation, inverse_rotation = invert(translation, rotation)

    result_translation, result_rotation = compose(
        translation, rotation, inverse_translation, inverse_rotation)

    assert_vector_close(result_translation, (0.0, 0.0, 0.0))
    assert_vector_close(result_rotation, (0.0, 0.0, 0.0, 1.0))


def test_vehicle_pose_removes_rotating_sensor_lever_arm():
    # The MID360 IMU is 0.3 m in front of vehicle center. After the robot
    # rotates 90 degrees around vehicle center, the raw IMU pose is at +Y.
    vehicle_to_body_translation = (0.3, 0.0, 0.0)
    vehicle_to_body_rotation = (0.0, 0.0, 0.0, 1.0)
    body_to_vehicle = invert(
        vehicle_to_body_translation, vehicle_to_body_rotation)
    world_to_body_rotation = normalize_quaternion((
        0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0)))

    vehicle_translation, vehicle_rotation = compose(
        (0.0, 0.3, 0.0),
        world_to_body_rotation,
        body_to_vehicle[0],
        body_to_vehicle[1],
    )

    assert_vector_close(vehicle_translation, (0.0, 0.0, 0.0))
    assert_vector_close(vehicle_rotation, world_to_body_rotation)


def test_mounting_rotation_is_removed_from_vehicle_orientation():
    quarter_turn = normalize_quaternion((
        0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0)))
    body_to_vehicle = invert((0.0, 0.0, 0.0), quarter_turn)

    _, vehicle_rotation = compose(
        (0.0, 0.0, 0.0),
        quarter_turn,
        body_to_vehicle[0],
        body_to_vehicle[1],
    )

    assert_vector_close(vehicle_rotation, (0.0, 0.0, 0.0, 1.0))


def test_invalid_quaternion_is_rejected():
    with pytest.raises(ValueError, match='norm is zero'):
        normalize_quaternion((0.0, 0.0, 0.0, 0.0))
