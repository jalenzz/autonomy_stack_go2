"""
Small dependency-free rigid-transform helpers.

Transforms use translation tuples and quaternions in ROS ``x, y, z, w``
order.  ``compose(T_ab, T_bc)`` returns ``T_ac``.
"""

import math


def normalize_quaternion(quaternion):
    """Return a normalized quaternion and reject invalid inputs."""
    if len(quaternion) != 4:
        raise ValueError('quaternion must contain exactly four values')
    if not all(math.isfinite(value) for value in quaternion):
        raise ValueError('quaternion contains a non-finite value')
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm < 1.0e-12:
        raise ValueError('quaternion norm is zero')
    return tuple(value / norm for value in quaternion)


def validate_translation(translation):
    """Return a finite three-element translation tuple."""
    if len(translation) != 3:
        raise ValueError('translation must contain exactly three values')
    if not all(math.isfinite(value) for value in translation):
        raise ValueError('translation contains a non-finite value')
    return tuple(float(value) for value in translation)


def quaternion_multiply(left, right):
    """Multiply two normalized ROS-order quaternions."""
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return normalize_quaternion((
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ))


def rotate_vector(quaternion, vector):
    """Rotate a three-element vector by a normalized quaternion."""
    qx, qy, qz, qw = normalize_quaternion(quaternion)
    vx, vy, vz = validate_translation(vector)

    # Equivalent to q * (v, 0) * conjugate(q), without normalizing the
    # intermediate pure-vector quaternion.
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


def compose(first_translation, first_rotation,
            second_translation, second_rotation):
    """Compose ``T_ab`` and ``T_bc`` to produce ``T_ac``."""
    first_translation = validate_translation(first_translation)
    first_rotation = normalize_quaternion(first_rotation)
    second_translation = validate_translation(second_translation)
    second_rotation = normalize_quaternion(second_rotation)
    rotated = rotate_vector(first_rotation, second_translation)
    translation = tuple(
        first_translation[index] + rotated[index] for index in range(3)
    )
    rotation = quaternion_multiply(first_rotation, second_rotation)
    return translation, rotation


def invert(translation, rotation):
    """Invert a rigid transform."""
    translation = validate_translation(translation)
    qx, qy, qz, qw = normalize_quaternion(rotation)
    inverse_rotation = (-qx, -qy, -qz, qw)
    inverse_translation = rotate_vector(
        inverse_rotation,
        tuple(-value for value in translation),
    )
    return inverse_translation, inverse_rotation
