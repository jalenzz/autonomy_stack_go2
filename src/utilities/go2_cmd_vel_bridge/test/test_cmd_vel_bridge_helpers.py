"""Minimal checks for L2+R2 edge detect, velocity clamp, and stick override helpers."""

from go2_cmd_vel_bridge.cmd_vel_bridge import (
    KEY_L2_R2,
    STICK_DEADZONE,
    apply_deadzone,
    clamp,
    edge_pressed,
    l2_r2_pressed,
    sticks_active,
    sticks_to_velocity,
)


def test_l2_r2_and_edge():
    assert l2_r2_pressed(KEY_L2_R2)
    assert not l2_r2_pressed(0)
    assert edge_pressed(True, False)
    assert not edge_pressed(True, True)
    assert not edge_pressed(False, True)


def test_clamp():
    assert clamp(1.0, 0.5) == 0.5
    assert clamp(-1.0, 0.5) == -0.5
    assert clamp(0.2, 0.5) == 0.2


def test_apply_deadzone():
    assert apply_deadzone(0.05, 0.1) == 0.0
    assert apply_deadzone(-0.05, 0.1) == 0.0
    assert apply_deadzone(0.2, 0.1) == 0.2
    assert apply_deadzone(-0.2, 0.1) == -0.2


def test_sticks_active_and_velocity():
    assert not sticks_active(0.05, 0.0, 0.0, STICK_DEADZONE)
    assert sticks_active(0.2, 0.0, 0.0, STICK_DEADZONE)

    vx, vy, wz = sticks_to_velocity(0.5, 1.0, -0.8, STICK_DEADZONE)
    assert vx == 1.0
    assert vy == -0.5
    assert wz == 0.8

    vx, vy, wz = sticks_to_velocity(0.05, 0.05, 0.05, STICK_DEADZONE)
    assert vx == 0.0
    assert vy == 0.0
    assert wz == 0.0
