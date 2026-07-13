"""Minimal checks for L2+R2 edge detect, clamp, and stick override helpers."""

from go2_cmd_vel_bridge.cmd_vel_bridge import (
    KEY_L2_R2,
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


def test_sticks_active_and_velocity():
    assert not sticks_active(0.05, 0.0, 0.0, 0.1)
    assert sticks_active(0.2, 0.0, 0.0, 0.1)
    vx, vy, wz = sticks_to_velocity(0.5, 1.0, -0.5, 0.5, 0.3, 1.0)
    assert vx == 0.5
    assert vy == 0.15
    assert wz == -0.5
