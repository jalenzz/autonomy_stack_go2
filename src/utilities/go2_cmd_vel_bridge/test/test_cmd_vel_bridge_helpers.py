"""Minimal checks for L2+R2 edge detect and velocity clamp."""

from go2_cmd_vel_bridge.cmd_vel_bridge import (
    KEY_L2_R2,
    clamp,
    edge_pressed,
    l2_r2_pressed,
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
