"""Minimal Unitree Go2 Sport API publisher for ROS2 Foxy."""

import json

from unitree_api.msg import Request

ROBOT_SPORT_API_ID_MOVE = 1008
ROBOT_SPORT_API_ID_SWITCHJOYSTICK = 1027
ROBOT_SPORT_API_ID_FREEWALK = 2045
ROBOT_SPORT_API_ID_CLASSICWALK = 2049


class SportClient:
    def __init__(self, publisher):
        self._pub = publisher
        self._req_id = 0

    def _publish(self, api_id, parameter=''):
        req = Request()
        self._req_id += 1
        req.header.identity.id = self._req_id
        req.header.identity.api_id = api_id
        req.header.policy.priority = 0
        req.header.policy.noreply = True
        req.parameter = parameter
        self._pub.publish(req)

    def move(self, vx, vy, vyaw):
        self._publish(
            ROBOT_SPORT_API_ID_MOVE,
            json.dumps({'x': float(vx), 'y': float(vy), 'z': float(vyaw)}),
        )

    def switch_joystick(self, enabled):
        self._publish(
            ROBOT_SPORT_API_ID_SWITCHJOYSTICK,
            json.dumps({'data': bool(enabled)}),
        )

    def free_walk(self):
        self._publish(ROBOT_SPORT_API_ID_FREEWALK)

    def classic_walk(self, enabled=True):
        self._publish(
            ROBOT_SPORT_API_ID_CLASSICWALK,
            json.dumps({'data': bool(enabled)}),
        )
