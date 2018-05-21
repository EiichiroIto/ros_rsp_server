#!/usr/bin/env python
from common import float_from_string, RosPublisher, RosSubscriber
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent, CliffEvent

class CmdVelPublisher(RosPublisher):
    def _setup(self):
        self.msg = Twist

    def make_twist_value(self):
        r = Twist()
        r.linear.x = float_from_string(self.robot.variables[u"lx"])
        r.linear.y = float_from_string(self.robot.variables[u"ly"])
        r.linear.z = float_from_string(self.robot.variables[u"lz"])
        r.angular.x = float_from_string(self.robot.variables[u"ax"])
        r.angular.y = float_from_string(self.robot.variables[u"ay"])
        r.angular.z = float_from_string(self.robot.variables[u"az"])
        return r

    def publish(self):
        self.publisher.publish(self.make_twist_value())

class BumperSubscriber(RosSubscriber):
    def _setup(self):
        self.msg = BumperEvent

    def _make_dic(self, msg):
        n = "bumper"+str(msg.bumper)
        v = msg.state
        return {n: v}

class CliffSubscriber(RosSubscriber):
    def _setup(self):
        self.msg = CliffEvent

    def _make_dic(self, msg):
        n1 = "cliff"+str(msg.sensor)
        v1 = msg.state
        n2 = "bottom"
        v2 = msg.bottom
        return {n1: v1, n2: v2}

class TurtleBot(object):
    def __init__(self):
        self.pub_topics = {u"cmd_vel": CmdVelPublisher("cmd_vel")}
        self.variables = {u"lx": "0", u"ly": "0", u"lz": "0",
                          u"ax": "0", u"ay": "0", u"az": "0"}
        self.sub_topics = {u"bumper": BumperSubscriber("bumper"),
                           u"cliff": CliffSubscriber("cliff")}
