#!/usr/bin/env python
from common import RosSubscriber
from sensor_msgs.msg import Joy

class JoystickSubscriber(RosSubscriber):
    def _setup(self):
        self.msg = Joy

    def _make_dic(self, msg):
        dict = {}
        for i in range(len(msg.axes)):
            n = "axes"+str(i)
            v = msg.axes[i]
            dict[n] = v
        for i in range(len(msg.buttons)):
            n = "buttons"+str(i)
            v = msg.buttons[i]
            dict[n] = v
        return dict

def setup(robot):
    robot.add_subscriber(JoystickSubscriber(u"joy"))
