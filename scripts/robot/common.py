#!/usr/bin/env python
import rospy

def float_from_string(str):
    try:
        r = float(str)
    except ValueError:
        r = 0.0
    return r

class RosRobot(object):
    def __init__(self):
        self.pub_topics = {}
        self.variables = {}
        self.sub_topics = {}

    def add_publisher(self, name, obj):
        self.pub_topics[name] = obj

    def add_subscriber(self, name, obj):
        self.sub_topics[name] = obj

    def add_variable(self, name, val):
        self.variables[name] = val

class RosPublisher(object):
    def __init__(self, name):
        self.name = name
        self._setup()

    def setup_publisher(self, ros, robot):
        self.robot = robot
        self.publisher = ros.Publisher(self.name, self.msg, queue_size=1)

class RosSubscriber(object):
    def __init__(self, name):
        self.name = name
        self._setup()

    def setup_subscriber(self, ros, robot, rsp_server):
        self.robot = robot
        self.subscriber = ros.Subscriber(self.name, self.msg, self.callback)
        self.rsp_server = rsp_server

    def callback(self, msg):
        dic = self._make_dic(msg)
        self.rsp_server.sensor_update(dic)
        self.rsp_server.broadcast(self.name)

    def _make_dic(self, msg):
        return {}

