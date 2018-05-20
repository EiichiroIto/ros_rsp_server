#!/usr/bin/env python
import rospy

def float_from_string(str):
    try:
        r = float(str)
    except ValueError:
        r = 0.0
    return r

class RosPublisher(object):
    def __init__(self, name):
        self.name = name
        self._setup()

    def setup_publisher(self, ros):
        self.publisher = ros.Publisher(self.name, self.msg, queue_size=1)

class RosSubscriber(object):
    def __init__(self, name):
        self.name = name
        self._setup()

    def setup_subscriber(self, ros, rsp_server):
        self.subscriber = ros.Subscriber(self.name, self.msg, self.callback)
        self.rsp_server = rsp_server

    def callback(self, msg):
        dic = self._make_dic(msg)
        self.rsp_server.sensor_update(dic)
        self.rsp_server.broadcast(self.name)

    def _make_dic(self, msg):
        return {}

