#!/usr/bin/env python
import rospy
from rsp_server.rsserver import RemoteSensorServer
from geometry_msgs.msg import Twist

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
        self.publisher = ros.Publisher(self. name, self.msg, queue_size=1)

class CmdVelPublisher(RosPublisher):
    def _setup(self):
        self.msg = Twist

    def make_twist_value(self):
        r = Twist()
        r.linear.x = float_from_string(variables[u"lx"])
        r.linear.y = float_from_string(variables[u"ly"])
        r.linear.z = float_from_string(variables[u"lz"])
        r.angular.x = float_from_string(variables[u"ax"])
        r.angular.y = float_from_string(variables[u"ay"])
        r.angular.z = float_from_string(variables[u"az"])
        return r

    def publish(self):
        self.publisher.publish(self.make_twist_value())

pub_topics = {u"cmd_vel": CmdVelPublisher("cmd_vel")}
variables = {u"lx": "0", u"ly": "0", u"lz": "0",
             u"ax": "0", u"ay": "0", u"az": "0"}

class RosRspServer(object):
    def __init__(self, server=None):
        if server is not None:
            server.set_controller(self)
        self.setup_publishers()
        server.start()

    def setup_publishers(self):
        global pub_topics
        self.publishers = {}
        for topic in pub_topics:
            pub = pub_topics[topic]
            pub.setup_publisher(rospy)

    def sensor_update(self, list):
        global variables
        for (n, v) in list:
            if n in variables:
                variables[n] = v
                print("%s=%s" % (n,v))
            else:
                rospy.logwarn("unknown variable: %s" % n)

    def broadcast(self, str):
        if str in pub_topics:
            pub = pub_topics[str]
            pub.publish()
        else:
            rospy.logwarn("unknown topic: %s" % str)

if __name__ == '__main__':
    server = RemoteSensorServer()
    rrs = RosRspServer(server=server)
    rospy.init_node('ros_rsp_server')
    rospy.spin()
#    rate = rospy.Rate(10)
#    while not rospy.is_shutdown():
#        rate.sleep()
    server.stop()
