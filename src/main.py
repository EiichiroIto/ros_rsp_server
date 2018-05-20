#!/usr/bin/env python
import rospy
from rsp_server.rsserver import RemoteSensorServer
from robot.turtlebot import pub_topics, sub_topics, variables

class RosRspServer(object):
    def __init__(self, server=None):
        if server is not None:
            server.set_controller(self)
        self.setup_publishers()
        self.setup_subscribers()
        server.start()

    def setup_publishers(self):
        for topic in pub_topics:
            pub = pub_topics[topic]
            pub.setup_publisher(rospy)

    def sensor_update(self, list):
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

    def setup_subscribers(self):
        for topic in sub_topics:
            sub = sub_topics[topic]
            sub.setup_subscriber(rospy, server)

if __name__ == '__main__':
    rospy.init_node('ros_rsp_server', anonymous=True)
    server = RemoteSensorServer()
    rrs = RosRspServer(server=server)
    rospy.spin()
    server.stop()
