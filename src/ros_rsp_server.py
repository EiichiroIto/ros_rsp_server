#!/usr/bin/env python
import rospy

class RosRspServer(object):
    def __init__(self, robot, server):
        if robot is None:
            rospy.logerr("robot must be set")
        if server is None:
            rospy.logerr("server must be set")
        self.robot = robot
        self.server = server
        self.server.set_controller(self)
        self.setup_publishers()
        self.setup_subscribers()
        self.server.start()

    def setup_publishers(self):
        for topic in self.robot.pub_topics:
            pub = self.robot.pub_topics[topic]
            pub.setup_publisher(rospy, self.robot)

    def setup_subscribers(self):
        for topic in self.robot.sub_topics:
            sub = self.robot.sub_topics[topic]
            sub.setup_subscriber(rospy, self.robot, self.server)

    def sensor_update(self, list):
        for (n, v) in list:
            if n in self.robot.variables:
                self.robot.variables[n] = v
                print("%s=%s" % (n,v))
            else:
                rospy.logwarn("unknown variable: %s" % n)

    def broadcast(self, str):
        if str in self.robot.pub_topics:
            pub = self.robot.pub_topics[str]
            pub.publish()
        else:
            rospy.logwarn("unknown topic: %s" % str)
