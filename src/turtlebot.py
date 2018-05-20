#!/usr/bin/env python
import rospy
from ros_rsp_server import RosRspServer
from rsp_server.rsserver import RemoteSensorServer
from robot.turtlebot import TurtleBot

if __name__ == '__main__':
    rospy.init_node('ros_rsp_server', anonymous=True)
    server = RemoteSensorServer()
    RosRspServer(TurtleBot(), server=server)
    rospy.spin()
    server.stop()
