#!/usr/bin/env python
# author: Eiichiro Ito
# e-mail: e-itoh@ygu.ac.jp
# created: 2018/5/20
# License: MIT License
import rospy
from ros_rsp_server import RosRspServer
from rsp_server.rsserver import RemoteSensorServer
from robot.turtlebot import TurtleBot

if __name__ == '__main__':
    rospy.init_node('ros_rsp_server', anonymous=True)
    server = RemoteSensorServer()
    robot = TurtleBot()
    if rospy.get_param("/ros_rsp_server/joy", 0) == 1:
        import robot.joystick as joystick
        joystick.setup(robot)
        rospy.loginfo("setup joystick")
    RosRspServer(robot, server=server)
    rospy.spin()
    server.stop()
