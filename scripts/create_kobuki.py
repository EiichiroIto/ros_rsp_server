#!/usr/bin/env python
# author: Eiichiro Ito
# e-mail: e-itoh@ygu.ac.jp
# created: 2018/5/21
# License: MIT License
import rospy
from create_node.msg import TurtlebotSensorState
from kobuki_msgs.msg import BumperEvent, CliffEvent

# bumper constants
LEFT = 0
CENTER = 1
RIGHT = 2
NOTHING = 3

# bumper status
last_l = 0
last_r = 0
last_no = NOTHING

rospy.init_node('create_kobuki')
pub = rospy.Publisher('bumper', BumperEvent, queue_size=1)

def bumper_no(left, right):
    if (left != 0) and (right == 0):
        return LEFT
    if (left != 0) and (right != 0):
        return CENTER
    if (left == 0) and (right != 0):
        return RIGHT
    return NOTHING

def publish_bumper(no, state):
    if no != NOTHING:
        bumper = BumperEvent()
        bumper.bumper = no
        bumper.state = state
#        rospy.loginfo("sent %d,%d" % (bumper.bumper, bumper.state))
        pub.publish(bumper)

def callback(msg):
    global last_l, last_r, last_no
#    rospy.loginfo("bumper=%d" % msg.bumps_wheeldrops)
    left = msg.bumps_wheeldrops & 2
    right = msg.bumps_wheeldrops & 1
    if (last_l != left) or (last_r != right):
        publish_bumper(last_no, 0)
        last_l = left
        last_r = right
        last_no = bumper_no(left, right)
        publish_bumper(last_no, 1)

sub = rospy.Subscriber('/turtlebot/sensor_state', TurtlebotSensorState, callback)
rospy.spin()
