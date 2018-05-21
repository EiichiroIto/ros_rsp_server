#!/bin/sh
# author: Eiichiro Ito
# e-mail: e-itoh@ygu.ac.jp
# created: 2018/5/20
# License: MIT License
python ./turtlebot.py cmd_vel:=cmd_vel_mux/input/teleop bumper:=mobile_base/events/bumper cliff:=mobile_base/events/cliff
