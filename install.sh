#!/bin/sh
cd `pwd`/../
###husky repo 
git clone https://github.com/husky/husky
git clone https://github.com/husky/husky_robot
git clone https://github.com/husky/husky_simulator
git clone https://github.com/husky/husky_customization
git clone https://github.com/husky/husky_desktop

###Universal robot repo 
git clone https://github.com/ros-industrial/universal_robot

cd husky_simulator && git checkout gripper-and-roscontrol-fix && cd ..
cd husky && git checkout gripper-addition && cp ../mbzirc/husky.patch ./
git apply husky.patch && cd ..
###catkin make 
#cd .. && catkin_make install mbzirc

###catkin build 

catkin build mbzirc && cd ..

