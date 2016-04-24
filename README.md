[![Build Status](https://travis-ci.org/start-jsk/jsk_mbzirc.svg?branch=master)](https://travis-ci.org/start-jsk/jsk_mbzirc)

a

## how to compile

```
cd <catkin_ws>
wstool init src
wstool set -t src jsk_mbzirc http://github.com/start-jsk/jsk_mbzirc --git
wstool merge -t src https://raw.githubusercontent.com/start-jsk/jsk_mbzirc/master/mbzirc.rosinstall
wstool update -t src
rosdep install -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO
catkin build
```

## how to run program

- run mbzirc_arena wolrd in gazebo:
```
$ roslaunch jsk_mbzirc_common mbzirc_arena_1.launch
```

- run simulation of task1 with uav in task1 stage
```
$ roslaunch jsk_mbzirc_tasks jsk_mbzirc_task_1.launch
```

- run simulation of husky + ur5 in task2 stage
```
$ roslaunch jsk_mbzirc_tasks jsk_mbzirc_task_2.launch
$ rostopic pub -r 10 r_gripper_controller/command std_msgs/Float64 "data: 0.2" # to move gripper
```

## Operation notes

### Task1 

- Keyboard teleoperation
```PL``` for going up and down
```H``` for Hover stable
```WASD``` for horizontal direction move
```QE``` for Yaw
Hold shift and press above button will accerate the speed of the drone

- Task1 cheat
A simple demonstration of landing on the truck.
the cheat program will move the drone if the sim time is less than 2 seconds, 
first launch the jsk_mbzirc_tasks and then launch the jsk_mbzirc_common
```
$ roslaunch jsk_mbzirc_tasks jsk_mbzirc_task_1.launch cheat:=true
$ roslaunch jsk_mbzirc_common mbzirc_arena_1.launch
```

### Task2

- Keyboard teleoperation
Using the same keyboard layout for horizontal and yaw control, also enabled Shift key.
```OC``` for open and close the gripper.


