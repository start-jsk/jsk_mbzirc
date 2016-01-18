## how to compile

```
cd <catkin_ws>
wstool init -t src
wstool merge -t src https://raw.githubusercontent.com/tongtybj/mbzirc/indigo-devel/mbzirc.rosinstall
wstool update -t src
rosdep install -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO
catkin build
```

## how to run program

- run mbzirc_arena wolrd in gazebo:

$ roslaunch jsk_mbzirc_common mbzirc_arena.launch

- run simulation of task1 with uav in task1 stage
```
$ roslaunch jsk_mbzirc_tasks mbzirc_task_1.launch
```

- run simulation of husky + ur5 in task2 stage
```
$ roslaunch jsk_mbzirc_tasks mbzirc_task_2.launch
$ rostopic pub -r 10 r_gripper_controller/command std_msgs/Float64 "data: 0.2" # to move gripper
```


