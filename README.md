- build this package
- mbzirc is a meta-package, the hierarchical style is like

###  mbzirc folder

- mbzirc package(meta)

-- launch folder, CMakeLists

- jsk_mbzirc_common,task1,task2,task3,README,setup,etc..

-- packages.

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

model files can be found in jsk_mbzirc_common package

$ gazebo mbzirc_arena.world

or 

$ roslaunch jsk_mbzirc_common mbzirc_arena.launch

- run simulation of husky + ur5 

`ROBOT_INITIAL_POSE="-x 65 -y -25 -z 0.05 -Y -3.1415" roslaunch jsk_mbzirc_ugv_sim husky_mbzirc.launch`

initial pose is very necessary 



