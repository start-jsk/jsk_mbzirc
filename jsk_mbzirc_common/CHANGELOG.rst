^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_mbzirc_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.1 (2016-01-29)
------------------
* move uav_teleop_keyboard to  jsk_mbzirc_tasks, no launch file in jsK_mbzirc_common use them
* add BSD copyright
* use full name for model author
* add full name to maintainer
* Merge remote-tracking branch 'origin/indigo-devel' into add_task2_test
* add mbzirc_gazebo_panel_plugi
* add stem joint
* add teleop gripper control
* jsk_mbzirc_common is general package, so it does not include any of specific robots
* .launch: add headress arguments
* add test codes
* mbzirc_gazebo_truck_plugin.cpp: use heliport
* convert to sdf version 1.4
* src/mbzirc_gazebo_truck_plugin.cpp: add lock
* jsk_mbzirc_common/src/mbzirc_gazebo_truck_plugin.h: pubish remaining time
* jsk_mbzirc_commonn/launch: support gui arguments
* jsk_mbzirc_common: move mbzirc_task_X.launch to mbzirc_arena_X.launch, htis is launc just to start empty arena world
* jsk_mbzirc_common, remove uav model, mbzirc_arena_1.launch, rviz config to jsk_mbzirc_task
* jsk_mbzirc_common/CMakeLists.txt: add install
* mbzirc_task_1.launch: always take care of indent of the flies
* mbzirc_arena.launch: remove uncomment includes tag
* change stem model, stable now
* follow the standard install way
* fix task3 world
* remove unnecessary models
* wrenches ready to be picked
* add wrench
* add uav model, and keyboard teleop to do the task1
* 1. complete the truck model
  - add tires
  - seperate the heliport as another link
  2. fix the arena position
  - level with the z=0 plane
* not sure why @cretaceous-creature changed direction
* fix inertia
* add mbzirc_task_3_launch
* add mbzirc_gazebo_treasure_plugin, better to use ModelPlugin?
* convert panel model to version 1.4
* add task2
* add mbzirc_arena_task_1.world
* add task1.launch
* fix mbzirc_gazebo_truck_plugin, support scroe publisher
* mbzirc_arena.launch: add paused and debug argument
* add std_msgs
* mbzirc_gazebo_truck_plugin.cpp: fix typo  cicle - circle
* src/mbzirc_gazebo_truck_plugin.cpp: fix indent
* add mbzirc_gazebo_truck_plugin to CMakeLIsts.txt
* add truck model
* using new map
* use gazebo_ros tag in package.xml to set GAZEBO_MODEL_PATH environment, DO NOT USE setup.sh
* include gazebo_ros/launch/empty_world.launch
* move model fold under jsk_mbzirc_common
* remove all the backup file with "~", and add .gitignore
* fix conflict
* create package hierarchical style
* Contributors: Kei Okada, Moju Zhao, Xiangyu Chen
