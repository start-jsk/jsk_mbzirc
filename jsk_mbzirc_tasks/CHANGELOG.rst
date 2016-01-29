^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_mbzirc_tasks
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.1 (2016-01-29)
------------------
* add tools to tune effort controllers pid param
* fix the height of the target
* update effort controller parameters
* move uav_teleop_keyboard to  jsk_mbzirc_tasks, no launch file in jsK_mbzirc_common use them
* add BSD copyright
* add full name to maintainer
* disable allow_sensor_manager, which uses OpenGL in travis
* task_2_cheat.py: set turn valve motion to 4 times
* initialize all interface before sending pose
* use JTA for rotate valve
* increase time-limit
* set initial pose before sending cheat motion
* fix test name
* need to wait for 5 sec to stow ur5 arm
* jsk_mbzirc_task2.launch is also needs  https://github.com/ros/ros_comm/pull/730 due to husky
* Merge remote-tracking branch 'origin/indigo-devel' into add_task2_test
* jsk_mbzirc_task_2.launch: support headless mode
* add moveit_commander
* use effort controller of ur5
* fix typo in jsk_mbzirc_task_2.launch
* use effort_controller for ur5
* add jsk_mbzirc_task2.test
* add teleop gripper control
* rm separate task folders and minor change in README
* jsk_mbzirc_tasks/test/jsk_mbzirc_task_1.test: use retry = 2
* use https://github.com/ros/ros_comm/pull/730 version of roslaunch-check
* jsk_mbzirc_tasks/package.xml add husky
* jsk_mbzirc_tasks/package.xml: add depends to hector_quadrotor_gazebo, rviz
* jsk_mbzirc_tasks/test: use headless mode for test
* .launch: add headress arguments
* add depends to teleop_twist_keyboard
* add test codes
* task_2: add moveit code
* jsk_mbzirc_task_1.launch: add cheat mode
* add jsk_mbzirc_tasks
* Contributors: Kei Okada, Xiangyu Chen
