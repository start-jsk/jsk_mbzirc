- build this package
- mbzirc is a meta-package, the hierarchical style is like

###  mbzirc folder

- mbzirc package(meta)

-- launch folder, CMakeLists

- gazebo_model,task1,task2,task3,README,setup,etc..

-- packages.


e.g. $ catkin build mbzirc


- add the mbzirc.sh to ~/.bashrc

$ echo "source `rospack find mbzirc`/../setup.sh" >> ~/.bashrc

$ source ~/.bashrc


- run mbzirc_arena wolrd in gazebo:

$ gazebo mbzirc_arena.world

or 

$ roslaunch jsk_mbzirc_common mbzirc_arena.launch

- run simulation of husky + ur5 

`ROBOT_INITIAL_POSE="-y 65 -x 25 -Y -1.57" roslaunch jsk_mbzirc_ugv_sim husky_mbzirc.launch`

initial pose is very necessary 

# for husky, please install as follow:
do `./install.sh` 
There maybe some ros packages that needed, like 
ros-indigo-moveit-simple-controller-manager 
ros-indigo-pr2-controller-manager 
and etc. Plz follow the error message and do apt install.


# Before push do 
`find . -type f -name '*~' -delete`

