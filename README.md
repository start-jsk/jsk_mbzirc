- build this package
e.g. $ catkin build mbzirc

- add the mbzirc.sh to ~/.bashrc
$ echo "source `rospack find mbzirc`/setup.sh" >> ~/.bashrc
$ source ~/.bashrc

- run mbzirc_arena wolrd in gazebo:
$ gazebo mbzirc_arena.world
or 
$ roslaunch mbzirc mbzirc_arena.launch


- for developer
models are the sdf files, put those files into :~/.gazebo/models/

