#!/bin/sh
#set -e 
export GAZEBO_MODEL_PATH=`rospack find jsk_mbzirc_common`/gazebo_model/models:$GAZEBO_MODEL_PATH
export GAZEBO_RESOURCE_PATH=`rospack find jsk_mbzirc_common`/gazebo_model/world:$GAZEBO_RESOURCE_PATH


