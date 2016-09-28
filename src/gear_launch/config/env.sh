#!/usr/bin/env bash

export ROSLAUNCH_SSH_UNKNOW=1
export ROS_MASTER_URI=http://cooplab-gear:11311

source $HOME/gear/devel/setup.bash
exec "$@"