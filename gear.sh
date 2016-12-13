#!/bin/bash

roscore &
sleep 1

roslaunch gear_launch gear.launch do_pointcloud:=false enable_loggers:=false enable_trackers:=false&
sleep 2

roslaunch gear_launch logger_gear.launch start_manager:=false nodelet_manager:=gear &
sleep 8

rqt -f -l --perspective-file `rosstack find gear_gui`/config/right.perspective &
rqt -f -l --perspective-file `rosstack find gear_gui`/config/left.perspective &
rqt -f -l --perspective-file `rosstack find gear_gui`/config/controller.perspective

fg
