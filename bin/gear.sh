#!/bin/bash

roscore &
sleep 1

roslaunch gear_launch gear.launch do_pointcloud:=false enable_loggers:=false enable_trackers:=false enable_kinect4_depth:=false enable_kinect5_depth:=false&
sleep 2

roslaunch gear_launch logger_gear.launch start_manager:=false nodelet_manager:=gear enable_kinect4_depth:=false enable_kinect5_depth:=false enable_pointgrey1:=false enable_pointgrey2:=false&
sleep 8

rqt -f -l --perspective-file `rosstack find gear_gui`/config/caili.perspective &
rqt -f -l --perspective-file `rosstack find gear_gui`/config/adam.perspective &
rqt -f -l --perspective-file `rosstack find gear_gui`/config/controller.perspective

fg
