#!/bin/bash

roscore &
sleep 1

roslaunch gear_launch gear.launch do_pointcloud:=false enable_loggers:=true enable_trackers:=true use_sim_time:=true enable_playback:=false &
sleep 1

roslaunch gear_data_handler playback.launch &
sleep 1

roslaunch gear_gui_launch playback_gui.launch
fg
