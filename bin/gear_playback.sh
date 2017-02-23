#!/bin/bash

roscore &
sleep 1

roslaunch gear_launch gear.launch do_pointcloud:=false enable_loggers:=false enable_trackers:=true use_sim_time:=true enable_playback:=false &
sleep 1

roslaunch gear_launch logger_gear.launch start_manager:=false nodelet_manager:=gear synchronize:=false \
enable_kinect1:=false enable_kinect2:=false enable_kinect3:=false enable_kinect4:=false enable_kinect5:=false \
enable_kinect1_depth:=false enable_kinect2_depth:=false enable_kinect3_depth:=false enable_kinect4_depth:=false enable_kinect5_depth:=false \
enable_audio:=false &
sleep 1

roslaunch gear_data_handler playback.launch &
sleep 1

roslaunch gear_gui_launch playback_gui.launch
fg
