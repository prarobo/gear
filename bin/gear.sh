#!/bin/bash

roscore &
sleep 1

roslaunch gear_launch gear.launch do_pointcloud:=false enable_loggers:=false enable_trackers:=false enable_kinect4_depth:=false enable_kinect5_depth:=false&
sleep 2

roslaunch gear_launch logger_gear.launch start_manager:=false nodelet_manager:=gear enable_kinect4_depth:=false enable_kinect5_depth:=false enable_pointgrey1:=false enable_pointgrey2:=false&
sleep 8

roslaunch gear_gui_launch data_collection_gui.launch
fg
