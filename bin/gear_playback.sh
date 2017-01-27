#!/bin/bash

roscore &
sleep 1

roslaunch gear_launch gear.launch do_pointcloud:=false enable_loggers:=false enable_trackers:=false use_sim_time:=true &
sleep 1

rosrun rqt_image_view rqt_image_view &
sleep 1

rosrun gear_playback gear_playback
fg
