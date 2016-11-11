#!/bin/bash

roscore &
sleep 1

roslaunch gear_launch gear.launch enable_loggers:=false &
sleep 1

roslaunch gear_launch logger_gear.launch &

rqt

fg
