#!/bin/bash

roscore &
sleep 1

roslaunch gear_launch gear.launch &

rqt

fg
