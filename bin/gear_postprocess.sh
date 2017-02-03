#!/bin/bash

roscore &
sleep 3

roslaunch gear_gui_launch postprocess_gui.launch
fg
