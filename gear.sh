#!/bin/bash

roscore &
sleep 1

roslaunch gear_launch gear.launch &

sleep 8
rqt -f -l --perspective-file `rosstack find gear_gui`/config/controller.perspective &
rqt -f -l --perspective-file `rosstack find gear_gui`/config/left.perspective &
rqt -f -l --perspective-file `rosstack find gear_gui`/config/right.perspective

fg
