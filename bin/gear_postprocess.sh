#!/bin/bash

roscore &
sleep 3

rosrun gear_postprocess gear_postprocess
fg
