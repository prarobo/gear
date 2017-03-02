#!/bin/bash

LAUNCHER_DIR=$GEAR_DIR/launchers
ICON_DIR=$GEAR_DIR/icons
SYS_LAUNCHER_DIR=$HOME/.local/share/applications

cp ${LAUNCHER_DIR}/gear.desktop ${SYS_LAUNCHER_DIR}/gear.desktop
echo Icon=${ICON_DIR}/icon_gear.png>>${SYS_LAUNCHER_DIR}/gear.desktop

cp ${LAUNCHER_DIR}/gear_postprocess.desktop ${SYS_LAUNCHER_DIR}/gear_postprocess.desktop
echo Icon=${ICON_DIR}/icon_postprocess.png>>${SYS_LAUNCHER_DIR}/gear_postprocess.desktop

cp ${LAUNCHER_DIR}/gear_playback.desktop ${SYS_LAUNCHER_DIR}/gear_playback.desktop
echo Icon=${ICON_DIR}/icon_playback.png>>${SYS_LAUNCHER_DIR}/gear_playback.desktop