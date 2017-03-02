#!/bin/bash

rosdoc_lite -o $GEAR_DIR/doc `rospack find gear_data_handler`
rosdoc_lite -o $GEAR_DIR/doc `rospack find gear_tf`
rosdoc_lite -o $GEAR_DIR/doc `rospack find gear_diagnostics`
rosdoc_lite -o $GEAR_DIR/doc `rospack find gear_image_capture`
rosdoc_lite -o $GEAR_DIR/doc `rospack find gear_playback`
rosdoc_lite -o $GEAR_DIR/doc `rospack find gear_postprocess`
rosdoc_lite -o $GEAR_DIR/doc `rospack find gear_session_duration`
