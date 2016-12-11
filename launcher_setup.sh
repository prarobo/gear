#!/bin/bash

cp $HOME/gear/gear.desktop $HOME/.local/share/applications/gear.desktop
echo Icon=$HOME/gear/icon.png>>$HOME/.local/share/applications/gear.desktop

cp $HOME/gear/gear_postprocess.desktop $HOME/.local/share/applications/gear_postprocess.desktop
echo Icon=$HOME/gear/icon2.png>>$HOME/.local/share/applications/gear_postprocess.desktop
