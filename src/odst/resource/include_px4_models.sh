#!/usr/bin/env bash
# Mostly taken from the gz_env.sh script of PX4
# (https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/simulation/gz_bridge/gz_env.sh.in)
# to be sure it created an equivalent environment
PX4_GZ_MODELS=~/Documents/git/SpartanLIFT/PX4-Autopilot/Tools/simulation/gz/models
PX4_GZ_WORLDS=~/Documents/git/SpartanLIFT/PX4-Autopilot/Tools/simulation/gz/worlds

if ! [[ $GZ_SIM_RESOURCE_PATH =~ "$PX4_GZ_MODELS" ]]; then
    export GZ_SIM_RESOURCE_PATH="$PX4_GZ_MODELS:$GZ_SIM_RESOURCE_PATH"
fi
if ! [[ $GZ_SIM_RESOURCE_PATH =~ "$PX4_GZ_WORLDS" ]]; then
    export GZ_SIM_RESOURCE_PATH="$PX4_GZ_WORLDS:$GZ_SIM_RESOURCE_PATH"
fi

# echo $GZ_SIM_RESOURCE_PATH
