#!/bin/bash
# This script gets map and automatically does postprocessing
# usage: util_scripts/postprocess.sh [data folder name]
# This script must be run at workspace root
FASTLIO_MAP_PATH="src/localization/FAST_LIO/PCD/scans.pcd"
FAR_PLANNER_MAP_PATH="map.vgh"

folder_name=$1
cp $FASTLIO_MAP_PATH "data/${folder_name}/map_raw.pcd"
cp $FAR_PLANNER_MAP_PATH "data/${folder_name}/map.vgh"

# PCD map processing
source pcd-map-preprocessing/.venv/bin/activate
python pcd-map-preprocessing/main.py "data/${folder_name}/map_raw.pcd" "data/${folder_name}/map.pcd"
deactivate
