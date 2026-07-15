## Commands
1. `ros2 launch bringup cmu_mapping.launch.py`
2. `./util_scripts/get_map.sh iw_maps/scans.pcd iw_graphs/map.vgh`
3. `source pcd-map-preprocessing/.venv/bin/activate`
4. `python pcd-map-preprocessing/main.py iw_maps/scans.pcd iw_maps/scans_transformed.pcd`
5. `ros2 launch bringup inspect.launch.py`