## CMU navigation Launch
ros2 launch bringup cmu_navigation.launch.py map_path:=iw_maps/11-3-IWLG-full.pcd map_2d_path:=iw_2d_maps/11-3-IWLG-full.yaml bag_path:=rosbags/jitter1 > log4.txt

### Use rosbag
ros2 launch bringup cmu_navigation.launch.py map_path:=iw_maps/11-3-IWLG-full.pcd map_2d_path:=iw_2d_maps/11-3-IWLG-full.yaml use_bag:=True record_bag:=False > log5.txt

ros2 bag play rosbags/slow-moving --clock --topics /livox/imu /livox/lidar /initialpose /way_point

## Debug
### Plot cmd_vel from rosbag
python3 debug_tools/twist_plot_from_bag.py rosbags/one_way4/ --topic /cmd_vel --output debug/test.png



## Inspection Pose Selector testing pipeline
### Recording GT
1. ros2 launch bringup cmu_mapping.launch.py
2. ros2 launch inspection_planner_test odometry_saver.launch.py

### Processing data
#### Navigation map
1. ./util_scripts/get_map.sh data/iw_maps/1.pcd data/iw_graphs/1.vgh
2. source pcd-map-preprocessing/.venv/bin/activate
3. python pcd-map-preprocessing/main.py data/iw_maps/1.pcd data/iw_maps/1_transformed.pcd
4. change maps in cmu_navigation.launch.py
#### Inspection Pose selection
1. Copy database to image_selection ws
2. python src/image_selection/sample_images_along_trajectory.py --interval-m 1.0 --db data/databases/inspections.db --commit-gt --no-skip-existing-gt --no-copy
3. Copy database back to inspection_planner_test/databases

##  Inspection
1. ros2 launch bringup inspect.launch.py
2. ros2 service call /load_inspection_poses inspection_planner_interfaces/srv/LoadInspectionPoses "{db_path: 'src/navigation/inspection_planner_test/databases/processed.db'}"