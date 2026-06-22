## CMU navigation Launch
ros2 launch bringup cmu_navigation.launch.py map_path:=iw_maps/11-3-IWLG-full.pcd map_2d_path:=iw_2d_maps/11-3-IWLG-full.yaml bag_path:=rosbags/jitter1 > log4.txt

### Use rosbag
ros2 launch bringup cmu_navigation.launch.py map_path:=iw_maps/11-3-IWLG-full.pcd map_2d_path:=iw_2d_maps/11-3-IWLG-full.yaml use_bag:=True record_bag:=False > log5.txt

ros2 bag play rosbags/slow-moving --clock --topics /livox/imu /livox/lidar /initialpose /way_point

## Debug
### Plot cmd_vel from rosbag
python3 debug_tools/twist_plot_from_bag.py rosbags/one_way4/ --topic /cmd_vel --output debug/test.png