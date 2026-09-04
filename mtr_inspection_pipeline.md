# Mapping Run
1. ros2 launch bringup cmu_mapping.launch.py
2. Check ros topics `ros2 topic list -t`

# Map processing
1. ./util_scripts/postprocess.sh [folder_name]
2. change maps in cmu_navigation.launch.py

# Inspection
1. ros2 launch bringup inspect.launch.py
2. ros2 service call /load_inspection_poses inspection_planner_interfaces/srv/LoadInspectionPoses "{db_path: 'data/[folder_name]/database.db'}"