# Logging Messages
- name: `/inspection_log/msg`
  - type: `std_msgs::msg::String`
  - qos: 10
  - function: Logs the message to inspection log file
- name: `/inspection_log/log_msg`
  - type: `inspection_planner_interfaces::msg::LogMsg`
  - qos: 10
  - function: Logs the message to log file with header, optionally to terminal as well
- name: `/inspection_log/poses_debug`
  - type: `inspection_planner_interfaces::msg::ViewPosesDebug`
  - qos: 5
  - function: Passes the poses && order of waypoints
- name: `/inspection_log/num_poses_received`
  - type: `std_msgs::msg::UInt32`
  - qos: 5
  - function: Passes the number of poses received each service call
- name: `/inspection_log/num_poses_added`
  - type: `std_msgs::msg::UInt32`
  - qos: 5
  - function: Passes the number of poses added each service call

# Logging Services
- name: `/inspection_log/log_displacement`
  - type: `inspection_planner_interfaces::srv::LogDisplacement`

# Visualization Messages
- name: `/inspection_log/viewpoints_viz`
  - type: `visualization_msgs::msg::MarkerArray`
  - qos: 5
  - function: Rviz visualization of waypoints && tsp result
- name: `/inspection_log/viewposes_viz`
  - type: `geometry_msgs::msg::PoseArray`
  - qos: 5
  - function: Rviz visualization of orientation of view poses
