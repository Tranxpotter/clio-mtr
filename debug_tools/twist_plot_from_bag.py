#!/usr/bin/env python3

import os
import argparse
import matplotlib.pyplot as plt

# ROS 2 Python API for reading bags
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from geometry_msgs.msg import TwistStamped

def get_rosbag_options(bag_path: str, storage_id: str = 'sqlite3'):
    """Configures storage and converter options for the bag reader."""
    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path,
        storage_id=storage_id
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    return storage_options, converter_options

def read_and_plot_bag(bag_path: str, topic_name: str, save_path: str):
    # Initialize the reader
    reader = rosbag2_py.SequentialReader()
    storage_options, converter_options = get_rosbag_options(bag_path)
    
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Error opening bag file: {e}")
        print("Note: Ensure the path points to the directory containing the metadata.yaml or the database file itself.")
        return

    # Filter storage to only look at our specific topic
    storage_filter = rosbag2_py.StorageFilter(topics=[topic_name])
    reader.set_filter(storage_filter)

    # Data arrays
    times = []
    linear_x, linear_y, linear_z = [], [], []
    angular_x, angular_y, angular_z = [], [], []
    start_time = None

    print(f"Parsing bag file for topic: {topic_name}...")

    # Read through all matching messages sequentially
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        
        # Deserialize the raw binary data into a usable Python object
        msg = deserialize_message(data, TwistStamped)
        
        # Calculate time from header stamp
        msg_sec = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)
        
        if start_time is None:
            start_time = msg_sec
            
        relative_time = msg_sec - start_time
        
        # Append message values
        times.append(relative_time)
        linear_x.append(msg.twist.linear.x)
        linear_y.append(msg.twist.linear.y)
        linear_z.append(msg.twist.linear.z)
        angular_x.append(msg.twist.angular.x)
        angular_y.append(msg.twist.angular.y)
        angular_z.append(msg.twist.angular.z)

    if not times:
        print(f"No messages found on topic '{topic_name}' in this bag.")
        return

    print(f"Successfully parsed {len(times)} messages. Generating plots...")

    # --- Plotting Generation ---
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 8), sharex=True)
    
    # Linear velocities
    ax1.plot(times, linear_x, label='Linear X', color='tab:red')
    ax1.plot(times, linear_y, label='Linear Y', color='tab:green')
    ax1.plot(times, linear_z, label='Linear Z', color='tab:blue')
    ax1.set_ylabel('Velocity (m/s)')
    ax1.set_title(f'Linear Velocities vs Time ({topic_name})')
    ax1.legend()
    ax1.grid(True, linestyle='--')
    
    # Angular velocities
    ax2.plot(times, angular_x, label='Angular X', color='tab:red', linestyle='--')
    ax2.plot(times, angular_y, label='Angular Y', color='tab:green', linestyle='--')
    ax2.plot(times, angular_z, label='Angular Z', color='tab:blue', linestyle='--')
    ax2.set_xlabel('Time (seconds)')
    ax2.set_ylabel('Angular Velocity (rad/s)')
    ax2.set_title(f'Angular Velocities vs Time ({topic_name})')
    ax2.legend()
    ax2.grid(True, linestyle='--')
    
    plt.tight_layout()
    
    # Ensure save directory structure exists
    directory = os.path.dirname(save_path)
    if directory and not os.path.exists(directory):
        os.makedirs(directory)
        
    plt.savefig(save_path)
    plt.close(fig)
    print(f"Plot saved successfully to: {os.path.abspath(save_path)}")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Extracts TwistStamped data directly from a rosbag and plots it.")
    parser.add_argument('bag_path', type=str, help="Path to the ROS 2 bag folder or database file.")
    parser.add_argument('--topic', type=str, default='/cmd_vel', help="Topic name to extract (default: /cmd_vel)")
    parser.add_argument('--output', type=str, default='./bag_plot.png', help="Output path for the plot image (default: ./bag_plot.png)")
    
    args = parser.parse_args()
    read_and_plot_bag(args.bag_path, args.topic, args.output)