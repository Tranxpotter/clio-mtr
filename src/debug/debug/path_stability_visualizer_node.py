#!/usr/bin/env python3

import os
import sys
import time
import threading
import signal
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class PathStabilityVisualizer(Node):
    def __init__(self):
        super().__init__('path_stability_visualizer')
        
        # --- Parameters ---
        self.declare_parameter('topic_name', '/path')
        self.declare_parameter('window_duration', 2.0) # seconds
        
        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.window_duration = self.get_parameter('window_duration').get_parameter_value().double_value
        
        # --- Subscriber ---
        self.subscription = self.create_subscription(
            Path,
            self.topic_name,
            self.path_callback,
            10
        )
        
        # --- Data Buffers & Thread Safety ---
        self.lock = threading.Lock()
        # History stores tuples of: (timestamp, x_array, y_array, is_failure)
        self.history = [] 
        
        self.get_logger().info(f"Visualizer Node initialized. Monitoring {self.topic_name}...")

    def path_callback(self, msg: Path):
        current_time = time.time()
        path_size = len(msg.poses)
        
        if path_size == 0:
            return
            
        # Extract full path coordinates
        x_coords = [p.pose.position.x for p in msg.poses]
        y_coords = [p.pose.position.y for p in msg.poses]
        
        # CMU/standard convention: path size <= 1 means a planning failure/empty fallback state
        is_failure = (path_size <= 1)
        
        with self.lock:
            self.history.append((current_time, x_coords, y_coords, is_failure))


def main(args=None):
    rclpy.init(args=args)
    node = PathStabilityVisualizer()

    # Spin ROS 2 in a background thread to keep it separate from the GUI loop
    ros_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    ros_thread.start()

    # --- Setup Matplotlib Figure (Main Thread) ---
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_title("Real-Time Path Accumulation & Jitter Profile", fontsize=12, fontweight='bold')
    ax.set_xlabel("X (Meters)")
    ax.set_ylabel("Y (Meters)")
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.axis('equal') # Keeps the aspect ratio 1:1 for accurate geometry

    # Maintain a list of active line graphic elements to clear/redraw
    lines = []

    def update_plot(frame):
        nonlocal lines
        current_time = time.time()
        
        with node.lock:
            # Filter history matrix to match the rolling window duration
            node.history = [entry for entry in node.history if current_time - entry[0] <= node.window_duration]
            
            # Remove old lines from the canvas axes
            for line in lines:
                line.remove()
            lines.clear()
            
            if not node.history:
                return lines

            num_paths = len(node.history)
            
            # Draw accumulated lines from oldest to newest
            for idx, (timestamp, x_data, y_data, is_failure) in enumerate(node.history):
                # Calculate alpha fade based on age (newer paths are opaque, older paths fade out)
                age = current_time - timestamp
                alpha = max(0.1, 1.0 - (age / node.window_duration))
                
                if is_failure:
                    # Highlight dropped/failure configurations in dotted red
                    line, = ax.plot(x_data, y_data, color='red', linestyle=':', alpha=alpha, linewidth=1.5)
                elif idx == num_paths - 1:
                    # Bold green line for the absolute freshest path frame
                    line, = ax.plot(x_data, y_data, color='green', alpha=1.0, linewidth=2.5, label='Current Path')
                else:
                    # Trailing historical paths show up as fading blue lines
                    line, = ax.plot(x_data, y_data, color='tab:blue', alpha=alpha * 0.4, linewidth=1.0)
                
                lines.append(line)

            # Dynamically rescale viewport to comfortably focus around active path limits
            all_x = [x for entry in node.history for x in entry[1]]
            all_y = [y for entry in node.history for y in entry[2]]
            
            if all_x and all_y:
                xmin, xmax = min(all_x), max(all_x)
                ymin, ymax = min(all_y), max(all_y)
                padx = max(0.5, 0.1 * (xmax - xmin))
                pady = max(0.5, 0.1 * (ymax - ymin))
                ax.set_xlim(xmin - padx, xmax + padx)
                ax.set_ylim(ymin - pady, ymax + pady)

        return lines

    # --- Graceful Terminal Signal Handling (Resolves 10s SIGKILL hung loop) ---
    def signal_handler(sig, frame):
        print("\nCtrl+C detected! Shutting down visualizer cleanly...")
        plt.close('all')

    signal.signal(signal.SIGINT, signal_handler)

    # Refresh visual layout frame at 25 FPS (every 40 milliseconds)
    ani = FuncAnimation(fig, update_plot, interval=40, blit=False, cache_frame_data=False)
    plt.tight_layout()

    try:
        plt.show()
    except Exception as e:
        print(f"Window closed: {e}")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        sys.exit(0)

if __name__ == '__main__':
    main()