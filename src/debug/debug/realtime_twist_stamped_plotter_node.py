#!/usr/bin/env python3

import os
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import signal
import sys

class RealTimeTwistStampedPlotterNode(Node):
    def __init__(self):
        super().__init__('realtime_twist_stamped_plotter_node')
        
        # --- Parameters ---
        self.declare_parameter('topic_name', '/cmd_vel')
        self.declare_parameter('window_size_seconds', 10.0)
        self.declare_parameter('save_path', './realtime_last_session.png')
        
        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.window_size = self.get_parameter('window_size_seconds').get_parameter_value().double_value
        self.save_path = self.get_parameter('save_path').get_parameter_value().string_value

        # --- Data Buffers ---
        self.lock = threading.Lock()  # Thread safety between ROS and Matplotlib
        self.times = []
        self.linear_x, self.linear_y, self.linear_z = [], [], []
        self.angular_x, self.angular_y, self.angular_z = [], [], []
        self.start_time = None

        # --- Subscriber ---
        self.subscription = self.create_subscription(
            TwistStamped,
            self.topic_name,
            self.listener_callback,
            10
        )
        self.get_logger().info(f"Listening to {self.topic_name}. Showing a rolling {self.window_size}s window.")

    def listener_callback(self, msg: TwistStamped):
        msg_sec = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)
        
        with self.lock:
            if self.start_time is None:
                self.start_time = msg_sec
                
            relative_time = msg_sec - self.start_time
            
            # Append new data
            self.times.append(relative_time)
            self.linear_x.append(msg.twist.linear.x)
            self.linear_y.append(msg.twist.linear.y)
            self.linear_z.append(msg.twist.linear.z)
            self.angular_x.append(msg.twist.angular.x)
            self.angular_y.append(msg.twist.angular.y)
            self.angular_z.append(msg.twist.angular.z)
            
            # Evict data older than the moving window size (10 seconds)
            cutoff_time = relative_time - self.window_size
            while self.times and self.times[0] < cutoff_time:
                self.times.pop(0)
                self.linear_x.pop(0); self.linear_y.pop(0); self.linear_z.pop(0)
                self.angular_x.pop(0); self.angular_y.pop(0); self.angular_z.pop(0)


def main(args=None):
    rclpy.init(args=args)
    node = RealTimeTwistStampedPlotterNode()

    # Spin ROS 2 in a dedicated background thread
    ros_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    ros_thread.start()

    # --- Setup Matplotlib Figure ---
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    fig.suptitle(f"Real-Time Twist Stamped Canvas\nTopic: {node.topic_name}", fontsize=14, fontweight='bold')
    
    # Initialize empty line plots
    line_lx, = ax1.plot([], [], label='Linear X', color='r')
    line_ly, = ax1.plot([], [], label='Linear Y', color='g')
    line_lz, = ax1.plot([], [], label='Linear Z', color='b')
    ax1.set_ylabel('Velocity (m/s)')
    ax1.grid(True)
    ax1.legend(loc='upper right')

    line_ax, = ax2.plot([], [], label='Angular X', color='r', linestyle='--')
    line_ay, = ax2.plot([], [], label='Angular Y', color='g', linestyle='--')
    line_az, = ax2.plot([], [], label='Angular Z', color='b', linestyle='--')
    ax2.set_xlabel('Relative Time (seconds)')
    ax2.set_ylabel('Angular Velocity (rad/s)')
    ax2.grid(True)
    ax2.legend(loc='upper right')

    def update_plot(frame):
        with node.lock:
            if not node.times:
                return line_lx, line_ly, line_lz, line_ax, line_ay, line_az

            # Update line data paths
            line_lx.set_data(node.times, node.linear_x)
            line_ly.set_data(node.times, node.linear_y)
            line_lz.set_data(node.times, node.linear_z)
            
            line_ax.set_data(node.times, node.angular_x)
            line_ay.set_data(node.times, node.angular_y)
            line_az.set_data(node.times, node.angular_z)

            # Adjust X-axis view frame dynamically based on current time bounds
            latest_time = node.times[-1]
            ax2.set_xlim(max(0.0, latest_time - node.window_size), max(node.window_size, latest_time))
            
            # Dynamically rescale Y-axis limits comfortably to fit changing data thresholds
            for ax, lines in zip([ax1, ax2], [[line_lx, line_ly, line_lz], [line_ax, line_ay, line_az]]):
                all_y = []
                for line in lines:
                    y_data = line.get_ydata()
                    if len(y_data) > 0:
                        all_y.extend(y_data)
                if all_y:
                    ymin, ymax = min(all_y), max(all_y)
                    padding = max(0.1, 0.15 * (ymax - ymin)) # Prevent zero-height flatness errors
                    ax.set_ylim(ymin - padding, ymax + padding)

        return line_lx, line_ly, line_lz, line_ax, line_ay, line_az

    # --- Handle Ctrl+C (SIGINT) Cleanly ---
    def signal_handler(sig, frame):
        print("\nCtrl+C detected! Closing plot and shutting down...")
        # Closing all plots immediately breaks the plt.show() blocking loop
        plt.close('all') 

    # Override the default SIGINT behavior
    signal.signal(signal.SIGINT, signal_handler)

    # Refresh the GUI frame every 50ms
    ani = FuncAnimation(fig, update_plot, interval=50, blit=False, cache_frame_data=False)
    plt.tight_layout()
    
    try:
        plt.show()  # This will now unblock immediately on Ctrl+C
    except Exception as e:
        print(f"Window closed with exception: {e}")
    finally:
        # Saving snapshot logic upon exit
        with node.lock:
            if node.times:
                directory = os.path.dirname(node.save_path)
                if directory and not os.path.exists(directory):
                    os.makedirs(directory)
                plt.savefig(node.save_path)
                print(f"Saved final window snapshot to: {os.path.abspath(node.save_path)}")
        
        # Shutdown sequences cleanly without waiting for SIGKILL
        node.destroy_node()
        rclpy.try_shutdown()
        sys.exit(0)

if __name__ == '__main__':
    main()