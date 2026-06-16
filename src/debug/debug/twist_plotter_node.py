#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import matplotlib.pyplot as plt

class TwistPlotterNode(Node):
    def __init__(self):
        super().__init__('twist_plotter_node')
        
        # --- Parameters ---
        # Declare parameters so they can be easily configured via launch files or CLI
        self.declare_parameter('topic_name', '/cmd_vel')
        self.declare_parameter('save_path', './twist_plots.png')
        self.declare_parameter('max_data_points', 500) # Prevents memory leaks over long runs
        
        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.save_path = self.get_parameter('save_path').get_parameter_value().string_value
        self.max_pts = self.get_parameter('max_data_points').get_parameter_value().integer_value

        # --- Data Storage ---
        self.times = []
        self.linear_x, self.linear_y, self.linear_z = [], [], []
        self.angular_x, self.angular_y, self.angular_z = [], [], []
        
        self.start_time = None

        # --- Subscriber & Shutdown Hook ---
        self.subscription = self.create_subscription(
            TwistStamped,
            self.topic_name,
            self.listener_callback,
            10
        )
        
        
        self.get_logger().info(f"Node initialized. Listening to {self.topic_name}...")
        self.get_logger().info(f"Plots will save to {os.path.abspath(self.save_path)} upon shutdown.")

    def listener_callback(self, msg: TwistStamped):
        # Extract timestamp from the message header
        msg_sec = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)
        
        if self.start_time is None:
            self.start_time = msg_sec
            
        # Calculate relative time from the start of recording
        relative_time = msg_sec - self.start_time
        
        # Append data
        self.times.append(relative_time)
        self.linear_x.append(msg.twist.linear.x)
        self.linear_y.append(msg.twist.linear.y)
        self.linear_z.append(msg.twist.linear.z)
        self.angular_x.append(msg.twist.angular.x)
        self.angular_y.append(msg.twist.angular.y)
        self.angular_z.append(msg.twist.angular.z)
        
        # Keep buffer bounded
        # if len(self.times) > self.max_pts:
        #     self.times.pop(0)
        #     self.linear_x.pop(0); self.linear_y.pop(0); self.linear_z.pop(0)
        #     self.angular_x.pop(0); self.angular_y.pop(0); self.angular_z.pop(0)

    def save_plots(self):
        self.get_logger().info("Shutdown detected. Generating and saving plots...")
        
        if not self.times:
            self.get_logger().warn("No data received yet. Skipping plot generation.")
            return

        # Setup 2x1 grid of subplots (Linear on top, Angular on bottom)
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
        
        # 1. Linear Subplot
        ax1.plot(self.times, self.linear_x, label='Linear X', color='r')
        ax1.plot(self.times, self.linear_y, label='Linear Y', color='g')
        ax1.plot(self.times, self.linear_z, label='Linear Z', color='b')
        ax1.set_ylabel('Velocity (m/s)')
        ax1.set_title('Linear Velocities vs Time')
        ax1.legend()
        ax1.grid(True)
        
        # 2. Angular Subplot
        ax2.plot(self.times, self.angular_x, label='Angular X', color='r', linestyle='--')
        ax2.plot(self.times, self.angular_y, label='Angular Y', color='g', linestyle='--')
        ax2.plot(self.times, self.angular_z, label='Angular Z', color='b', linestyle='--')
        ax2.set_xlabel('Time (seconds)')
        ax2.set_ylabel('Angular Velocity (rad/s)')
        ax2.set_title('Angular Velocities vs Time')
        ax2.legend()
        ax2.grid(True)
        
        plt.tight_layout()
        
        # Ensure directories exist before saving
        directory = os.path.dirname(self.save_path)
        if directory and not os.path.exists(directory):
            os.makedirs(directory)
            
        try:
            plt.savefig(self.save_path)
            self.get_logger().info(f"Successfully saved plots to: {self.save_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save plot: {str(e)}")
            
        plt.close(fig)

def main(args=None):
    rclpy.init(args=args)
    node = TwistPlotterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Node cleanup is handled gracefully here
        node.save_plots()
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()