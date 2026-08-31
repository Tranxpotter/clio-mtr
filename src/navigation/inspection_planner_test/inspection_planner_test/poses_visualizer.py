#!/usr/bin/env python3
"""
poses_visualizer - ROS2 node to visualize ViewPosesDebug messages.

Subscribes to /inspection_log/poses_debug, saves messages to disk,
and provides a matplotlib-based UI to visualize inspection poses
with colors for visited/unvisited/failed states, orientation arrows,
pose IDs, and an animation mode for ordered waypoints.

Usage:
    ros2 run inspection_planner_test poses_visualizer
    ros2 launch inspection_planner_test pose_visualizer.launch.py
    ros2 launch inspection_planner_test pose_visualizer.launch.py load_file:=/path/to/saved_message.json
"""

import json
import math
import os
import threading
from datetime import datetime

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.animation as animation
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

import rclpy
from rclpy.node import Node
from inspection_planner_interfaces.msg import ViewPosesDebug, ViewPoses, ViewPose
from geometry_msgs.msg import Pose, Point, Quaternion


def quaternion_to_yaw(q):
    """Convert quaternion (x, y, z, w) to yaw angle in radians."""
    import math
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def save_message_to_file(msg, save_dir):
    """Serialize a ViewPosesDebug message to a JSON file with a timestamp filename."""
    os.makedirs(save_dir, exist_ok=True)
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    filename = os.path.join(save_dir, f'poses_debug_{timestamp}.json')

    def pose_to_dict(p: Pose) -> dict:
        return {
            'position': {
                'x': p.position.x,
                'y': p.position.y,
                'z': p.position.z,
            },
            'orientation': {
                'x': p.orientation.x,
                'y': p.orientation.y,
                'z': p.orientation.z,
                'w': p.orientation.w,
            }
        }

    def view_pose_to_dict(vp: ViewPose) -> dict:
        return {'id': vp.id, 'pose': pose_to_dict(vp.pose)}

    def view_poses_to_dict(vps: ViewPoses) -> list:
        return [view_pose_to_dict(vp) for vp in vps.poses]

    data = {
        'visited': view_poses_to_dict(msg.visited),
        'unvisited': view_poses_to_dict(msg.unvisited),
        'failed': view_poses_to_dict(msg.failed),
        'ordered_ids': list(msg.ordered_ids),
    }

    with open(filename, 'w') as f:
        json.dump(data, f, indent=2)

    return filename


def load_message_from_file(filepath):
    """Load a ViewPosesDebug message from a JSON file."""
    with open(filepath, 'r') as f:
        data = json.load(f)

    msg = ViewPosesDebug()

    def parse_pose_dict(d):
        p = Pose()
        p.position.x = d['position']['x']
        p.position.y = d['position']['y']
        p.position.z = d['position']['z']
        p.orientation.x = d['orientation']['x']
        p.orientation.y = d['orientation']['y']
        p.orientation.z = d['orientation']['z']
        p.orientation.w = d['orientation']['w']
        return p

    def parse_view_poses_list(lst):
        vps = ViewPoses()
        for item in lst:
            vp = ViewPose()
            vp.id = item['id']
            vp.pose = parse_pose_dict(item['pose'])
            vps.poses.append(vp)
        return vps

    msg.visited = parse_view_poses_list(data['visited'])
    msg.unvisited = parse_view_poses_list(data['unvisited'])
    msg.failed = parse_view_poses_list(data['failed'])
    msg.ordered_ids = data['ordered_ids']

    return msg


class PosesVisualizerNode(Node):
    """ROS2 node that subscribes to ViewPosesDebug and triggers visualization."""

    def __init__(self):
        super().__init__('poses_visualizer')

        self.declare_parameter('save_dir', os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'saved_poses'))
        self.declare_parameter('load_file', '')

        self.save_dir = self.get_parameter('save_dir').get_parameter_value().string_value
        self.load_file = self.get_parameter('load_file').get_parameter_value().string_value

        self._latest_msg = None
        self._msg_lock = threading.Lock()
        self._visualization_ready = threading.Event()

        if self.load_file:
            self.get_logger().info(f'Loading message from file: {self.load_file}')
            try:
                msg = load_message_from_file(self.load_file)
                with self._msg_lock:
                    self._latest_msg = msg
                self._visualization_ready.set()
            except Exception as e:
                self.get_logger().error(f'Failed to load file: {e}')
                raise
        else:
            subscriber = self.create_subscription(
                ViewPosesDebug,
                '/inspection_log/poses_debug',
                self._msg_callback,
                10
            )
            self.get_logger().info('Subscribed to /inspection_log/poses_debug')

    def _msg_callback(self, msg: ViewPosesDebug):
        with self._msg_lock:
            self._latest_msg = msg
        saved_path = save_message_to_file(msg, self.save_dir)
        self.get_logger().info(f'Saved message to {saved_path}')
        if not self._visualization_ready.is_set():
            self._visualization_ready.set()

    def get_message(self):
        """Block until a message is available, then return it."""
        self._visualization_ready.wait()
        with self._msg_lock:
            return self._latest_msg


def visualize_poses(msg: ViewPosesDebug):
    """Open a matplotlib window to visualize the poses from a ViewPosesDebug message."""
    visited_poses = msg.visited.poses
    unvisited_poses = msg.unvisited.poses
    failed_poses = msg.failed.poses
    ordered_ids = list(msg.ordered_ids)

    # Collect all poses with their status
    all_poses = []
    for vp in visited_poses:
        all_poses.append((vp, 'visited'))
    for vp in unvisited_poses:
        all_poses.append((vp, 'unvisited'))
    for vp in failed_poses:
        all_poses.append((vp, 'failed'))

    if not all_poses:
        print('No poses to visualize.')
        return

    # Build lookup by id
    pose_by_id = {vp.id: vp for vp, _ in all_poses}
    status_by_id = {vp.id: status for vp, status in all_poses}

    colors = {'visited': 'green', 'unvisited': 'blue', 'failed': 'red'}
    arrow_scale = 0.5

    fig, ax = plt.subplots(figsize=(12, 10))
    plt.subplots_adjust(bottom=0.25 if ordered_ids else 0.1)

    # Store plot elements for animation
    scatter_visited = None
    scatter_unvisited = None
    scatter_failed = None
    arrow_patches = []
    id_texts = []
    order_arrows = []

    def setup_plot():
        nonlocal scatter_visited, scatter_unvisited, scatter_failed
        nonlocal arrow_patches, id_texts, order_arrows

        # Separate by status
        visited_pts = [(vp.pose.position.x, vp.pose.position.y) for vp, s in all_poses if s == 'visited']
        unvisited_pts = [(vp.pose.position.x, vp.pose.position.y) for vp, s in all_poses if s == 'unvisited']
        failed_pts = [(vp.pose.position.x, vp.pose.position.y) for vp, s in all_poses if s == 'failed']

        if visited_pts:
            scatter_visited = ax.scatter([p[0] for p in visited_pts], [p[1] for p in visited_pts],
                                         c='green', s=60, zorder=5, label='Visited')
        if unvisited_pts:
            scatter_unvisited = ax.scatter([p[0] for p in unvisited_pts], [p[1] for p in unvisited_pts],
                                           c='blue', s=60, zorder=5, label='Unvisited')
        if failed_pts:
            scatter_failed = ax.scatter([p[0] for p in failed_pts], [p[1] for p in failed_pts],
                                        c='red', s=60, zorder=5, label='Failed')

        # Orientation arrows
        for vp, status in all_poses:
            x = vp.pose.position.x
            y = vp.pose.position.y
            yaw = quaternion_to_yaw(vp.pose.orientation)
            dx = arrow_scale * math.cos(yaw)
            dy = arrow_scale * math.sin(yaw)
            arrow = ax.arrow(x, y, dx, dy, head_width=0.3, head_length=0.2,
                             fc=colors[status], ec=colors[status], zorder=4)
            arrow_patches.append(arrow)

            # ID text
            txt = ax.text(x, y + 0.4, str(vp.id), fontsize=8, ha='center', va='bottom',
                          color=colors[status], zorder=6)
            id_texts.append(txt)

        # Order arrows
        if len(ordered_ids) > 1:
            for i in range(len(ordered_ids) - 1):
                id_from = ordered_ids[i]
                id_to = ordered_ids[i + 1]
                if id_from in pose_by_id and id_to in pose_by_id:
                    p_from = pose_by_id[id_from].pose.position
                    p_to = pose_by_id[id_to].pose.position
                    dx = p_to.x - p_from.x
                    dy = p_to.y - p_from.y
                    length = math.sqrt(dx * dx + dy * dy)
                    if length > 0:
                        dx_norm = dx / length * arrow_scale
                        dy_norm = dy / length * arrow_scale
                        order_arrow = ax.annotate('', xy=(p_to.x, p_to.y),
                                                  xytext=(p_from.x, p_from.y),
                                                  arrowprops=dict(arrowstyle='->', color='purple',
                                                                  lw=1.5, alpha=0.6),
                                                  zorder=3)
                        order_arrows.append(order_arrow)

        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('Inspection Poses Visualization')
        ax.legend(loc='upper right')
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)

    setup_plot()

    # Animation mode if ordered_ids exists
    if ordered_ids and len(ordered_ids) > 1:
        # For animation, we create individual point markers that we can control
        # Clear the static plot and rebuild for animation
        ax.clear()
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('Inspection Poses — Animated Waypoint Order')
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)

        # Compute axis limits from all poses
        all_x = [vp.pose.position.x for vp, _ in all_poses]
        all_y = [vp.pose.position.y for vp, _ in all_poses]
        margin_x = (max(all_x) - min(all_x)) * 0.1 if max(all_x) != min(all_x) else 1.0
        margin_y = (max(all_y) - min(all_y)) * 0.1 if max(all_y) != min(all_y) else 1.0
        ax.set_xlim(min(all_x) - margin_x, max(all_x) + margin_x)
        ax.set_ylim(min(all_y) - margin_y, max(all_y) + margin_y)

        # Background points (faded)
        for vp, status in all_poses:
            ax.plot(vp.pose.position.x, vp.pose.position.y, 'o',
                    c=colors[status], markersize=6, alpha=0.2, zorder=2)

        # Active elements (updated each frame)
        active_dot, = ax.plot([], [], 'o', c='black', markersize=12, zorder=5)
        active_arrow = ax.annotate('', xy=(0, 0), xytext=(0, 0),
                                    arrowprops=dict(arrowstyle='->', color='black', lw=2),
                                    zorder=4)
        active_text = ax.text(0, 0, '', fontsize=10, ha='center', va='bottom',
                              color='black', fontweight='bold', zorder=6)

        # Order path line
        order_line, = ax.plot([], [], '-', c='purple', lw=1.5, alpha=0.5, zorder=3)

        # Legend
        for label, color in [('Visited', 'green'), ('Unvisited', 'blue'), ('Failed', 'red')]:
            ax.plot([], [], 'o', c=color, markersize=6, alpha=0.2, label=label)
        ax.legend(loc='upper right')

        # Slider for speed
        ax_slider = plt.axes([0.15, 0.02, 0.7, 0.03])
        speed_slider = Slider(ax_slider, 'Speed', 0.5, 10.0, valinit=2.0)

        def update(frame):
            idx = frame % len(ordered_ids)
            current_id = ordered_ids[idx]
            if current_id not in pose_by_id:
                return active_dot, active_arrow, active_text, order_line

            vp = pose_by_id[current_id]
            x = vp.pose.position.x
            y = vp.pose.position.y
            status = status_by_id[current_id]
            color = colors[status]

            # Update active dot
            active_dot.set_data([x], [y])
            active_dot.set_color(color)

            # Update active arrow
            yaw = quaternion_to_yaw(vp.pose.orientation)
            dx = arrow_scale * math.cos(yaw)
            dy = arrow_scale * math.sin(yaw)
            active_arrow.xytext = (x, y)
            active_arrow.xy = (x + dx, y + dy)
            active_arrow.arrowprops['color'] = color

            # Update active text
            active_text.set_position((x, y + 0.5))
            active_text.set_text(str(current_id))
            active_text.set_color(color)

            # Update order path
            path_x = []
            path_y = []
            for j in range(idx + 1):
                pid = ordered_ids[j]
                if pid in pose_by_id:
                    path_x.append(pose_by_id[pid].pose.position.x)
                    path_y.append(pose_by_id[pid].pose.position.y)
            order_line.set_data(path_x, path_y)

            return active_dot, active_arrow, active_text, order_line

        speed = 2.0
        speed_slider.on_changed(lambda v: speeds.append(v))
        speeds = [2.0]

        anim = animation.FuncAnimation(fig, update,
                                        frames=len(ordered_ids),
                                        interval=500,
                                        blit=False,
                                        cache_frame_data=False,
                                        repeat=True)

        plt.tight_layout()
        plt.show()
        return

    plt.tight_layout()
    plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = PosesVisualizerNode()

    # Run the ROS2 node in a background thread so matplotlib can use the main thread
    def spin_node():
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass

    spin_thread = threading.Thread(target=spin_node, daemon=True)
    spin_thread.start()

    try:
        # Wait for the first message (or load from file)
        msg = node.get_message()
        node.get_logger().info('Message received, launching visualization...')

        # Visualize in the main thread (required for matplotlib GUI)
        visualize_poses(msg)
    except Exception as e:
        node.get_logger().error(f'Visualization error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
