from typing import List
import datetime
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from ament_index_python.packages import get_package_share_directory

import cv2
from cv_bridge import CvBridge

class RecordNode(Node):
    def __init__(self) -> None:
        super().__init__("record_node")

        self.has_video_writer = False
        # Video recorder parameters
        timestring = datetime.datetime.now().isoformat(timespec="seconds").replace(":", "-")
        self.declare_parameter("filepath", f"~/Videos/{timestring}.mp4")
        self.filepath = self.get_parameter("filepath").get_parameter_value().string_value
        self.filepath = os.path.expanduser(self.filepath)

        self.declare_parameter("fps", 20)
        self.fps = self.get_parameter("fps").get_parameter_value().integer_value


        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')

        # ===============================================
        self.bridge = CvBridge()

        # topics parameters
        self.declare_parameter("color_topic", "/camera/head_camera/color/image_raw")
        # self.declare_parameter("depth_topic", "/camera/head_camera/aligned_depth_to_color/image_raw")

        color_topic = self.get_parameter("color_topic").get_parameter_value().string_value
        # depth_topic = self.get_parameter("depth_topic").get_parameter_value().string_value

        self.color_subscription = self.create_subscription(Image, color_topic, self.color_callback, 10)
        # self.depth_subscription = self.create_subscription(Image, depth_topic, self.depth_callback, 10)
        self.get_logger().info(f"Recording node initialized. Filepath: {self.filepath}, fps: {self.fps}")
    
    def initialize_video_writer(self, img):
        shape = img.shape
        frame_size = (shape[1], shape[0])
        self.get_logger().info(f"{frame_size}")
        self.video_writer = cv2.VideoWriter(self.filepath, self.fourcc, self.fps, frame_size)
        self.has_video_writer = True

        # Check if the video writer opened successfully
        if not self.video_writer.isOpened():
            self.get_logger().error(f"Failed to open VideoWriter for path: {self.filepath}")
            return

        self.get_logger().info(f"Video writer initialized. Saving to {self.filepath}, resolution: {frame_size}, fps: {self.fps}")


    def color_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg)
            cv_image.shape
        except Exception as e:
            self.get_logger().error(f"Error converting color image: {e}")
            return
        
        
        if not self.has_video_writer:
            self.initialize_video_writer(cv_image)
        self.video_writer.write(cv_image)

        cv2.imshow("color image", cv_image)
        cv2.waitKey(1)

    def depth_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg)
        except Exception as e:
            self.get_logger().error(f"Error converting depth image: {e}")
            return
        
        cv2.imshow("depth image", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = RecordNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.has_video_writer:
            node.video_writer.release()
        node.destroy_node()


if __name__ == '__main__':
    main()
