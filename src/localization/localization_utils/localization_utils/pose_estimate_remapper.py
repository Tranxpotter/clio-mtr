'''
PoseEstimateRemapper Node
This node reads nav2 initial pose estimate topic and call FASTLIO2_ROS2 localizer topic 

Parameters
-----------
map_path: `str`
    The path of the map to use (.pcd)
verbose: `bool`
    Log what the node is doing
'''


'''Note for FASTLIO2_ROS2 interfaces:

Relocalize: 
string pcd_path
float32 x
float32 y
float32 z
float32 yaw
float32 pitch
float32 roll
---
bool success
string message

IsValid:
int32 code
---
bool valid
'''

from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import rclpy
from rclpy.node import Node
import math
from interface.srv import Relocalize, IsValid # Service of FASTLIO2_ROS2 /localizer/relocalize and /localizer/relocalize_check
from geometry_msgs.msg import PoseWithCovarianceStamped # This is the msg type of /goal_pose

class PoseEstimateRemapper(Node):
    def __init__(self):
        super().__init__('pose_estimate_remapper')
        
        # Parameters
        self.declare_parameter('map_path', 'maps/map.pcd')
        self.declare_parameter('verbose', True)
        self.map_path = self.get_parameter('map_path').get_parameter_value().string_value
        self.verbose = self.get_parameter('verbose').get_parameter_value().bool_value
    
        # Subscriber
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose",
            self.initial_pose_callback,
            10)
        self.relocalize_client = self.create_client(Relocalize, "/localizer/relocalize")
        

    def initial_pose_callback(self, msg:PoseWithCovarianceStamped):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        x, y, z = position.x, position.y, position.z
        ox, oy, oz, ow = orientation.x, orientation.y, orientation.z, orientation.w

        # Convert quaternion to roll, pitch, yaw
        roll = math.atan2(2 * (ow * ox + oy * oz), 1 - 2 * (ox * ox + oy * oy))
        pitch = math.asin(2 * (ow * oy - oz * ox))
        yaw = math.atan2(2 * (ow * oz + ox * oy), 1 - 2 * (oy * oy + oz * oz))

        # Lidar is now facing downward without horizontal tilts
        # Only apply 180 degree yaw rotation for frame alignment
        # pitch += math.radians(-180.0)
        # yaw += math.radians(180.0)

        request = Relocalize.Request()
        request.pcd_path = self.map_path
        request.x, request.y, request.z = x, y, z
        request.yaw, request.pitch, request.roll = yaw, pitch, roll

        future = self.relocalize_client.call_async(request)
        if self.verbose:
            self.get_logger().info(f"Sending relocalize request: {x=} {y=} {z=} {yaw=} {pitch=} {roll=}")
        future.add_done_callback(self.on_relocalize_done)
    
    def on_relocalize_done(self, future):
        try:
            response = future.result()
            if self.verbose:
                self.get_logger().info(f"Relocalize response: success={response.success}, message={response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
        

def main(args=None):
    rclpy.init(args=args)
    goal_pose_remapper = PoseEstimateRemapper()
    try:
        rclpy.spin(goal_pose_remapper)
    except KeyboardInterrupt:
        pass
    goal_pose_remapper.destroy_node()

if __name__ == '__main__':
    main()