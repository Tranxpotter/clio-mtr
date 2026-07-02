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
from std_msgs.msg import String, Empty

class PoseEstimateRemapper(Node):
    def __init__(self):
        super().__init__('pose_estimate_remapper')
        
        # Parameters
        self.declare_parameter('map_path', 'maps/map.pcd')
        self.declare_parameter('graph_path', 'graphs/graph.vgh')
        self.declare_parameter('verbose', True)
        self.map_path = self.get_parameter('map_path').get_parameter_value().string_value
        self.graph_path = self.get_parameter('graph_path').get_parameter_value().string_value
        self.verbose = self.get_parameter('verbose').get_parameter_value().bool_value
    
        self.delay_timer = None
        
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose",
            self.initial_pose_callback,
            10)
        self.relocalize_client = self.create_client(Relocalize, "/localizer/relocalize")
        self.graph_reset_publisher = self.create_publisher(Empty, "/reset_visibility_graph", 5)
        self.graph_path_publisher = self.create_publisher(String, "/read_file_dir", 5)
        

    def initial_pose_callback(self, msg:PoseWithCovarianceStamped):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        x, y, z = position.x, position.y, position.z
        ox, oy, oz, ow = orientation.x, orientation.y, orientation.z, orientation.w

        # Convert quaternion to roll, pitch, yaw
        roll = math.atan2(2 * (ow * ox + oy * oz), 1 - 2 * (ox * ox + oy * oy))
        pitch = math.asin(2 * (ow * oy - oz * ox))
        yaw = math.atan2(2 * (ow * oz + ox * oy), 1 - 2 * (oy * oy + oz * oz))

        request = Relocalize.Request()
        request.pcd_path = self.map_path
        request.x, request.y, request.z = x, y, z
        request.yaw, request.pitch, request.roll = yaw, pitch, roll

        future = self.relocalize_client.call_async(request)
        if self.verbose:
            self.get_logger().info(f"Sending relocalize request: {x=} {y=} {z=} {yaw=} {pitch=} {roll=}")
        future.add_done_callback(self.on_relocalize_done)


        if self.verbose:
            self.get_logger().info("Sending graph reset message.")
        reset_msg = Empty()
        self.graph_reset_publisher.publish(reset_msg)

        if self.delay_timer is not None:
            self.delay_timer.cancel()
            self.destroy_timer(self.delay_timer)
        
        self.delay_timer = self.create_timer(1.0, self.graph_path_timer_callback)

        
    

    def graph_path_timer_callback(self):
        if self.delay_timer is not None:
            self.delay_timer.cancel()
            self.destroy_timer(self.delay_timer)
            self.delay_timer = None
        
        if self.verbose:
            self.get_logger().info(f"Sending graph path message: {self.graph_path=}")
        graph_msg = String()
        graph_msg.data = self.graph_path
        self.graph_path_publisher.publish(graph_msg)


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