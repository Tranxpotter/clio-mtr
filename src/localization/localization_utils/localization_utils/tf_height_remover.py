'''
TfHeightRemover Node
This node publishes a new tf frame derived from an input tf frame that has the same z transform value as the world frame

Parameters
-----------
world_frame: `str`
    The world frame to align the height of input_frame to. Default: map
input_frame: `str`
    The tf frame to listen to. Default: body
output_frame: `str`
    The name of the processed tf frame. Default: base_footprint
z_extra_offset: `float`
    Extra offset as buffer for not letting output tf < 0.0. Default: 0.0
verbose: `bool`
    Log tf transformer
'''


from tf2_ros import TransformBroadcaster, Buffer, TransformListener, TransformStamped
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

class TfHeightRemover(Node):
    def __init__(self):
        super().__init__('tf_height_remover')
        
        # Parameters
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('input_frame', 'body')
        self.declare_parameter('output_frame', 'base_footprint')
        self.declare_parameter('z_extra_offset', 0.0)
        self.declare_parameter('verbose', True)

        self.world_frame = self.get_parameter('world_frame').get_parameter_value().string_value
        self.input_frame = self.get_parameter('input_frame').get_parameter_value().string_value
        self.output_frame = self.get_parameter('output_frame').get_parameter_value().string_value
        self.z_extra_offset = self.get_parameter('z_extra_offset').get_parameter_value().double_value
        self.verbose = self.get_parameter('verbose').get_parameter_value().bool_value

        if self.verbose:
            self.get_logger().info(f"Loaded parameters: {self.world_frame=} {self.input_frame=} {self.output_frame=} {self.z_extra_offset=}")
    
        # TF Buffer and Listener with longer cache time
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        

        self.timer = self.create_timer(0.1, self.on_timer)

    def on_timer(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.input_frame,
                Time())
        except Exception as ex:
            if self.verbose:
                self.get_logger().info(
                    f'Could not transform {self.world_frame} to {self.input_frame}: {ex}')
            return
        
        # Get height diff
        z = t.transform.translation.z
        if self.verbose:
            self.get_logger().info(f"Obtained z diff = {z}")

        # Create new frame
        new_t = TransformStamped()
        new_t.header.frame_id = t.child_frame_id
        new_t.header.stamp = self.get_clock().now().to_msg()
        new_t.child_frame_id = self.output_frame

        new_t.transform.translation.x = 0.0
        new_t.transform.translation.y = 0.0
        new_t.transform.translation.z = -z - self.z_extra_offset
        new_t.transform.rotation.w = 1.0
        new_t.transform.rotation.x = 0.0
        new_t.transform.rotation.y = 0.0
        new_t.transform.rotation.z = 0.0

        self.tf_broadcaster.sendTransform(new_t)

        if self.verbose:
            self.get_logger().info(f"Published transform from {self.input_frame} to {self.output_frame} with z={new_t.transform.translation.z} sec={new_t.header.stamp.sec} nanosec={new_t.header.stamp.nanosec}")

            

def main(args=None):
    rclpy.init(args=args)
    tf_height_remover = TfHeightRemover()
    try:
        rclpy.spin(tf_height_remover)
    except KeyboardInterrupt as e:
        pass
    tf_height_remover.destroy_node()
    rclpy.shutdown()
