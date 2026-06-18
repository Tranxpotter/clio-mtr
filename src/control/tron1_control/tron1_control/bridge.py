import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import String
import websocket
import json
import threading
import time
import uuid

class Tron1Bridge(Node):
    def __init__(self):
        super().__init__('tron1_bridge')

        # === CONFIGURATION ===
        self.declare_parameter('robot_ip', '10.192.1.2')
        self.declare_parameter('robot_port', 5000) 
        self.declare_parameter('accid', 'WF_TRON1A_212')

        self.robot_ip = self.get_parameter('robot_ip').value
        self.robot_port = self.get_parameter('robot_port').value
        self.accid = self.get_parameter('accid').value
        
        self.ws_url = f"ws://{self.robot_ip}:{self.robot_port}"
        self.get_logger().info(f"Connecting to {self.ws_url} with AccID: {self.accid}")

        # === WEBSOCKET SETUP ===
        self.ws = None
        self.ws_connected = False
        self.ws_thread = threading.Thread(target=self.websocket_loop)
        self.ws_thread.daemon = True
        self.ws_thread.start()

        # === SUBSCRIBERS ===
        self.sub_vel = self.create_subscription(TwistStamped, '/cmd_vel_smoothed', self.cmd_vel_callback, 10)
        self.sub_mode = self.create_subscription(String, '/tron1/mode', self.mode_callback, 10)

    def websocket_loop(self):
        while rclpy.ok():
            try:
                self.ws = websocket.WebSocketApp(
                    self.ws_url,
                    on_open=self.on_open,
                    on_error=self.on_error,
                    on_close=self.on_close
                )
                self.ws.run_forever()
                time.sleep(2) 
            except Exception as e:
                self.get_logger().error(f"Connection failed: {e}")
                time.sleep(5)

    def on_open(self, ws):
        self.get_logger().info("WebSocket Connected")
        self.ws_connected = True

    def on_error(self, ws, error):
        self.get_logger().error(f"WebSocket Error: {error}")

    def on_close(self, ws, status, msg):
        self.get_logger().warn("WebSocket Closed")
        self.ws_connected = False

    def send_packet(self, title, data_content={}):
        """Generates the specific TRON 1 JSON protocol wrapper."""
        # self.get_logger().info(f"Sending title: {title} , data: {data_content}")
        if not self.ws_connected:
            return

        payload = {
            "accid": self.accid,
            "title": title,
            "timestamp": int(time.time() * 1000), # Unix ms
            "guid": uuid.uuid4().hex,             # Random UUID
            "data": data_content
        }

        try:
            self.ws.send(json.dumps(payload))
            # self.get_logger().info("Send successful")
        except Exception as e:
            self.get_logger().error(f"Send failed: {e}")

    def mode_callback(self, msg):
        """
        Switches modes. 
        Input examples: "walk", "stand", "rest"
        """
        mode = msg.data.lower()
        
        # Mapping ROS strings to Protocol Titles
        if mode == "walk":
            self.send_packet("request_walk_mode")
        elif mode == "stand":
            self.send_packet("request_stand_mode") 
        elif mode == "sit":
            self.send_packet("request_sitdown") 
        elif mode == "stop":
            self.send_packet("request_emgy_stop")
        else:
            self.get_logger().warn(f"Unknown mode: {mode}")

    def cmd_vel_callback(self, msg:TwistStamped):
        """
        Sends velocity commands.
        """
        # data = {
        #     "x": msg.linear.x,      # Forward/Back
        #     "y": msg.linear.y,      # Left/Right (Strafe)
        #     "z": msg.angular.z      # Rotation
        # }

        # For TwistStamped
        data = {
            "x": msg.twist.linear.x,      # Forward/Back
            "y": msg.twist.linear.y,      # Left/Right (Strafe)
            "z": msg.twist.angular.z      # Rotation
        }
        
        self.send_packet("request_twist", data)

def main(args=None):
    rclpy.init(args=args)
    node = Tron1Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ws:
            node.ws.close()
        node.destroy_node()

if __name__ == '__main__':
    main()