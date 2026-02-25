import sys
import termios
import tty
import select
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

# Instructions displayed to the user hello
msg = """
--------------------------------
   PX4 Key Teleop Controller
--------------------------------oiuhkuibgjytfhdf
   Moving (Step: 1m):
   w: Forward (x+)
   s: Backward (x-)
   a: Left    (y+)
   d: Right   (y-)

   Altitude (Step: 1m):
   q: Up      (z+)
   e: Down    (z-)

   Commands:
   u: ARM
   i: OFFBOARD Mode
   o: LAND Mode
   p: DISARM (Emergency)
   
   CTRL+C to quit
--------------------------------
"""

class TeleopNode(Node):
    def __init__(self):
        super().__init__('px4_teleop_node')

        # QoS Settings for MAVROS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publisher for Position
        self.local_pos_pub = self.create_publisher(
            PoseStamped, 
            '/mavros/setpoint_position/local', 
            10)

        # Subscriber for State
        self.state_sub = self.create_subscription(
            State, 
            '/mavros/state', 
            self.state_cb, 
            qos_profile)

        # Services
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Current State
        self.current_state = State()
        
        # Current Target Position (Initialized to ground 0,0,0)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        # Run the publisher loop at 10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

    def state_cb(self, msg):
        self.current_state = msg

    def timer_callback(self):
        # Always publish the current target position
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = self.z
        
        self.local_pos_pub.publish(pose)

    # Helper functions to call services from the main loop
    def set_arm(self, armed):
        if self.arming_client.wait_for_service(timeout_sec=1.0):
            req = CommandBool.Request()
            req.value = armed
            self.arming_client.call_async(req)
            self.get_logger().info(f"Arming request sent: {armed}")

    def set_mode(self, mode):
        if self.set_mode_client.wait_for_service(timeout_sec=1.0):
            req = SetMode.Request()
            req.custom_mode = mode
            self.set_mode_client.call_async(req)
            self.get_logger().info(f"Mode request sent: {mode}")

# Function to read one keypress without pressing Enter
def get_key():
    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

    # Spin ROS in a separate thread so it doesn't block the keyboard loop
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    try:
        print(msg)
        while rclpy.ok():
            key = get_key()
            
            if key == 'w':
                node.x += 1.0
                print(f"Target: X={node.x}, Y={node.y}, Z={node.z}")
            elif key == 's':
                node.x -= 1.0
                print(f"Target: X={node.x}, Y={node.y}, Z={node.z}")
            elif key == 'a':
                node.y += 1.0
                print(f"Target: X={node.x}, Y={node.y}, Z={node.z}")
            elif key == 'd':
                node.y -= 1.0
                print(f"Target: X={node.x}, Y={node.y}, Z={node.z}")
            elif key == 'q':
                node.z += 1.0
                print(f"Target: X={node.x}, Y={node.y}, Z={node.z}")
            elif key == 'e':
                node.z -= 1.0
                print(f"Target: X={node.x}, Y={node.y}, Z={node.z}")

            elif key == 'u':
                node.set_arm(True)
            elif key == 'p':
                node.set_arm(False)
            elif key == 'i':
                node.set_mode("OFFBOARD")
            elif key == 'o':
                node.set_mode("AUTO.LAND")

            elif key == '\x03': # CTRL+C
                break

    except Exception as e:
        print(e)

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()