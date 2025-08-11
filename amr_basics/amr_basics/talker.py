import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String

class TalkerNode(Node):
    def __init__(self):
        super().__init__('amr_talker')
        
        # Define QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        # Create publisher with custom QoS
        self.publisher_ = self.create_publisher(
            String, 
            'amr_topic', 
            qos_profile
        )
        
        # Timer for periodic publishing
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        
        self.get_logger().info('AMR Talker Node Started')

    def timer_callback(self):
        msg = String()
        msg.data = f'AMR Message {self.i}: Robot status OK'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = TalkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
