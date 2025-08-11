import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String

class ListenerNode(Node):
    def __init__(self):
        super().__init__('amr_listener')
        
        # Define matching QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        # Create subscription with custom QoS
        self.subscription = self.create_subscription(
            String,
            'amr_topic',
            self.listener_callback,
            qos_profile
        )
        
        self.get_logger().info('AMR Listener Node Started')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')
        # Process the message (e.g., update robot state)

def main(args=None):
    rclpy.init(args=args)
    node = ListenerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
