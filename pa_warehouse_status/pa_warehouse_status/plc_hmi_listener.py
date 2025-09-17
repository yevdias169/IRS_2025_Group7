import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class WarehouseEavesDropper(Node):
    def __init__(self):
        super().__init__('warehouse_eavesdropper')

        self.subscription = self.create_subscription(
            String,
            '/hmi/unified_status',
            self.listener_callback,
            10
        )


    def listener_callback(self,msg):
        self.get_logger().info(f'pss, listen up: "{msg.data}"')

def main (args=None):
    rclpy.init(args=args)
    warehouse_eavesdropper = WarehouseEavesDropper()
    rclpy.spin(warehouse_eavesdropper)
    warehouse_eavesdropper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()