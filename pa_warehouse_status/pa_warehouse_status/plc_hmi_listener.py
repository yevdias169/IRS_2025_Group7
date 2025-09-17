import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class PLC_HMI_LISTENER(Node):
    def __init__(self):
        super().__init__('plc_hmi_listener')

        self.subscription = self.create_subscription(
            String,
            '/hmi/unified_status',
            self.listener_callback,
            10
        )


    def listener_callback(self,msg):
        try:
            data = json.loads(msg.data)
            stamp = data["stamp"]
            box = data["box"]
            counts = data["counts"]

            print("📥 Received PLC status:")
            print(f" ⏱ Time: {stamp['sec']}.{stamp['nanosec']}")
            print(f" 📦📦 Box weight raw={box['weight_raw']}")
            print(f" 📍📍 Location: {box['location']}")
            print(f" 🔢🔢 Counts: big={counts['big']}, medium={counts['medium']}, "
                f"small={counts['small']}, total={counts['total']}")
            print() 
        except Exception as e:
            self.get_logger().erro(f"Failed to parse JSON: (e) \nRaw msg={msg.data}")


def main (args=None):
    rclpy.init(args=args)
    plc_hmi_listener = PLC_HMI_LISTENER()
    rclpy.spin(plc_hmi_listener)
    plc_hmi_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()