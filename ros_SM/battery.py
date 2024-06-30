import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random as rnd


class Battery(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        time = self.create_timer(2.5, self.callback)
        self.publisher = self.create_publisher(String, "/battery", 10)

    def callback(self):
        msg = String()
        battery_level = rnd.randint(0, 100)
        msg.data = str(battery_level)
        self.publisher.publish(msg)

        self.get_logger().info(f"Publishing battery level: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    talker = Battery("battery")
    rclpy.spin(talker)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
