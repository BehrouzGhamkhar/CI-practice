import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random as rnd


class Collision(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        time = self.create_timer(2.5, self.callback)
        self.publisher = self.create_publisher(String, "/collision", 10)

    def callback(self):
        msg = String()
        collision = rnd.choice(["True", "False"])
        msg.data = collision
        self.publisher.publish(msg)

        self.get_logger().info(f"Publishing collision: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    talker = Collision("collision")
    rclpy.spin(talker)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
