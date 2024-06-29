### Implement the safety functionalities for the Robile by setting up
### a state machine and implementing all required states here

import rclpy
import smach

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
# import tf_transformations, tf2_ros, tf2_geometry_msgs
import numpy as np
import yaml
import time


# Reference: https://wiki.ros.org/smach/Tutorials/Simple%20State%20Machine [but in ROS1]

class MonitorBatteryAndCollision(smach.State):
    """State to monitor the battery level and possible collisions
    """

    def __init__(self, node):
        # TODO: define outcomes, class variables, and desired publisher/subscribers
        # YOUR CODE HERE
        smach.State.__init__(self, outcomes=["below_threshold", "collision", "monitor"],
                             output_keys=['collision_output'])
        self.node = node

        self.vel_cmd = Twist()
        self.pub_cmd_vel = self.node.create_publisher(Twist, "/cmd_vel", 10)
        time = self.node.create_timer(0.2, self.callback)
        #self.odom_data_sub = self.node.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.scan_data_sub = self.node.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.battery_sub = self.node.create_subscription(String, "/battery", self.battery_callback, 10)

        self.collision = "False"
        self.threshold = 30
        self.battery_level = 50
        self.collision_distance = 0.5

    def battery_callback(self, msg):
        self.node.get_logger().info(f"Monitor State: Getting battery level: {msg.data}")
        self.battery_level = int(msg.data)

    def scan_callback(self, data):

        self.collision = self.check_collision(data)
        self.node.get_logger().info(f"Monitor State: Getting collision data: {self.collision}")
        self.move_forward()

    def check_collision(self, data):
        for distance in data.ranges:
            if distance < self.collision_distance:
                return True
        return False

    def move_forward(self):
        self.vel_cmd.linear.x = 1.0 if not self.collision else 0.0


    # def odom_callback(self, data):
    #
    #     position = data.pose.pose.position
    #     orientation = data.pose.pose.orientation
    #
    #     self.robile_position = np.array([position.x, position.y])

    def callback(self):
        self.pub_cmd_vel.publish(self.vel_cmd)


    def execute(self, userdata):
        # TODO: implement state execution logic and return outcome
        # YOUR CODE HERE
        rclpy.spin_once(self.node)
        userdata.collision_output = self.collision

        if self.collision == "True":
            self.node.get_logger().info("Robile is about to collide...")
            return "collision"

        elif self.battery_level < self.threshold:
            self.node.get_logger().info("Battery level is low...")
            return "below_threshold"

        else:
            return "monitor"


class RotateBase(smach.State):
    """State to rotate the Robile base
    """

    def __init__(self, node):
        # TODO: define outcomes, class variables, and desired publisher/subscribers
        # YOUR CODE HERE
        smach.State.__init__(self, outcomes=["above_threshold", "monitor"])
        self.node = node
        self.battery_sub = self.node.create_subscription(String, "/battery", self.battery_callback, 10)
        self.battery_level = 10
        self.threshold = 30

    def battery_callback(self, msg):
        self.battery_level = int(msg.data)

    def execute(self, userdata):
        # TODO: implement state execution logic and return outcome
        # YOUR CODE HERE
        rclpy.spin_once(self.node)
        if self.battery_level > self.threshold:
            return "above_threshold"

        rate = self.node.create_rate(0.5)

        if self.battery_level < self.threshold:
            self.rotate_in_place()
            self.node.get_logger().info(f"RotateBase State: Rotating in place")
            # rate.sleep()
            return "monitor"

    def rotate_in_place(self):
        pass


class StopMotion(smach.State):
    """State to stop the robot's motion
    """

    def __init__(self, node):
        # TODO: define outcomes, class variables, and desired publisher/subscribers
        # YOUR CODE HERE
        smach.State.__init__(self, outcomes=["stopped"])
        self.node = node

    def execute(self, userdata):
        # TODO: implement state execution logic and return outcome
        # YOUR CODE HERE
        rclpy.spin_once(self.node)
        self.stop_motion()
        self.node.get_logger().info("Robile is stopped...")
        return "stopped"

    def stop_motion(self):
        pass


# TODO: define any additional states if necessary
class ManuallyMoveToSafeDistance(smach.State):
    """State to stop the robot's motion
    """

    def __init__(self, node):
        # TODO: define outcomes, class variables, and desired publisher/subscribers
        # YOUR CODE HERE
        smach.State.__init__(self, outcomes=["safe_distance"], input_keys=['collision_input'])
        self.node = node
        self.safe_distance = False

    def execute(self, userdata):
        # TODO: implement state execution logic and return outcome
        # YOUR CODE HERE
        rclpy.spin_once(self.node)

        if userdata.collision_input == "True":
            return self.move_to_safe_distance(userdata)

        elif userdata.collision_input == "False":
            return self.set_safe_distance(userdata)

    def move_to_safe_distance(self, userdata):
        self.node.get_logger().info("Moving to a safe distance...")
        return self.set_safe_distance(userdata)

    def set_safe_distance(self, userdata):
        self.node.get_logger().info("Robile is now in a safe distance...")
        self.safe_distance = True
        return "safe_distance"


def main(args=None):
    """Main function to initialise and execute the state machine
    """

    # TODO: initialise a ROS2 node, set any threshold values, and define the state machine
    # YOUR CODE HERE
    rclpy.init(args=args)

    node = rclpy.create_node("robot_sm")
    # Create a SMACH state machine

    state_machine = smach.StateMachine(outcomes=['robot_sm'])
    state_machine.userdata.collision = "False"

    # Open the container
    # Add states to the container

    with state_machine:
        smach.StateMachine.add('Monitor_Battery_And_Collision', MonitorBatteryAndCollision(node),
                               transitions={'below_threshold': 'Rotate_Base',
                                            'collision': 'Stop_Motion',
                                            'monitor': 'Monitor_Battery_And_Collision'},
                               remapping={'collision_output': 'collision'})

        smach.StateMachine.add('Rotate_Base', RotateBase(node),
                               transitions={'above_threshold': 'Monitor_Battery_And_Collision',
                                            'monitor': 'Monitor_Battery_And_Collision'})

        smach.StateMachine.add('Stop_Motion', StopMotion(node),
                               transitions={'stopped': 'Manually_Move_To_Safe_Distance'})

        smach.StateMachine.add('Manually_Move_To_Safe_Distance', ManuallyMoveToSafeDistance(node),
                               transitions={'safe_distance': 'Monitor_Battery_And_Collision'},
                               remapping={'collision_input': 'collision'})

        outcome = state_machine.execute()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
