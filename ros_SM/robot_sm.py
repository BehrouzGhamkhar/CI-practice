### Implement the safety functionalities for the Robile by setting up
### a state machine and implementing all required states here

import rclpy
import smach

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from enum import Enum
# import tf_transformations, tf2_ros, tf2_geometry_msgs
import numpy as np
import yaml
import time


# Reference: https://wiki.ros.org/smach/Tutorials/Simple%20State%20Machine [but in ROS1]

class MotionType(Enum):
    stop = 1
    move_forward = 2
    rotate_in_place = 3
    move_to_safe = 4


class MonitorBatteryAndCollision(smach.State):
    """State to monitor the battery level and possible collisions
    """

    def __init__(self, node):
        # TODO: define outcomes, class variables, and desired publisher/subscribers

        smach.State.__init__(self, outcomes=["below_threshold", "collision", "move"],
                             output_keys=['collision_output', 'battery_output'])
        self.node = node

        # self.odom_data_sub = self.node.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.scan_data_sub = self.node.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.battery_sub = self.node.create_subscription(String, "/battery", self.battery_callback, 10)

        self.collision_pub = self.node.create_publisher(String, "/collision", 10)
        self.collision = False
        self.threshold = 30
        self.battery_level = 50
        self.collision_distance = 0.4

    def battery_callback(self, msg):
        self.node.get_logger().info(f"Monitor State: Getting battery level: {msg.data}")
        self.battery_level = int(msg.data)

    def scan_callback(self, data):
        msg = String()
        self.collision = self.check_collision(data)
        msg.data = str(self.collision)
        self.collision_pub.publish(msg)
        self.node.get_logger().info(f"Monitor State: Getting collision data: {self.collision}")

    def check_collision(self, data):
        for distance in data.ranges:
            if distance < self.collision_distance:
                return True
        return False

    def execute(self, userdata):
        # TODO: implement state execution logic and return outcome

        rclpy.spin_once(self.node)
        userdata.collision_output = self.collision
        userdata.battery_output = self.battery_level

        if self.collision:
            self.node.get_logger().info("Robile is about to collide...")
            return "collision"

        elif self.battery_level < self.threshold:
            self.node.get_logger().info("Battery level is low...")
            return "below_threshold"

        else:
            temp_vel = Twist()
            temp_vel.linear.x = 1.0
            userdata.cmd_vel_output = temp_vel
            self.node.get_logger().info("Robile is Moving...")
            return "move"


class ApplyMotion(smach.State):
    """State to stop the robot's motion
    """

    def __init__(self, node):
        smach.State.__init__(self, outcomes=["monitor"])
        self.node = node
        self.vel_cmd = Twist()
        self.pub_cmd_vel = self.node.create_publisher(Twist, "/cmd_vel", 10)

    def execute(self, userdata):
        rclpy.spin_once(self.node)
        self.vel_cmd.linear.x = 1.0
        self.pub_cmd_vel.publish(self.vel_cmd)
        return "monitor"


class RotateBase(smach.State):
    """State to rotate the Robile base
    """

    def __init__(self, node):
        # TODO: define outcomes, class variables, and desired publisher/subscribers

        smach.State.__init__(self, outcomes=["above_threshold", "monitor"],
                             input_keys=["battery_input"])
        self.node = node
        self.pub_cmd_vel = self.node.create_publisher(Twist, "/cmd_vel", 10)

        self.threshold = 30
        self.vel_cmd = Twist()

    def execute(self, userdata):
        # TODO: implement state execution logic and return outcome

        rclpy.spin_once(self.node)

        if userdata.battery_input > self.threshold:
            return "above_threshold"

        else:
            self.node.get_logger().info(f"RotateBase State: Rotating in place")
            self.rotate_in_place()
            return "monitor"

    def rotate_in_place(self):
        self.vel_cmd.angular.z = -2.0
        self.vel_cmd.linear.x = 0.0
        self.pub_cmd_vel.publish(self.vel_cmd)


class StopMotion(smach.State):
    """State to stop the robot's motion
    """

    def __init__(self, node):
        # TODO: define outcomes, class variables, and desired publisher/subscribers

        smach.State.__init__(self, outcomes=["stopped"])
        self.node = node
        self.pub_cmd_vel = self.node.create_publisher(Twist, "/cmd_vel", 10)

    def execute(self, userdata):
        # TODO: implement state execution logic and return outcome

        rclpy.spin_once(self.node)
        self.stop_motion()
        self.node.get_logger().info("Robile is stopped...")
        return "stopped"

    def stop_motion(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        self.pub_cmd_vel.publish(cmd_vel)


# TODO: define any additional states if necessary
class ManuallyMoveToSafeDistance(smach.State):
    """State to stop the robot's motion
    """

    def __init__(self, node):
        # TODO: define outcomes, class variables, and desired publisher/subscribers

        smach.State.__init__(self, outcomes=["safe_distance"], input_keys=['collision_input'])
        self.node = node
        self.collision = False
        self.pub_cmd_vel = self.node.create_publisher(Twist, "/cmd_vel", 10)
        self.sub_collision = self.node.create_subscription(String, "/collision", self.collision_callback, 10)
        self.cmd_vel = Twist()

    def collision_callback(self, msg):
        self.node.get_logger().info(f"Monitor State: Getting collision data: {msg.data}")
        self.collision = bool(msg.data)

    def execute(self, userdata):
        # TODO: implement state execution logic and return outcome

        rclpy.spin_once(self.node)

        if self.collision:
            return self.move_to_safe_distance(userdata)

        elif not self.collision:
            return self.set_safe_distance(userdata)

    def move_to_safe_distance(self, userdata):
        self.node.get_logger().info("Moving to a safe distance...")

        # move back a bit
        self.cmd_vel.linear.x = - 1.0
        self.pub_cmd_vel.publish(self.cmd_vel)
        rclpy.spin_once(self.node, timeout_sec=1.0)

        self.stop_robile()

        # Rotate to the right
        self.cmd_vel.angular.z = -1.5
        self.pub_cmd_vel.publish(self.cmd_vel)
        rclpy.spin_once(self.node, timeout_sec=1.0)

        self.stop_robile()
        return self.set_safe_distance(userdata)

    def stop_robile(self):
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 0.0
        self.pub_cmd_vel.publish(self.cmd_vel)
        rclpy.spin_once(self.node, timeout_sec=0.5)

    def set_safe_distance(self, userdata):
        self.node.get_logger().info("Robile is now in a safe distance...")
        return "safe_distance"


def main(args=None):
    """Main function to initialise and execute the state machine
    """

    # TODO: initialise a ROS2 node, set any threshold values, and define the state machine

    rclpy.init(args=args)

    node = rclpy.create_node("robot_sm")
    # Create a SMACH state machine

    state_machine = smach.StateMachine(outcomes=['robot_sm'])
    state_machine.userdata.collision = False
    state_machine.userdata.battery = 100

    # Open the container
    # Add states to the container

    with state_machine:
        smach.StateMachine.add('Monitor_Battery_And_Collision', MonitorBatteryAndCollision(node),
                               transitions={'below_threshold': 'Rotate_Base',
                                            'collision': 'Stop_Motion',
                                            'move': 'Apply_Motion'},
                               remapping={'collision_output': 'collision',
                                          'battery_output': 'battery'})

        smach.StateMachine.add('Apply_Motion', ApplyMotion(node),
                               transitions={'monitor': 'Monitor_Battery_And_Collision'})

        smach.StateMachine.add('Rotate_Base', RotateBase(node),
                               transitions={'above_threshold': 'Monitor_Battery_And_Collision',
                                            'monitor': 'Monitor_Battery_And_Collision'},
                               remapping={'battery_input': 'battery'})

        smach.StateMachine.add('Stop_Motion', StopMotion(node),
                               transitions={'stopped': 'Manually_Move_To_Safe_Distance'})

        smach.StateMachine.add('Manually_Move_To_Safe_Distance', ManuallyMoveToSafeDistance(node),
                               transitions={'safe_distance': 'Monitor_Battery_And_Collision'},
                               remapping={'collision_input': 'collision'})

        outcome = state_machine.execute()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
