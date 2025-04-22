#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import math


class CmdVelToRpm(Node):
    def __init__(self):
        super().__init__('cmdvel_to_rpm')

        # Robot parameters (wheel separation L, wheel radius R)
        self.declare_parameter('wheel_base', 0.281)   # meters
        self.declare_parameter('wheel_radius', 0.065) # meters
        L = self.get_parameter('wheel_base').value
        R = self.get_parameter('wheel_radius').value

        # Compute conversion factor: m/s → RPM
        self.factor = 60.0 / (2.0 * math.pi * R)
        self.half_base = L / 2.0

        # Latest velocity commands
        self.latest_v = 0.0
        self.latest_w = 0.0
        self.new_command = False

        # QoS profile: BEST_EFFORT, KEEP_LAST depth=1
        # For most local/inter-node comms this is sufficient; switch to RELIABLE if drops are critical
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriber: listens on 'cmd_vel' for Twist messages
        self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmdvel_callback,
            qos_profile=qos
        )

        # Publisher: publishes wheel RPMs on 'rpm'
        self.rpm_pub = self.create_publisher(
            Int32MultiArray,
            'rpm',
            qos_profile=qos
        )

        # Timer for fixed-rate publishing at 10 Hz (0.1 s)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('RPM node initialized with fixed 10Hz publishing rate')

    def cmdvel_callback(self, msg: Twist):
        """
        Callback for incoming Twist messages.
        Stores the latest linear and angular velocities.
        """
        self.latest_v = msg.linear.x
        self.latest_w = msg.angular.z
        self.new_command = True
        self.get_logger().debug(f'Received cmd_vel: v={self.latest_v}, w={self.latest_w}')

    def timer_callback(self):
        """
        Periodic timer callback (10 Hz) to compute and publish
        left and right wheel RPMs based on the latest cmd_vel.
        """
        # Differential drive kinematics
        v_l = self.latest_v - self.half_base * self.latest_w
        v_r = self.latest_v + self.half_base * self.latest_w

        # Convert m/s to RPM and round to nearest integer
        rpm_l = round(v_l * self.factor)
        rpm_r = round(v_r * self.factor)

        # Publish as Int32MultiArray [left_rpm, right_rpm]
        arr = Int32MultiArray()
        arr.data = [rpm_l, rpm_r]
        self.rpm_pub.publish(arr)

        # Log when a new velocity command has just been handled
        if self.new_command:
            self.get_logger().debug(f'Published RPM: L={rpm_l}, R={rpm_r}')
            self.new_command = False


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToRpm()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
