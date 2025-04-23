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

        # Robot parameters
        self.declare_parameter('wheel_base',   0.281)   # meters
        self.declare_parameter('wheel_radius', 0.065)   # meters
        L = self.get_parameter('wheel_base').value
        R = self.get_parameter('wheel_radius').value

        # Conversion factor: m/s → RPM
        self.factor    = 60.0 / (2.0 * math.pi * R)
        self.half_base = L / 2.0

        # State holders
        self.latest_v    = 0.0
        self.latest_w    = 0.0

        # ---- RELIABLE QoS, KEEP_LAST depth=1 on both sides ----
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=    QoSHistoryPolicy.KEEP_LAST,
            depth=      1
        )

        # Subscribe to cmd_vel (assumes whatever teleop is publishing is RELIABLE)
        self.create_subscription(
            Twist,
            '/marl_bot1/cmd_vel',
            self.cmdvel_callback,
            qos_profile=qos
        )

        # Publish wheel RPMs
        self.rpm_pub = self.create_publisher(
            Int32MultiArray,
            '/marl_bot1/rpm',
            qos_profile=qos
        )

        # Timer @10 Hz
        self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('RPM node up @10Hz, RELIABLE QoS')

    def cmdvel_callback(self, msg: Twist):
        # Just store the very latest command—no filtering!
        self.latest_v = msg.linear.x
        self.latest_w = msg.angular.z

    def timer_callback(self):
        # Differential-drive kinematics
        v_l = self.latest_v - self.half_base * self.latest_w
        v_r = self.latest_v + self.half_base * self.latest_w

        # Straight m/s → RPM
        rpm_l = round(v_l * self.factor)
        rpm_r = round(v_r * self.factor)

        # Publish [left, right]
        arr = Int32MultiArray(data=[rpm_l, rpm_r])
        self.rpm_pub.publish(arr)

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
