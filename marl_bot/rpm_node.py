#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray

class CmdVelToRpm(Node):
    def __init__(self):
        super().__init__('cmdvel_to_rpm')
        # Robot parameters (wheel separation L, wheel radius R)
        self.declare_parameter('wheel_base', 0.281)    # meters
        self.declare_parameter('wheel_radius', 0.065)  # meters
        L = self.get_parameter('wheel_base').value
        R = self.get_parameter('wheel_radius').value

        # Compute conversion factor: m/s → RPM
        self.factor = 60.0 / (2.0 * 3.141592653589793 * R)
        self.half_base = L / 2.0

        # Subscription
        self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmdvel_callback,
            qos_profile=10
        )

        # Publisher
        self.rpm_pub = self.create_publisher(
            Int32MultiArray,
            'rpm',
            qos_profile=10
        )

        self.timer = self.create_timer(0.1, self.timer_callback)

    def cmdvel_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z
        v_l = v - self.half_base * w
        v_r = v + self.half_base * w
        rpm_l = round(v_l * self.factor)
        rpm_r = round(v_r * self.factor)

        arr = Int32MultiArray()
        arr.data = [rpm_l, rpm_r]
        self.rpm_pub.publish(arr)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToRpm()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
