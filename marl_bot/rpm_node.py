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

        # ─── ROBOT GEOMETRY PARAMETERS ─────────────────────────────────────────
        # Wheel‐base (distance between wheel centers) in meters
        self.declare_parameter('wheel_base',   0.281)
        # Wheel radius in meters (65 mm diameter → 0.065 m radius)
        self.declare_parameter('wheel_radius', 0.065)
        L = self.get_parameter('wheel_base').value
        R = self.get_parameter('wheel_radius').value

        # Precompute for m/s → RPM: factor = 60 / (2πR)
        self.factor    = 60.0 / (2.0 * math.pi * R)
        # Half of wheel base, for kinematics
        self.half_base = L / 2.0

        # ─── DEAD-BAND & SPEED MULTIPLIER ──────────────────────────────────────
        # Any nonzero RPM with |RPM|<100 will be bumped to ±100
        self.deadband_rpm    = 150
        # Always double the commanded velocities
        self.speed_multiplier = 2.0

        # ─── STATE HOLDERS ─────────────────────────────────────────────────────
        self.latest_v = 0.0   # last linear.x
        self.latest_w = 0.0   # last angular.z

        # ─── QoS FOR RELIABLE COMMUNICATION ────────────────────────────────────
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribe to cmd_vel
        self.create_subscription(
            Twist,
            '/marl_bot1/cmd_vel',
            self.cmdvel_callback,
            qos_profile=qos
        )
        # Publisher for [left_rpm, right_rpm]
        self.rpm_pub = self.create_publisher(
            Int32MultiArray,
            '/marl_bot1/rpm',
            qos_profile=qos
        )

        # Timer at 10 Hz to compute & publish RPM
        self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('RPM node up @10Hz, RELIABLE QoS')

    def cmdvel_callback(self, msg: Twist):
        # Store the most recent command
        self.latest_v = msg.linear.x
        self.latest_w = msg.angular.z

    def apply_deadband(self, rpm: float) -> int:
        """
        If 0 < |rpm| < deadband_rpm, bump to ±deadband_rpm.
        Zero remains zero.
        """
        if rpm == 0.0:
            return 0
        if abs(rpm) < self.deadband_rpm:
            return int(math.copysign(self.deadband_rpm, rpm))
        return int(round(rpm))

    def timer_callback(self):
        # 1) DOUBLE the commanded velocities
        v = self.speed_multiplier * self.latest_v
        w = self.speed_multiplier * self.latest_w

        # 2) Differential-drive kinematics → left/right linear speeds
        v_l = v - self.half_base * w
        v_r = v + self.half_base * w

        # 3) Convert m/s → raw RPM
        raw_rpm_l = v_l * self.factor
        raw_rpm_r = v_r * self.factor

        # 4) Apply ±100 RPM deadband
        rpm_l = self.apply_deadband(raw_rpm_l)
        rpm_r = self.apply_deadband(raw_rpm_r)

        # 5) Publish as Int32MultiArray [left, right]
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
