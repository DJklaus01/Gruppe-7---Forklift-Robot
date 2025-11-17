#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class TwistToFwdRearGroup(Node):
    def __init__(self):
        super().__init__('twist_to_fwdrear_group')

        # Parametre (kan overstyres i launch)
        self.declare_parameter('wheel_radius', 0.08)   # m
        self.declare_parameter('wheelbase', 0.45)      # m
        self.declare_parameter('max_steer', 0.6)       # rad
        self.declare_parameter('max_wheel_speed', 30.0)# rad/s
        self.declare_parameter('vel_topic', '/front_wheel_velocity_controller/commands')
        self.declare_parameter('steer_topic', '/back_steer_position_controller/commands')
        # Noen koblinger har motsatt sign på høyre/venstre; justér ved behov:
        self.declare_parameter('steer_sign_left', 1.0)
        self.declare_parameter('steer_sign_right', 1.0)

        self.r = float(self.get_parameter('wheel_radius').value)
        self.L = float(self.get_parameter('wheelbase').value)
        self.max_steer = float(self.get_parameter('max_steer').value)
        self.max_w = float(self.get_parameter('max_wheel_speed').value)
        vel_topic = self.get_parameter('vel_topic').value
        steer_topic = self.get_parameter('steer_topic').value
        self.sL = float(self.get_parameter('steer_sign_left').value)
        self.sR = float(self.get_parameter('steer_sign_right').value)

        self.pub_vel = self.create_publisher(Float64MultiArray, vel_topic, 10)
        self.pub_str = self.create_publisher(Float64MultiArray, steer_topic, 10)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cb, 20)

        self.get_logger().info(f'Lytter på /cmd_vel → {vel_topic} (vel), {steer_topic} (steer)')

    def sat(self, x, lim): return max(-lim, min(lim, x))

    def cb(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        if self.r <= 0.0:
            self.get_logger().warn('wheel_radius <= 0')
            return

        # Forhjul: like hjulhastigheter (forhjulsdrift)
        w_wheel = self.sat(v / self.r, self.max_w)

        # Bakhjulsstyring (enkel sykkelmodell). Når v≈0, la rattet vinkles proporsjonalt med w.
        eps = 1e-3
        if abs(v) > eps:
            steer = math.atan(self.L * w / v)
        else:
            steer = self.sat(0.5 * w, self.max_steer)

        steer = self.sat(steer, self.max_steer)

        # Send i samme rekkefølge som YAML-listene dine:
        vel_msg = Float64MultiArray(); vel_msg.data = [w_wheel, w_wheel]  # [FL, FR]
        str_msg = Float64MultiArray(); str_msg.data = [self.sL * steer, self.sR * steer]  # [BL, BR]

        self.pub_vel.publish(vel_msg)
        self.pub_str.publish(str_msg)

def main():
    rclpy.init()
    rclpy.spin(TwistToFwdRearGroup())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
