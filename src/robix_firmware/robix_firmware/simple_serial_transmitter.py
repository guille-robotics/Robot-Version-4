#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import TwistStamped
import serial
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float64MultiArray

class SuscriberCmd(Node):
    def __init__(self):
        super().__init__("suscriber_cmd")
        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.34)

        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value

        self.get_logger().info("Using wheel radius %d" % self.wheel_radius_)
        self.get_logger().info("Using wheel separation %d" % self.wheel_separation_)

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)

        self.port_ = self.get_parameter("port").value
        self.baudrate_ = self.get_parameter("baudrate").value

        self.arduino_ = serial.Serial(port=self.port_, baudrate=self.baudrate_, timeout=0.1)
        self.cmd_vel_subscription = self.create_subscription(TwistStamped,'robix_controller/cmd_vel',self.cmd_vel_callback,10)

        self.speed_conversion_ = np.array([[self.wheel_radius_/2, self.wheel_radius_/2],
                                           [self.wheel_radius_/self.wheel_separation_, -self.wheel_radius_/self.wheel_separation_]])
        self.get_logger().info("The conversion matrix is %s" % self.speed_conversion_)

    def cmd_vel_callback(self, msg):
        # Implements the differential kinematic model
        # Given v and w, calculate the velocities of the wheels
        robot_speed = np.array([[msg.twist.linear.x],
                                [msg.twist.angular.z]])
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion_), robot_speed) 

        # Construir el mensaje a enviar al Arduino
        if wheel_speed[1, 0] > 0:
            right_wheel_cmd = 'p'
        else:
            right_wheel_cmd = 'n'

        if wheel_speed[0, 0] > 0:
            left_wheel_cmd = 'p'
        else:
            left_wheel_cmd = 'n'

        serial_data = "r{}{:.2f},l{}{:.2f},".format(right_wheel_cmd, abs(wheel_speed[1, 0]), left_wheel_cmd, abs(wheel_speed[0, 0]))
        self.get_logger().info("The data is r{:.2f} ({}) and l{:.2f}({})".format(abs(wheel_speed[1, 0]),right_wheel_cmd,abs(wheel_speed[0, 0]),left_wheel_cmd))

        # Enviar datos al Arduino a través de la conexión serial
        self.arduino_.write(serial_data.encode())


def main():
    rclpy.init()
    suscriber_cmd = SuscriberCmd()
    rclpy.spin(suscriber_cmd)
    suscriber_cmd.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
