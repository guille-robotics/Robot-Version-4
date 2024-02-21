#!/usr/bin/env python3
import serial
import rclpy
from rclpy.node import Node
from rclpy.constants import S_TO_NS
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Twist, Quaternion, Vector3, TransformStamped
from tf_transformations import quaternion_from_euler
from rclpy.time import Time
import math


class SimpleSerialReceiver(Node):
    def __init__(self):
        super().__init__("simple_serial_receiver")

        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.34)

        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value

        self.get_logger().info("Using wheel radius %d" % self.wheel_radius_)
        self.get_logger().info("Using wheel separation %d" % self.wheel_separation_)

        self.declare_parameter("port", "/dev/ttyUSB1")
        self.declare_parameter("baudrate", 115200)

        self.port_ = self.get_parameter("port").value
        self.baudrate_ = self.get_parameter("baudrate").value

        self.odom_pub_ = self.create_publisher(Odometry, "odom", 10)
        self.arduino_ = serial.Serial(port=self.port_, baudrate=self.baudrate_, timeout=0.1)

        self.joint_sub= self.create_subscription(JointState,"joint_states",self.jointCallback,10)

        self.frequency_ = 0.01
        self.timer_ = self.create_timer(self.frequency_, self.timerCallback)


        self.left_wheel_prev_pos_ = 0.0
        self.right_wheel_prev_pos_ = 0.0
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0

        self.right_wheel_meas=0.0
        self.left_wheel_meas=0.0

        # Publish odometry message
        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_link"
        self.odom_msg_.pose.pose.orientation.x=0.0
        self.odom_msg_.pose.pose.orientation.y=0.0
        self.odom_msg_.pose.pose.orientation.z=0.0
        self.odom_msg_.pose.pose.orientation.w=1.0

        #Publish the TF message

        self.br_ = TransformBroadcaster(self)
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_link"

        self.prev_time_=self.get_clock().now()

    def timerCallback(self):
        while True:
            data = self.arduino_.readline().decode().strip()
            partes=data.split(',')
            if len(partes)>=2:
                self.right_wheel_meas=partes[0]
                self.left_wheel_meas=partes[1]

            #try:
                # Parse the received data
            #    right_wheel_meas_vel_ser, left_wheel_meas_vel_ser = map(int, data.split(","))
            #except ValueError:
                # Invalid data received
            #    return
    def jointCallback(self,msg):

        dp_left = msg.position[1] - self.left_wheel_prev_pos_
        dp_right = msg.position[0] - self.right_wheel_prev_pos_
        dt = Time.from_msg(msg.header.stamp) - self.prev_time_

                # Actualize the prev pose for the next itheration
        self.left_wheel_prev_pos_ = msg.position[1]
        self.right_wheel_prev_pos_ = msg.position[0]
        self.prev_time_ = Time.from_msg(msg.header.stamp)

            # Calculate estimated velocities
        self.right_wheel_est_vel = self.right_wheel_meas
        self.left_wheel_est_vel =  self.left_wheel_meas#left_wheel_meas_vel


            # Calculate linear and angular velocities
        linear = (self.right_wheel_est_vel*self.wheel_radius_ + self.left_wheel_est_vel*self.wheel_radius_) / 2
        angular = (self.right_wheel_est_vel*self.wheel_radius_ - self.left_wheel_est_vel*self.wheel_radius_) / self.wheel_separation_

            # Calculate the position increment
        d_s = (self.wheel_radius_ * dp_right + self.wheel_radius_ * dp_left) / 2
        d_theta = (self.wheel_radius_ * dp_right - self.wheel_radius_ * dp_left) / self.wheel_separation_
        self.theta_ += d_theta
        self.x_ += d_s * math.cos(self.theta_)
        self.y_ += d_s * math.sin(self.theta_)


            # Compose and publish the odom message
        q = quaternion_from_euler(0, 0, self.theta_)
        self.odom_msg_.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]
        self.odom_msg_.twist.twist.linear.x = linear
        self.odom_msg_.twist.twist.angular.z = angular
        self.odom_pub_.publish(self.odom_msg_)

            # TF
        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]
        self.transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        self.br_.sendTransform(self.transform_stamped_)


def main():
    rclpy.init()

    simple_serial_receiver = SimpleSerialReceiver()
    rclpy.spin(simple_serial_receiver)
    
    simple_serial_receiver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
