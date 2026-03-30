#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

from vs_msgs.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy


class ParkingController(Node):
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """

    def __init__(self):
        super().__init__("parking_controller")

        self.declare_parameter("drive_topic", "/drive")
        self.declare_parameter("line_follower", False)
        DRIVE_TOPIC = self.get_parameter("drive_topic").value
        self.line_follower = self.get_parameter("line_follower").value

        self.drive_pub = self.create_publisher(AckermannDriveStamped, DRIVE_TOPIC, 10)
        self.error_pub = self.create_publisher(ParkingError, "/parking_error", 10)

        self.create_subscription(
            ConeLocation, "/relative_cone", self.relative_cone_callback, 1)
        self.create_subscription(
            Joy, "/vesc/joy", self.joy_callback, 1)

        self.rb_held = False
        self.parking_distance = 0.3  # meters; try playing with this number!
        self.relative_x = 0
        self.relative_y = 0

        self.speed = 1.0 # hardcoded max speed
        self.acceptable_distance_error = self.speed * 0.08 # can change this
        self.acceptable_angle_error = 0.05 # can change this
        self.max_angle = 0.4 # can change this
        self.previous_steering_angle = None

        self.get_logger().info("Parking Controller Initialized")

    def joy_callback(self, msg):
        # RB is typically button index 5
        self.rb_held = msg.buttons[5] == 1 if len(msg.buttons) > 5 else False

    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        drive_cmd = AckermannDriveStamped()
        # self.get_logger().info(f"msg: {msg}")

        #################################

        # YOUR CODE HERE
        # Use relative position and your control law to set drive_cmd

        #################################
        relative_distance = np.sqrt(self.relative_x ** 2 + self.relative_y ** 2)
        distance_error = relative_distance - self.parking_distance
        relative_angle = np.arctan2(self.relative_y, self.relative_x)
        steering_angle = min(np.abs(relative_angle), self.max_angle) * (relative_angle / np.abs(relative_angle))

        velocity = self.speed
        if np.abs(relative_angle) < self.max_angle: # if target is within front cone
            if np.abs(distance_error) < self.acceptable_distance_error and np.abs(relative_angle) < self.acceptable_angle_error:
                # stop if both distance and angle errors are acceptable
                velocity *= 0
            elif distance_error < 0:
                # back up if target is too close in front
                velocity *= -1
            else:
                # go forward if target is far in front
                velocity *= 1
            # slow down when close to target:
            if np.abs(distance_error) < self.acceptable_distance_error * 5.0:
                velocity *= np.abs(distance_error) / (self.acceptable_distance_error * 5.0)
        elif np.abs(relative_angle) > np.pi - self.max_angle: # if target is within back cone
            # defaults to driving foward, causing the car to circle until the target is within the front cone
            velocity *= 1
        else:
            if relative_distance < self.speed * 0.5 / np.sin(self.max_angle):
                # back up if target is in "unreachable zones"
                velocity *= -1
                steering_angle = 0.0
            else: # if target is far to the side
                # defaults to driving foward, causing the car to circle until the target is within the front cone
                velocity *= 1

        # Line follower: never reverse, always drive forward
        if self.line_follower and velocity < 0:
            velocity = self.speed * 0.5

        """
        # optional derivative controller
        if velocity > 0:
            if self.previous_steering_angle:
                self.get_logger().info(f"adjusting {steering_angle} by: {((self.previous_steering_angle - steering_angle) * 0.5)}")
                steering_angle += (self.previous_steering_angle - steering_angle) * 0.5
            self.previous_steering_angle = steering_angle
        else:
            self.previous_steering_angle = None
        """
        
        drive_cmd.header.frame_id = "base_link"
        drive_cmd.header.stamp = self.get_clock().now().to_msg()
        drive_cmd.drive.speed = velocity
        drive_cmd.drive.steering_angle = steering_angle

        if self.rb_held:
            self.drive_pub.publish(drive_cmd)
        self.error_publisher()

    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()

        #################################

        # YOUR CODE HERE
        # Populate error_msg with relative_x, relative_y, sqrt(x^2+y^2)

        #################################
        error_msg.x_error = self.relative_x
        error_msg.y_error = self.relative_y
        error_msg.distance_error = np.sqrt(self.relative_x ** 2 + self.relative_y ** 2)

        self.error_pub.publish(error_msg)


def main(args=None):
    rclpy.init(args=args)
    pc = ParkingController()
    rclpy.spin(pc)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
