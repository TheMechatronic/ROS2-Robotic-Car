#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import time
import smbus
import threading
import math
import signal
from example_interfaces.msg import String

class Controller(Node):
    def __init__(self):
        # Initialize Node:
        super().__init__("controller_node")
        self.get_logger().info("controller_node initialised")
        self.timer_delay = 0.03
        self.publisher_topic = "wheel_data"
        self.subscriber_topic = "imu_data"
        self.start_subscriber()
        self.start_publisher()
        signal.signal(signal.SIGINT, self.sigint_handeler)

        # Constants influincing the shape of the drive.
        self.sin_amplitude = 0 # Factor - needs tuning (normal is 15 for 's' shape)
        self.sin_period = 2 # Seconds
        self.start_delay = 10 # Seconds
        # PID parameters
        self.Kp = 1
        self.Ki = 0.5
        self.Kd = 2
        # Initial values
        self.angle = 0.0
        self.ref_angle = 0.0
        self.prev_error = 0
        self.integral = 0.0
        self.t = 0.0
        self.angular_velocity = 0.0
        self.dt = self.timer_delay
        self.timer = threading.Timer(self.start_delay, self.start_program)
        self.timer.start()

    def start_program(self):
        self.start_wheels()
        self.angle = 0.0
        self.ref_angle = 0.0
        self.prev_error = 0
        self.integral = 0.0
        self.t = 0

    def start_subscriber(self):
        self.v_count = 0.0
        self.v_accum = 0.0
        self.subscriber_ = self.create_subscription(String, self.subscriber_topic, self.imu_data_callback, 10)

    def imu_data_callback(self, msg):
        self.v_count += 1
        self.v_accum += float(msg.data) / (2*math.pi)

    def start_publisher(self):
        # Set the timer to publish the data to the topic
        self.publisher_ = self.create_publisher(String, self.publisher_topic, 10)
        self.timer_ = self.create_timer(self.timer_delay, self.wheel_data_publisher)
        self.get_logger().info("wheel_data publisher started")

    def sigint_handeler(self, signum, frame):
        self.cleanup()

    def wheel_data_publisher(self):
        # Call the control function
        self.control()
        # Set the data to send the PID control output
        msg = String()
        msg.data = str(self.output)
        # Publish the data to the topic
        self.publisher_.publish(msg)

    def cleanup(self):
        self.stop_wheels()
        self.get_logger().info("cleaning up before shutting down (wheels should stop now)")
        self.destroy_node()
        rclpy.shutdown()

    def start_wheels(self):
        msg = String()
        msg.data = "start"
        self.publisher_.publish(msg)
        self.get_logger().info("start message sent to wheels")

    def stop_wheels(self):
        msg = String()
        msg.data = "stop"
        self.publisher_.publish(msg)
        self.get_logger().info("stop message sent to wheels")

    def control(self):
        if self.v_count == 0:
            self.angular_velocity = self.v_accum
            self.get_logger().info("zero v_count ...")
            self.get_logger().info(str(self.v_count))
            self.get_logger().info(str(self.v_accum))
        else:
            self.angular_velocity = self.v_accum / self.v_count
        self.v_accum = 0
        self.v_count = 0
        # Trigger the PID controller ...
        self.PID_output()

    def PID_output(self):
        self.angle += self.angular_velocity * self.dt
        # Calculate the error in the angle
        self.error = self.ref_angle - self.angle
        # Derivative of the error
        self.derivative = (self.error - self.prev_error) / self.dt
        # Acccumulated error of the angle
        self.integral += self.error * self.dt
        # Calculate the output of the PID
        self.output = self.Kp * self.error + self.Ki * self.integral + self.Kd * self.derivative
        # Reset the previous error value for next itteration
        self.prev_error = self.error
        # Discretisation of the PID control
        self.t += self.dt
        # Referance Angle determined from predefined constants
        self.ref_angle = self.sin_amplitude * math.sin(self.t * math.pi * 2 / self.sin_period)

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)

if __name__ == "__main__":
    main()