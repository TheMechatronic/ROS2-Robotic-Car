#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import signal
import threading
import RPi.GPIO as GPIO

class WheelsNode(Node):
    def __init__(self):
        super().__init__("simple_wheels_node")
        self.get_logger().info("simple_wheels_node initialised")
        signal.signal(signal.SIGINT, self.sigint_handeler)

        # Allocate the GPIO pins
        pwm_input = 100
        A1_in = 16  # 36/16
        A2_in = 26  # 37/26
        B1_in = 5   #29/5
        B2_in = 6   #31/6

        self.initial_PWM = 50
        self.running = False
        self.start_delay = 15
        self.stop_delay = 10

        # Set up the GPIO pins
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(A1_in, GPIO.OUT)
        GPIO.setup(A2_in, GPIO.OUT)
        GPIO.setup(B1_in, GPIO.OUT)
        GPIO.setup(B2_in, GPIO.OUT)

        self.pwm_R_fwd = GPIO.PWM(A2_in, 1000)
        self.pwm_R_bwd = GPIO.PWM(A1_in, 1000)
        self.pwm_L_fwd = GPIO.PWM(B2_in, 1000)
        self.pwm_L_bwd = GPIO.PWM(B1_in, 1000)

        # Starting PWM value
        self.pwm_R_fwd.start(0)
        self.pwm_R_bwd.start(0)
        self.pwm_L_fwd.start(0)
        self.pwm_L_bwd.start(0)

        # Start the timer to start the wheels
        self.start_timer = threading.Timer(self.start_delay, self.start_wheels)
        self.start_timer.start()
        # Set the timer to stop the wheels
        self.stop_timer = threading.Timer(self.stop_delay, self.stop_wheels)


    def sigint_handeler(self, signum, frame):
        self.cleanup()

    def cleanup(self):
        self.get_logger().info("cleaning up before shutting down")
        GPIO.cleanup()
        self.destroy_node()
        rclpy.shutdown()

    def start_wheels(self):
        self.left_pwm = self.initial_PWM
        self.right_pwm = self.initial_PWM

        # Ensure that the PWM value is not out of bounds
        if self.left_pwm > 100:
            self.left_pwm = 100
        elif self.left_pwm < 0:
            self.left_pwm = 0
        if self.right_pwm > 100:
            self.right_pwm = 100
        elif self.right_pwm < 0:
            self.right_pwm = 0   # Set the PWM value to the motors
        
        # Set the PWM values of the forward wheel pins
        self.pwm_L_fwd.start(self.left_pwm)
        self.pwm_R_fwd.start(self.right_pwm)
        self.get_logger().info("wheels started")
        # Start the timer that will stop the movement
        self.stop_timer.start()

    def stop_wheels(self):
        # Set the PWM values of the forward wheel pins
        self.pwm_L_fwd.stop()
        self.pwm_R_fwd.stop()
        self.get_logger().info("wheels stopped")
        self.cleanup()

def main(args=None):
    rclpy.init(args=args)
    node = WheelsNode()
    rclpy.spin(node)
    

if __name__ == "__main__":
    main()
