#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import time
import smbus
from example_interfaces.msg import String


class mpu_9250(Node):
    def __init__(self):
        # Initialize Node:
        super().__init__("imu_node")
        self.get_logger().info("imu_node initialised")
        self.timer_delay = 0.01
        self.publisher_topic = "imu_data"
        self.start_publisher()
        # Initialize the bus
        self.bus = smbus.SMBus(1)
        # MPU6050 Registers
        self.MPU6050_ADDR = 0x68
        self.PWR_MGMT_1   = 0x6B
        self.SMPLRT_DIV   = 0x19
        self.CONFIG       = 0x1A
        self.GYRO_CONFIG  = 0x1B
        self.ACCEL_CONFIG = 0x1C
        self.INT_ENABLE   = 0x38
        self.ACCEL_XOUT_H = 0x3B
        self.ACCEL_YOUT_H = 0x3D
        self.ACCEL_ZOUT_H = 0x3F
        self.TEMP_OUT_H   = 0x41
        self.GYRO_XOUT_H  = 0x43
        self.GYRO_YOUT_H  = 0x45
        self.GYRO_ZOUT_H  = 0x47
        # Initialise the sensor
        self.gyro_sens, self.accel_sens = self.mpu6050_start() # instantiate gyro/accel

    def start_publisher(self):
        # Set the timer to publish the data to the topic
        self.publisher_ = self.create_publisher(String, self.publisher_topic, 10)
        self.timer_ = self.create_timer(self.timer_delay, self.imu_publisher)
        self.get_logger().info("imu_data publisher started")

    def imu_publisher(self):
        # Get the data to publish
        a_x,a_y,a_z,w_x,w_y,w_z = self.mpu6050_conv()
        # Set the data to send the z-axis angular velocity
        msg = String()
        msg.data = str(w_z)
        # Publish the data to the topic
        self.publisher_.publish(msg)

    def mpu6050_start(self):
        # alter sample rate (stability)
        samp_rate_div = 0 # sample rate = 8 kHz/(1+samp_rate_div)
        self.bus.write_byte_data(self.MPU6050_ADDR, self.SMPLRT_DIV, samp_rate_div)
        time.sleep(0.1)
        # reset all sensors
        self.bus.write_byte_data(self.MPU6050_ADDR, self.PWR_MGMT_1, 0x00)
        time.sleep(0.1)
        # power management and crystal settings
        self.bus.write_byte_data(self.MPU6050_ADDR, self.PWR_MGMT_1, 0x01)
        time.sleep(0.1)
        #Write to Configuration register
        self.bus.write_byte_data(self.MPU6050_ADDR, self.CONFIG, 0)
        time.sleep(0.1)
        #Write to Gyro configuration register
        gyro_config_sel = [0b00000,0b010000,0b10000,0b11000] # byte registers
        gyro_config_vals = [250.0,500.0,1000.0,2000.0] # degrees/sec
        gyro_indx = 0
        self.bus.write_byte_data(self.MPU6050_ADDR, self.GYRO_CONFIG, int(gyro_config_sel[gyro_indx]))
        time.sleep(0.1)
        #Write to Accel configuration register
        accel_config_sel = [0b00000,0b01000,0b10000,0b11000] # byte registers
        accel_config_vals = [2.0,4.0,8.0,16.0] # g (g = 9.81 m/s^2)
        accel_indx = 1                            
        self.bus.write_byte_data(self.MPU6050_ADDR, self.ACCEL_CONFIG, int(accel_config_sel[accel_indx]))
        time.sleep(0.1)
        # interrupt register (related to overflow of data [FIFO])
        self.bus.write_byte_data(self.MPU6050_ADDR, self.INT_ENABLE, 1)
        time.sleep(0.1)
        self.get_logger().info("IMU sensor started")
        return gyro_config_vals[gyro_indx],accel_config_vals[accel_indx]

    def read_raw_bits(self, register):
        # read accel and gyro values
        high = self.bus.read_byte_data(self.MPU6050_ADDR, register)
        low = self.bus.read_byte_data(self.MPU6050_ADDR, register+1)

        # combine higha and low for unsigned bit value
        value = ((high << 8) | low)

        # convert to +- value
        if(value > 32768):
            value -= 65536
        return value

    def mpu6050_conv(self):
        # raw acceleration bits
        acc_x = self.read_raw_bits(self.ACCEL_XOUT_H)
        acc_y = self.read_raw_bits(self.ACCEL_YOUT_H)
        acc_z = self.read_raw_bits(self.ACCEL_ZOUT_H)

        # raw temp bits
        ##    t_val = read_raw_bits(self.TEMP_OUT_H) # uncomment to read temp
    
        # raw gyroscope bits
        gyro_x = self.read_raw_bits(self.GYRO_XOUT_H)
        gyro_y = self.read_raw_bits(self.GYRO_YOUT_H)
        gyro_z = self.read_raw_bits(self.GYRO_ZOUT_H)

        #convert to acceleration in g and gyro dps
        a_x = (acc_x/(2.0**15.0))*self.accel_sens
        a_y = (acc_y/(2.0**15.0))*self.accel_sens
        a_z = (acc_z/(2.0**15.0))*self.accel_sens
        w_x = (gyro_x/(2.0**15.0))*self.gyro_sens
        w_y = (gyro_y/(2.0**15.0))*self.gyro_sens
        w_z = (gyro_z/(2.0**15.0))*self.gyro_sens

        ##    temp = ((t_val)/333.87)+21.0 # uncomment and add below in return
        return a_x,a_y,a_z,w_x,w_y,w_z

def main(args=None):
    rclpy.init(args=args)
    node = mpu_9250()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()