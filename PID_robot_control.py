import RPi.GPIO as GPIO          
import time
import smbus
import math
import threading
import signal
import sys

class mpu_9250():
    def __init__(self):
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

        self.gyro_sens, self.accel_sens = self.mpu6050_start() # instantiate gyro/accel

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
        print("MPU6050 started")
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

class Controller():
    def __init__(self):
        # Constants influincing the shape of the drive.
        self.sin_amplitude = 10 # Factor - needs tuning (normal is 15 for 's' shape)
        self.sin_period = 1 # Seconds
        self.start_delay = 10 # Seconds
        # PID parameters
        self.Kp = 0.5
        self.Ki = 0.3
        self.Kd = 0.1
        # Initial values
        self.angle = 0.0
        self.ref_angle = 0.0
        self.prev_error = 0
        self.integral = 0.0
        self.t = 0
        self.dt = 0.01
        # remember to init the imu class ...
        self.mpu9250 = mpu_9250()
        self.wheel_controller = wheel_Controller()
        self.timer = threading.Timer(self.dt, self._run)

    def start_control(self):
        print("Controller starting")
        time.sleep(self.start_delay)
        self.start_timer()
        self.wheel_controller.start_wheels(50)

    def start_timer(self):
        self.timer = threading.Timer(self.dt, self._run)
        self.timer.start()

    def stop(self):
        self.timer.cancel()
        self.wheel_controller.cleanup()

    def _run(self):
        self.control()
        self.start_timer()

    def control(self):
        # Get the imu data
        a_x,a_y,a_z,w_x,w_y,w_z = self.mpu9250.mpu6050_conv()
        self.PID_output(w_z)
        # Call the wheel controller ...
        self.wheel_controller.wheel_control(self.output)

    def PID_output(self, angular_velocity):
        self.angle += angular_velocity * self.dt
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

class wheel_Controller():
    def __init__(self):
        # Allocate the GPIO pins
        pwm_input = 100
        A1_in = 16  # 36/16
        A2_in = 26  # 37/26
        B1_in = 5   #29/5
        B2_in = 6   #31/6

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
    
    def cleanup(self):
        GPIO.cleanup()

    def start_wheels(self, initial_PWM):
        self.left_pwm = initial_PWM
        self.right_pwm = initial_PWM

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
        self.pwm_L_fwd.ChangeDutyCycle(self.left_pwm)
        self.pwm_R_fwd.ChangeDutyCycle(self.right_pwm)

    def wheel_control(self, PID_output):
        # Set the motor speed
        self.left_pwm -= PID_output
        self.right_pwm += PID_output

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
        self.pwm_L_fwd.ChangeDutyCycle(self.left_pwm)
        self.pwm_R_fwd.ChangeDutyCycle(self.right_pwm)

def cleanup(signal, frame):
    GPIO.cleanup()
    sys.exit(0)

signal.signal(signal.SIGINT, cleanup)

def main():
    try:
        controller = Controller()
        controller.start_control()
    except KeyboardInterrupt:
        controller.stop()

if __name__ == "__main__":
    main()