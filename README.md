# Readme file for PID_car_controll Python code

This Python code contains a class called mpu_9250 which initializes and reads data from the MPU-9250 sensor. The MPU-9250 is a System in Package (SiP) that combines a 3-axis gyroscope, 3-axis accelerometer, 3-axis magnetometer, and a Digital Motion Processor (DMP) in a small package. The MPU-9250 communicates with the Raspberry Pi via the I2C bus.
## Prerequisites

The following Python modules need to be installed to run this code:

    RPi.GPIO
    time
    smbus
    math
    threading
    signal
    sys

The code is written to be used on a Raspberry Pi board with a MPU-9250 IMU module

## MPU_9250 Class Functions
### mpu_9250()

The mpu_9250 function initializes the MPU-9250 sensor by setting up the I2C communication, configuring the sampling rate, resetting all sensors, setting the power management and crystal settings, and setting the gyro and accelerometer configurations. The function also returns the values of the gyro and accelerometer sensitivities.
### mpu6050_start()

The mpu6050_start function is called by the mpu_9250 function and initializes the MPU-6050 sensor. This function also alters the sample rate and sets the power management and crystal settings.
### read_raw_bits(register)

The read_raw_bits function reads the raw accelerometer and gyroscope values from the MPU-9250 registers.
### mpu6050_conv()

The mpu6050_conv function reads the raw accelerometer and gyroscope values using read_raw_bits and converts these values to acceleration in g and gyro degrees per second (dps).
### Usage

To use this code, create an instance of the mpu_9250 class and call the mpu6050_conv function to read the sensor values. For example:

```python
sensor = mpu_9250()
while True:
    a_x, a_y, a_z, w_x, w_y, w_z = sensor.mpu6050_conv()
    print("Acceleration (g): x = {:.2f}, y = {:.2f}, z = {:.2f}".format(a_x, a_y, a_z))
    print("Gyroscope (dps): x = {:.2f}, y = {:.2f}, z = {:.2f}".format(w_x, w_y, w_z))
    time.sleep(0.1)
```

This code will create an instance of the mpu_9250 class, initialize the sensor, and then continuously read and print the acceleration and gyroscope values every 0.1 seconds.

## Controller Class
This code defines a Controller class, which implements a PID control algorithm to control the orientation of a robot. The robot's orientation is measured using an MPU-9250 sensor, and the control output is used to drive two wheels.
### Usage

To use the Controller class, you must first instantiate it:

```python
controller = Controller()
```

The following parameters can be set using the class constructor:

- sin_amplitude: a factor influencing the shape of the drive (default is 10).
- sin_period: the period of the sine wave used to generate the reference angle for the control algorithm (default is 1 second).
- start_delay: the delay in seconds before the control loop starts (default is 10 seconds).
- Kp: the proportional gain of the PID algorithm (default is 0.5).
- Ki: the integral gain of the PID algorithm (default is 0.3).
- Kd: the derivative gain of the PID algorithm (default is 0.1).
- angle: the initial orientation of the robot (default is 0).
- ref_angle: the initial reference angle for the control algorithm (default is 0).
- prev_error: the previous error value used for calculating the derivative term of the PID algorithm (default is 0).
- integral: the accumulated error value used for calculating the integral term of the PID algorithm (default is 0).
- t: the current time value (default is 0).
- dt: the time step used for discretizing the control algorithm (default is 0.01 seconds).
- mpu9250: an instance of the mpu_9250 class used for measuring the orientation of the robot.
- wheel_controller: an instance of the wheel_Controller class used for controlling the two wheels of the robot.
- timer: a threading.Timer object used to run the control loop at regular intervals.

Once the Controller object has been instantiated, you can start the control loop by calling the start_control() method:

```python
controller.start_control()
```

This will start the control loop after the delay specified by start_delay. The control loop runs indefinitely until it is stopped.

To stop the control loop, you can call the stop() method:

```python
controller.stop()
```
This will cancel the Timer object and clean up any resources used by the wheel_controller.
### Control Algorithm

The Controller class uses a PID control algorithm to control the orientation of the robot. The algorithm takes the following steps:

1) Measure the current orientation of the robot using the 'MPU-9250' sensor.
2) Calculate the error between the current orientation and the reference orientation, which is a sine wave generated using the 'sin_amplitude' and 'sin_period' constants.
3) Calculate the derivative of the 'error' by taking the difference between the current error and the previous error, and dividing by the time step 'dt'.
4) Calculate the integral of the error by accumulating the error over time, multiplied by dt.
5) Calculate the control output by multiplying the error by the proportional gain 'Kp', adding the integral of the error multiplied by the integral gain 'Ki', and adding the derivative of the error multiplied by the derivative gain 'Kd'.
6) Discretize the control output by multiplying by the time step 'dt'.
7) Send the control output to the 'wheel_controller' to control the two wheels of the robot.

The control loop runs at a regular interval specified by the 'dt' constant.

## Wheel_controller class
The wheel_Controller class provides control of the wheels of a robot. It sets up the GPIO pins for the motor drivers, initializes the PWM values, and allows for control of the wheels using a PID controller.
### Initialization

The class is initialized with no parameters. During initialization, the GPIO pins are allocated, and the PWM values for the motors are set to 0.
### start_wheels

The start_wheels method sets the initial PWM values for the motors. The initial_PWM parameter is used to set the initial value of the left and right motors. The PWM value is limited to between 0 and 100 to ensure that the motor speed is not out of bounds. The PWM values are then set to the motor drivers.
### wheel_control

The wheel_control method is used to adjust the PWM values of the left and right motors based on the output of a PID controller. The PID_output parameter is the output value of the PID controller, and it is used to adjust the PWM values of the left and right motors. The PWM value is limited to between 0 and 100 to ensure that the motor speed is not out of bounds. The PWM values are then set to the motor drivers.
### cleanup

The cleanup method is used to clean up the GPIO pins used by the motor drivers. This method should be called when the program is exiting to ensure that the GPIO pins are released.

## 'main' loop code and interrupt handling
This code sets up a signal handler to clean up GPIO pins and exit the program when the user sends a SIGINT signal (e.g. by pressing Ctrl+C). The main() function creates a Controller object and starts its control loop. If a KeyboardInterrupt (Ctrl+C) is detected, the controller.stop() method is called to gracefully shut down the control loop. The if __name__ == "__main__": statement ensures that the main() function is only called when the script is run directly, rather than when it is imported as a module.
