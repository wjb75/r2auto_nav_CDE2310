import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import numpy as np
import RPi.GPIO as GPIO
import time

# Servo motor setup
SERVO_PIN = 18  # Adjust as per your GPIO pin
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)
servo = GPIO.PWM(SERVO_PIN, 50)  # 50 Hz PWM frequency
servo.start(0)  # Initialize servo with 0 duty cycle


def set_servo_angle(angle):
    duty = (angle / 18) + 2  # Convert angle to duty cycle
    GPIO.output(SERVO_PIN, True)
    servo.ChangeDutyCycle(duty)
    time.sleep(0.5)
    GPIO.output(SERVO_PIN, False)
    servo.ChangeDutyCycle(0)


class Scanner(Node):
    def __init__(self):
        super().__init__('scanner')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        laser_range = np.array(msg.ranges)
        laser_range[laser_range == 0] = np.nan  # Replace 0s with NaN
        min_index = np.nanargmin(laser_range)
        min_distance = np.nanmin(laser_range)

        self.get_logger().info(f'Shortest distance at {min_index} degrees')
        self.get_logger().info(f'Shortest distance is {min_distance:.2f}m')

        # Check if obstacle is within 2m range and ±30 degrees (assuming 0 degrees is front)
        if min_distance < 2 and (min_index >= 330 or min_index <= 30):
            self.get_logger().info('Obstacle detected! Sweeping servo...')
            self.sweep_servo()

    def sweep_servo(self):
        set_servo_angle(0)
        time.sleep(0.5)
        set_servo_angle(90)
        time.sleep(0.5)
        set_servo_angle(180)
        time.sleep(0.5)
        set_servo_angle(90)
        time.sleep(0.5)
        set_servo_angle(0)


def main(args=None):
    rclpy.init(args=args)
    scanner = Scanner()
    rclpy.spin(scanner)
    scanner.destroy_node()
    GPIO.cleanup()  # Cleanup GPIO when the node is destroyed
    rclpy.shutdown()


if __name__ == '__main__':
    main()
