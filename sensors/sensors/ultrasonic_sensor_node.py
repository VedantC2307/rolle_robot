#!/usr/bin/env python3
# Improve logic

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import time


class UltrasonicSensorNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_sensor_node')

        # ROS 2 Publisher
        self.publisher_ = self.create_publisher(Float32, 'ultrasonic_distance', 2)

        # Timer for periodic distance measurement
        self.timer = self.create_timer(0.1, self.measure_and_publish_distance)  # 10 Hz

        # GPIO Setup
        self.GPIO_TRIGGER = 18
        self.GPIO_ECHO = 23
        self.ultrasonic_setup()

        self.prev_distance = None

        self.get_logger().info("Ultrasonic Sensor Node initialized!")

    def ultrasonic_setup(self):
        """
        Setup GPIO pins for the ultrasonic sensor.
        """
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.GPIO_TRIGGER, GPIO.OUT)
        GPIO.setup(self.GPIO_ECHO, GPIO.IN)
        GPIO.setwarnings(False)

    def measure_distance(self):
        """
        Measure the distance using the ultrasonic sensor.
        Returns the distance in meters or None in case of an error.
        """
        try:
            # Trigger a pulse
            GPIO.output(self.GPIO_TRIGGER, True)
            time.sleep(0.00001)  # 10 µs pulse
            GPIO.output(self.GPIO_TRIGGER, False)

            # Record the start and stop times
            start_time = time.time()
            stop_time = time.time()

            # Wait for the echo to start
            timeout_start = time.time()
            while GPIO.input(self.GPIO_ECHO) == 0:
                start_time = time.time()
                if time.time() - timeout_start > 0.02:  # 20ms timeout
                    raise TimeoutError("Timeout waiting for echo to start.")

            # Wait for the echo to end
            timeout_start = time.time()
            while GPIO.input(self.GPIO_ECHO) == 1:
                stop_time = time.time()
                if time.time() - timeout_start > 0.05:  # 50ms timeout
                    raise TimeoutError("Timeout waiting for echo to stop.")

            # Calculate the distance
            time_elapsed = stop_time - start_time
            distance = (time_elapsed * 34300) / 2  # Distance in cm
            return distance * 0.01  # Convert to meters

        except Exception as e:
            self.get_logger().error(f"Error measuring distance: {e}")
            return None

    def measure_and_publish_distance(self):
        """
        Callback function to measure the distance and publish it to the ROS 2 topic.
        Includes logic to smooth out erratic readings.
        """
        try:
            # Measure distance
            distance = self.measure_distance()

            if distance is not None and 0.02 <= distance <= 4.0:  # Valid range: 2cm to 4m
                # Smoothing logic: if the difference is too large, use the previous value
                if self.prev_distance is not None and abs(distance - self.prev_distance) > 1.0:
                    distance = self.prev_distance
                    # self.get_logger().warn(f"Erratic reading detected. Using previous value: {distance:.2f} meters")

                # Publish the distance
                msg = Float32()
                msg.data = 2.5 #distance
                self.publisher_.publish(msg)
                self.get_logger().debug(f"Distance: {distance:.2f} meters")

                self.prev_distance = distance

            else:
                self.get_logger().debug("Invalid distance value.")

        except Exception as e:
            self.get_logger().debug(f"Error in distance measurement: {e}")

    def cleanup(self):
        """
        Cleanup GPIO resources.
        """
        self.get_logger().info("Cleaning up GPIO resources...")
        try:
            GPIO.cleanup([self.GPIO_TRIGGER, self.GPIO_ECHO])
        except Exception as e:
            self.get_logger().error(f"Error during cleanup: {e}")

def main(args=None):
    rclpy.init(args=args)

    # Create the ultrasonic sensor node
    node = UltrasonicSensorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Ultrasonic Sensor Node.")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()