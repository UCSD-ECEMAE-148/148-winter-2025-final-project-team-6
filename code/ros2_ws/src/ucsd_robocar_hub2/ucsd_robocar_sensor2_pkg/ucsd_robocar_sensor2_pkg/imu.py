import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import numpy as np
import matplotlib.pyplot as plt
import serial
import threading

class IMUProcessingNode(Node):
    def __init__(self):
        super().__init__('imu_processing_node')

        # Publishers for yaw rate and acceleration magnitude
        self.yaw_pub = self.create_publisher(Float32, '/drift/yaw_rate', 10)
        self.accel_pub = self.create_publisher(Float32, '/drift/accel_magnitude', 10)

        # Subscribe to IMU data
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Serial port reading thread
        self.serial_thread = threading.Thread(target=self.read_serial_data)
        self.serial_thread.daemon = True
        self.serial_thread.start()

        # Data buffers for plotting
        self.yaw_data = []
        self.accel_data = []
        self.time_data = []
        self.t0 = self.get_clock().now().nanoseconds / 1e9

        # Start plot thread
        self.plot_thread = threading.Thread(target=self.plot_data)
        self.plot_thread.daemon = True
        self.plot_thread.start()

    def imu_callback(self, msg):
        # Extract yaw rate from angular velocity.z
        yaw_rate = msg.angular_velocity.z

        # Compute acceleration magnitude
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        accel_magnitude = (ax**2 + ay**2 + az**2)**0.5

        # Time stamp
        now = self.get_clock().now().nanoseconds / 1e9 - self.t0
        self.time_data.append(now)
        self.yaw_data.append(yaw_rate)
        self.accel_data.append(accel_magnitude)

        # Publish data
        self.yaw_pub.publish(Float32(data=yaw_rate))
        self.accel_pub.publish(Float32(data=accel_magnitude))

        self.get_logger().info(f"Yaw Rate: {yaw_rate:.2f} rad/s, Accel Mag: {accel_magnitude:.2f} m/s^2")

    def read_serial_data(self):
        try:
            ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            self.get_logger().info("Serial port /dev/ttyACM0 opened.")
            while True:
                line = ser.readline().decode('utf-8').strip()
                if line:
                    self.get_logger().info(f"[SERIAL] {line}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial connection failed: {e}")

    def plot_data(self):
        plt.ion()
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6))
        while True:
            if len(self.time_data) > 1:
                ax1.clear()
                ax2.clear()

                ax1.plot(self.time_data, self.yaw_data, label='Yaw Rate (rad/s)')
                ax1.set_ylabel('Yaw Rate')
                ax1.grid(True)
                ax1.legend()

                ax2.plot(self.time_data, self.accel_data, label='Accel Magnitude (m/s²)', color='orange')
                ax2.set_xlabel('Time (s)')
                ax2.set_ylabel('Accel Magnitude')
                ax2.grid(True)
                ax2.legend()

                plt.pause(0.1)


def main(args=None):
    rclpy.init(args=args)
    node = IMUProcessingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down IMU Processing Node")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
