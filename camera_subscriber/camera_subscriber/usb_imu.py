import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
import math


class ImuPublisher(Node):

    imu_ = Imu()
    imu_.orientation_covariance = [
        -1.0, -1.0, -1.0,
        -1.0, -1.0, -1.0,
        -1.0, -1.0, -1.0
    ]
    imu_.angular_velocity_covariance = [
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0
    ]
    imu_.linear_acceleration_covariance = [
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0
    ]
    mag_ = MagneticField()
    mag_.magnetic_field_covariance = [
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0
    ]
    header_ = Header()
    header_.frame_id = 'map'

    calibrated = False

    def __init__(self, portname):
        super().__init__('usb_imu')
        try:
            self.port = serial.Serial(port=portname, baudrate=9600)
            self.get_logger().info('Device is connected')
        except SerialException:
            self.get_logger().info('Port is already open')

        self.pubImu = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.pubMag = self.create_publisher(MagneticField, 'imu/mag', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        if self.calibrated:
            self.header_.stamp = self.get_clock().now().to_msg()
            acc = [0, 0, 0]
            gyr = [0, 0, 0]
            mag = [0, 0, 0]

            line = self.port.readline()
            line = line.decode()
            self.get_logger().info('Publishing {}'.format(line))
            data = line.split('.')
            acc = data[0].split(',')
            gyr = data[1].split(',')
            mag = data[2].split(',')
            deg_to_rad = math.pi / 180.0
            self.imu_.header = self.header_
            self.imu_.angular_velocity.x = float(gyr[1]) * deg_to_rad
            self.imu_.angular_velocity.y = float(gyr[2]) * deg_to_rad
            self.imu_.angular_velocity.z = float(gyr[3]) * deg_to_rad
            mg_to_mps = 9.81 * pow(10, -3)
            self.imu_.linear_acceleration.x = float(acc[1]) * mg_to_mps
            self.imu_.linear_acceleration.y = float(acc[2]) * mg_to_mps
            self.imu_.linear_acceleration.z = float(acc[3]) * mg_to_mps

            self.mag_.header = self.header_
            self.mag_.magnetic_field.x = float(mag[1])
            self.mag_.magnetic_field.y = float(mag[2])
            self.mag_.magnetic_field.z = float(mag[3])

            self.pubImu.publish(self.imu_)
            self.pubMag.publish(self.mag_)
            self.get_logger().info('Publishing IMU data')
            self.i += 1

        if self.calibrated == False:
            line = self.port.readline()
            line = line.decode()
            self.get_logger().info(line)
            if line[0] == ".":
                self.calibrated = True
                self.get_logger().info('Device is calibrated')


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = ImuPublisher("/dev/ttyUSB0")
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
