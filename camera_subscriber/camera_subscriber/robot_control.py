import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu
import math


def RadToDeg(rad):
    return rad * 180.0 / math.pi


def QuatToEuler(quat):
    qx, qy, qz, qw = quat
    pitch = 0
    sinR_cosP = 2.0 * (qw * qx + qy * qz)
    cosR_cosP = 1.0 - 2.0 * (qx ** 2.0 + qy ** 2.0)
    roll = math.atan2(sinR_cosP, cosR_cosP)
    try:
        sinP = math.sqrt(1.0 + 2.0 * (qw * qy + qy * qz))
        cosP = math.sqrt(1.0 - 2.0 * (qw * qy - qx * qz))
        pitch = 2.0 * math.atan2(sinP, cosP) - math.pi / 2.0
    except ValueError as e:
        print(e)
    sinY_copP = 2.0 * (qw * qz + qx * qy)
    cosY_cosP = 1.0 - 2.0 * (qy ** 2.0 + qz ** 2.0)
    yaw = math.atan2(sinY_copP, cosY_cosP)
    return list(map(RadToDeg, [roll, pitch, yaw]))


def normalizeQuat(quat):
    qx, qy, qz, qw = quat
    norm = float(math.sqrt(qw ** 2.0 + qx ** 2.0 + qy ** 2.0 + qz ** 2.0))
    return list(map(lambda x: float(x / norm), quat))


def invQuat(quat):
    qx, qy, qz, qw = quat
    return [-qx, -qy, -qz, qw]


def EulerToQuat(euler):
    roll, pitch, yaw = euler
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return [qx, qy, qz, qw]


def MultiplicationQuat(quat1, quat2):
    b1, c1, d1, a1 = quat1
    b2, c2, d2, a2 = quat2
    qw = a1 * a2 - b1 * b2 - c1 * c2 - d1 * d2
    qx = a1 * b2 + b1 * a2 + c1 * d2 - d1 * c2
    qy = a1 * c2 - b1 * d2 + c1 * a2 + d1 * b2
    qz = a1 * d2 + b1 * c2 - c1 * b2 + d1 * a2
    return normalizeQuat([qx, qy, qz, qw])


def getChangedOrientation(quat_now, quat_last):
    quat = MultiplicationQuat(quat_now, invQuat(quat_last))
    return normalizeQuat(quat)


class RobotControlPublisher(Node):
    cmd_vel = Twist()
    cmd_vel.linear.x = 0.0
    cmd_vel.linear.y = 0.0
    cmd_vel.linear.z = 0.0
    cmd_vel.angular.x = 0.0
    cmd_vel.angular.y = 0.0
    cmd_vel.angular.z = 0.0
    initial_guess = False
    initial_pose = Quaternion()

    def __init__(self):
        super().__init__('RobotControl')
        self.subscription = self.create_subscription(
            Point, 'point', self.listener_callback, 10)
        self.subscription_imu = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, 10)
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.publisher.publish(self.cmd_vel)

    def listener_callback(self, point):
        if point.y > 512.0 / 2.0:
            self.cmd_vel.linear.x = 0.5
        else:
            self.cmd_vel.linear.x = -0.5

    def imu_callback(self, imu):
        if self.initial_guess:
            orientation = [imu.orientation.x, imu.orientation.y,
                           imu.orientation.z, imu.orientation.w]
            q = getChangedOrientation(
                [self.initial_pose.x, self.initial_pose.y, self.initial_pose.x, self.initial_pose.w], orientation)
            try:
                euler = QuatToEuler(q)
            except ValueError as e:
                self.get_logger().info('Error: '.format(e))
            if QuatToEuler(q)[2] > 45.0:
                self.cmd_vel.angular.z = 1.8
            elif QuatToEuler(q)[2] < -45.0:
                self.cmd_vel.angular.z = -1.8
            else:
                self.cmd_vel.angular.z = 0.0

            self.get_logger().info(
                'Subscribe: Initial IMU {}'.format(QuatToEuler(q)[2]))
        else:
            self.initial_pose.x = imu.orientation.x
            self.initial_pose.y = imu.orientation.y
            self.initial_pose.z = imu.orientation.z
            self.initial_pose.w = imu.orientation.w
            self.initial_guess = True


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = RobotControlPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
