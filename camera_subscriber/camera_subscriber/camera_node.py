#!/usr/bin/env python3
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # ROS2 package to convert between ROS and OpenCV Images
import cv2 # Python OpenCV library
import numpy as np
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('Camera_control')
        self.window_name = "camera"
        self.subscription = self.create_subscription(Image,'image_raw',self.listener_callback,10)
        self.subscription  # prevent unused variable warning
        self.point = None
        self.publisher = self.create_publisher(Point, 'point', 10)
        self.publisher_cmd_vel = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        cmd_vel_= Twist()
        if self.point != None:
            if self.point[1] > 512.0 / 2.0:
                cmd_vel_.linear.x  = 0.1
            else:
                cmd_vel_.linear.x  = -0.1

        cmd_vel_.linear.y  = 0.0
        cmd_vel_.linear.z  = 0.0
        cmd_vel_.angular.x  = 0.0
        cmd_vel_.angular.y  = 0.0
        cmd_vel_.angular.z  = 0.0
        self.publisher_cmd_vel.publish(cmd_vel_)
        self.get_logger().info('Publishing: cmd_vel')

    def listener_callback(self, image_data):
        cv_image = CvBridge().imgmsg_to_cv2(image_data,"bgr8")
        if(self.point is not None):
            cv2.rectangle(cv_image,self.point,(self.point[0]+50,self.point[1]+50),(0,255,0),3)
            self.get_logger().info('Publishing {}'.format(self.point))
        cv2.imshow(self.window_name, cv_image)
        cv2.waitKey(25)
        cv2.setMouseCallback(self.window_name, self.draw_rectangle)
        corners = self.detectArUcoMarker(cv_image)
        if corners != None:
            self.point = (int(corners[0][0]), int(corners[0][1]))
            

    def detectArUcoMarker(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        param = cv2.aruco.DetectorParameters_create()
        corner, ids, rejected = cv2.aruco.detectMarkers(gray, dictionary, parameters=param)
        if len(corner) == 1:
            return list(corner[0][0])
        else:
            return None

    def draw_rectangle(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN: # check if mouse event is click
            self.point = (x,y)
            point_ = Point()
            point_.x =  float(x)
            point_.y =  float(y)
            self.publisher.publish(point_)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()