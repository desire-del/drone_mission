#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image # Image is the message type
import cvzone
import math
import cv2 # OpenCV library
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from nav_msgs.msg import Odometry
from yolov8_msgs.msg import DetectionArray

class Camera(Node):

    def __init__(self):
        super().__init__('camera')
        # self.odom_sub = self.create_subscription(
        #     Odometry,
        #     '/odometry',  # Remplacez par le nom du topic de votre odométrie
        #     self.odom_callback,
        #     10
        # )
        self.camera_sub = self.create_subscription(
            Image,
            '/camera/image_to_draw',
            self.show_image,
            10
        )
        
        self.br = CvBridge()

    def show_image(self, data):
        
        
        frame = self.br.imgmsg_to_cv2(data, "bgr8")

        # display the result
        cv2.imshow("Camera", frame)
        cv2.waitKey(1)
    def odom_callback(self, msg):
        # Extraction des informations de la pose
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        
        # Extraction des informations de la vitesse
        linear_velocity = msg.twist.twist.linear
        angular_velocity = msg.twist.twist.angular
        
        # Affichage des informations
        self.get_logger().info(f'Position: x={position.x:.2f}, y={position.y:.2f}, z={position.z:.2f}')
        self.get_logger().info(f'Orientation: x={orientation.x:.2f}, y={orientation.y:.2f}, z={orientation.z:.2f}, w={orientation.w:.2f}')
        self.get_logger().info(f'Linear Velocity: x={linear_velocity.x:.2f}, y={linear_velocity.y:.2f}, z={linear_velocity.z:.2f}')
        self.get_logger().info(f'Angular Velocity: x={angular_velocity.x:.2f}, y={angular_velocity.y:.2f}, z={angular_velocity.z:.2f}')

def main(args=None):
    rclpy.init(args=args)

    camera = Camera()

    rclpy.spin(camera)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()