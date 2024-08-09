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

class Detector(Node):

    def __init__(self):
        super().__init__('detection')
        self.camera_sub = self.create_subscription(
            Image,
            '/camera/image',
            self.camera_callback,
            qos_profile_sensor_data)
        # self.camera_sub  # prevent unused variable warning
        # # self.odom_sub = self.create_subscription(
        # #     Odometry,
        # #     '/odometry',  # Remplacez par le nom du topic de votre odométrie
        # #     self.odom_callback,
        # #     10
        # # )
        self.yolo_dection_sub = self.create_subscription(
            DetectionArray,
            '/yolo/detections',
            self.yolo_detection_callback,
            10
        )
        self.image_with_boxes_pub = self.create_publisher(
            Image,
            '/camera/image_to_draw',
            10)
        self.latest_image = None
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
    def camera_callback(self, data):
        self.latest_image = data
        

    def yolo_detection_callback(self, data):
        if not self.latest_image:
            return
        if len(data.detections) > 0:
            self.get_logger().info('------Boats-detected-------')
            frame = self.br.imgmsg_to_cv2(self.latest_image, "bgr8")
            for r in data.detections:
                box = r.bbox
                center = box.center.position
                size = box.size
                x1, y1 = center.x - size.x/2, center.y - size.y/2
                x2, y2 = center.x + size.x/2, center.y + size.y/2
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                w, h = int(size.x), int(size.y)
                self.get_logger().info(f'x1={x1}, y1={y1}, w={w}, h={h}')
                
                frame = cvzone.cornerRect(frame, (x1, y1, w, h))

                conf = math.ceil((r.score *100))/100

                name = "boat"

                frame, _ = cvzone.putTextRect(frame, f'{name} : {conf}', (max(0,x1), max(35,y1)), scale = 0.1, thickness = 1)
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                self.image_with_boxes_pub.publish(self.br.cv2_to_imgmsg(frame, encoding="bgr8"))
        else:
            self.get_logger().info('------No-boat-detected-------')
            self.image_with_boxes_pub.publish(self.latest_image)


def main(args=None):
    rclpy.init(args=args)

    ship_detector = Detector()

    rclpy.spin(ship_detector)
    ship_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()