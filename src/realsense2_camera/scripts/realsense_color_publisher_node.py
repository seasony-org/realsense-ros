#!/usr/bin/env python3
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

cap = cv2.VideoCapture("/dev/video6")

class MinimalImgPublisher(Node):

    def __init__(self):
        super().__init__('minimal_img_publisher')
        qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10,
                                 reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                 durability=QoSDurabilityPolicy.VOLATILE)
        self.rgb_pub = self.create_publisher(
            Image,
            '/camera/color/image_raw', qos_profile)

    def run(self):
        br = CvBridge()
        ret, frame = cap.read()
        msg = br.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.frame_id = 'camera_left_color_optical_frame'
        self.rgb_pub.publish(msg)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            return

def main(args=None):
    rclpy.init(args=args)

    minimal_img_publisher = MinimalImgPublisher()
    frequency = 30  # Hz
    timer_period = 1 / frequency
    timer = minimal_img_publisher.create_timer(timer_period, minimal_img_publisher.run)

    rclpy.spin(minimal_img_publisher)

    minimal_img_publisher.destroy_timer(timer)

    cap.release()
    cv2.destroyAllWindows()

    minimal_img_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
