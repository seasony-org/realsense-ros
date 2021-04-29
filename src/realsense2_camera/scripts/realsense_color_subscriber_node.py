#!/usr/bin/env python3
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

class MinimalImgSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_img_publisher')
        qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10,
                                 reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                 durability=QoSDurabilityPolicy.VOLATILE)
        self.rgb_pub = self.create_subscription(
            CompressedImage,
            '/camera/color/image_raw', self.rgb_callback, qos_profile)

        self.count = 0
        self.brige = CvBridge()
    def rgb_callback(self, data):
        try:
            cv_image = self.brige.compressed_imgmsg_to_cv2(data, "passthrough")
        except CvBridgeError as e:
            print(e)

        cv2.imshow('test', cv_image)   
        if cv2.waitKey(1) & 0xFF == ord('q'):
            return

def main(args=None):
    rclpy.init(args=args)

    minimal_img_subscriber = MinimalImgSubscriber()
    
    rclpy.spin(minimal_img_subscriber)

    minimal_img_subscriber.destroy_timer(timer)

    cv2.destroyAllWindows()

    minimal_img_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()