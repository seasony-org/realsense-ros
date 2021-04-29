#!/usr/bin/env python3
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

cap = cv2.VideoCapture("/dev/video4")

class MinimalImgPublisher(Node):

    def __init__(self):
        super().__init__('minimal_img_publisher')
        qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10,
                                 reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                 durability=QoSDurabilityPolicy.VOLATILE)
        self.rgb_pub = self.create_publisher(
            CompressedImage,
            '/camera/color/image_raw', qos_profile)
        self.count = 0
    def run(self):
        br = CvBridge()
        ret, frame = cap.read()
        
        #msg = br.cv2_to_imgmsg(frame, encoding="bgr8")
        #dtype, n_channels = br.encoding_as_cvtype2('8UC3')
        #im = np.ndarray(shape=(480, 640, n_channels), dtype=dtype)
        cmprsmsg = br.cv2_to_compressed_imgmsg(frame)
        cmprsmsg.header.frame_id = 'camera_left_color_optical_frame'

        self.rgb_pub.publish(cmprsmsg)
        if self.count == 100:
            cv2.imwrite('/home/watney/seasony/realsense-ros/src/realsense2_camera/scripts/test.png', frame)
            print("DONE!")
        self.count = self.count + 1
        if cv2.waitKey(1) & 0xFF == ord('q'):
            return

def main(args=None):
    rclpy.init(args=args)

    minimal_img_publisher = MinimalImgPublisher()
    frequency = 10  # Hz
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