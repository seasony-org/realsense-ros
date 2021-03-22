import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class MinimalImgSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_img_subscriber')
        qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10,
                                 reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                 durability=QoSDurabilityPolicy.VOLATILE)
        self.rgb_subs = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.rgb_callback, qos_profile)

        self.depth_subs = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.depth_callback, qos_profile)

    def rgb_callback(self, msg):
        br = CvBridge()
        cv_image = br.imgmsg_to_cv2(msg, desired_encoding='bgr8')  # the encoding type can be check in the message
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

    def depth_callback(self, msg):
        br = CvBridge()
        cv_image = br.imgmsg_to_cv2(msg, desired_encoding='16UC1')  # the encoding type can be check in the message
        print("Depth for given pixel (in mm): ", cv_image[265][283])

        cv2.imshow("Depth Image window", cv_image)
        cv2.waitKey(3)


def main(args=None):
    rclpy.init(args=args)

    minimal_img_subscriber = MinimalImgSubscriber()

    rclpy.spin(minimal_img_subscriber)

    minimal_img_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
