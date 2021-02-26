# realsense-ros
Intel(R) RealSense(TM) ROS Wrapper for D400 series, SR300 Camera and T265 Tracking Module

Go to the src directory for the main README file.



# Realsense D435i
## Depth camera D435i reference page

[https://www.intelrealsense.com/depth-camera-d435i/ ](https://www.intelrealsense.com/depth-camera-d435i/ )

## ROS driver (SDK wrapper) 

Below you may find the official maintained repository for the realsense camera. Just follow the instructions on the readme file to install the ROS wrapper of the SDK. 

For testing purposes, you shall clone the repo below in your /seasony workspace. Remember to switch to the right branch -> foxy 

[https://github.com/seasony-org/realsense-ros.git](https://github.com/seasony-org/realsense-ros.git) 

For more detailed info go to [https://github.com/seasony-org/realsense-ros/blob/foxy/src/README.md](https://github.com/seasony-org/realsense-ros/blob/foxy/src/README.md) 

---------------------------------- Extra info but not official ----------------------------------

Once everything is installed, you may run 

```bash
ros2 launch realsense2_camera rs_launch.py enable_pointcloud:=true
```

Or  
```bash
ros2 launch realsense2_camera rs_launch.py enable_accel:=true enable_gyro:=true
```

For setting up the different parameters please check the link above for further information about the parameters available. 

## RVIZ 

If you want to visualize the camera output on RVIZ you should run this command: 

```bash
rviz2 -d ros2_realsense_ws/src/realsense-ros/realsense2_camera/launch/default.rviz
```

You will notice that the Images are not displayed. This is due to the fact that the default.rviz settings are not correct. More specifically, the image subscribers created by rviz do not have the right Quality of Service settings (https://index.ros.org/doc/ros2/Concepts/About-Quality-of-Service-Settings/ ) 

You may check the topic’s QoS profile by entering this command in your terminal:  

```bash
ros2 topic info <topic-name> --verbose 
```

This will return something like this: 

 
```bash
Type: sensor_msgs/msg/Image 

  

Publisher count: 1 

  

Node name: _ros2cli_rosbag2 

Node namespace: / 

Topic type: sensor_msgs/msg/Image 

Endpoint type: PUBLISHER 

GID: 01.0f.80.86.5b.31.05.00.01.00.00.00.00.00.17.03.00.00.00.00.00.00.00.00 

QoS profile: 

  Reliability: RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT 

  Durability: RMW_QOS_POLICY_DURABILITY_VOLATILE 

  Lifespan: 2147483651294967295 nanoseconds 

  Deadline: 2147483651294967295 nanoseconds 

  Liveliness: RMW_QOS_POLICY_LIVELINESS_AUTOMATIC 

  Liveliness lease duration: 2147483651294967295 nanoseconds
```

As you may see, the reliability of this publisher is set to _best_effort_  and the durability to _volatile_. You should change these settings in rviz to match them. To do that, on the Displays panel, go to each Image item. Click on the topic flied and there you could see the current settings. Update these settings to a [compatible](https://index.ros.org/doc/ros2/Concepts/About-Quality-of-Service-Settings/#qos-compatibilities) subscriber/publisher.

## Simple subscriber
As a prerequirement to run this node, you should have openCV install, as well as numpy. If when trying to run any errors arise, install the missing python dependencies (pip3 install <depencency-name>)
Also remember that you need to include sensor_msg type as part of your building depencencies for the package where you are running the script from. Also remember to follow all the ros2 steps when you create a new python node.

For a quick test, this is the code:

```python
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

```

