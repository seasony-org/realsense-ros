# realsense-ros
Intel(R) RealSense(TM) ROS Wrapper for D400 series, SR300 Camera and T265 Tracking Module


Go to the src directory for the main README file.


# Realsense D435i
## Depth camera D435i reference page

[https://www.intelrealsense.com/depth-camera-d435i/ ](https://www.intelrealsense.com/depth-camera-d435i/ )

## ROS driver (SDK wrapper) 

For more detailed info go to [https://github.com/seasony-org/realsense-ros/blob/foxy/src/README.md](https://github.com/seasony-org/realsense-ros/blob/foxy/src/README.md) 

LibRealSense supported version: v2.45.0 (see [realsense2_camera release notes](https://github.com/IntelRealSense/realsense-ros/releases))

## Installation Instructions
This version supports ROS2 Dashing, Eloquent and Foxy.

   ### Step 1: Install the ROS2 distribution
   - #### Install [ROS2 Dashing](https://docs.ros.org/en/dashing/Installation/Ubuntu-Install-Debians.html), on Ubuntu 18.04 or [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html), on Ubuntu 20.04

### There are 2 sources to install realsense2_camera from:

* ### Method 1: The ROS distribution:

  *Ubuntu*

    realsense2_camera is available as a debian package of ROS distribution. It can be installed by typing:
    
    ```sudo apt-get install ros-$ROS_DISTRO-realsense2-camera```

    This will install both realsense2_camera and its dependents, including librealsense2 library.

    Notice:
    * The version of librealsense2 is almost always behind the one availeable in RealSense&trade; official repository.
    * librealsense2 is not built to use native v4l2 driver but the less stable RS-USB protocol. That is because the last is more general and operational on a larger variety of platforms. This have limitations when running multiple cameras and when using T265.
    * realsense2_description is available as a separate debian package of ROS distribution. It includes the 3D-models of the devices and is necessary for running launch files that include these models (i.e. view_model.launch.py). It can be installed by typing:
    `sudo apt-get install ros-$ROS_DISTRO-realsense2-description`


* ### Method 2: The RealSense&trade; distribution:
     > This option is demonstrated in the [.travis.yml](https://github.com/IntelRealSense/realsense-ros/blob/ros2/.travis.yml) file. It basically summerize the elaborate instructions in the following 2 steps:


   ### Step 1: Install the latest Intel&reg; RealSense&trade; SDK 2.0

   ### Install the latest Intel&reg; RealSense&trade; SDK 2.0
   - #### Install from [Debian Package](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages) - In that case treat yourself as a developer. Make sure you follow the instructions to also install librealsense2-dev and librealsense-dkms packages.

   #### OR
   - #### Build from sources by downloading the latest [Intel&reg; RealSense&trade; SDK 2.0](https://github.com/IntelRealSense/librealsense/releases/tag/v2.45.0) and follow the instructions under [Linux Installation](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)


   ### Step 3: Install Intel&reg; RealSense&trade; ROS2 wrapper from Sources
   - Create a ROS2 workspace
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src/
   ```
   - Clone the latest ROS2 Intel&reg; RealSense&trade;  wrapper from [here](https://github.com/IntelRealSense/realsense-ros.git) into '~/ros2_ws/src/'
   ```bashrc
   git clone --depth 1 --branch `git ls-remote --tags https://github.com/IntelRealSense/realsense-ros.git | grep -Po "(?<=tags/)3.\d+\.\d+" | sort -V | tail -1` https://github.com/IntelRealSense/realsense-ros.git
   cd ~/ros2_ws
   ```

  ### Step 4: Install dependencies:
   ```bash
  sudo apt-get install python3-rosdep -y
  sudo rosdep init
  rosdep update
  rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
  sudo apt purge ros-$ROS_DISTRO-librealsense2 -y
  ```

  ### Step 5: Build:
  ```bash
  colcon build
  ```

This will stream all camera sensors and publish on the appropriate ROS topics.

### Published Topics
The published topics differ according to the device and parameters.
After running the above command with D435i attached, the following list of topics will be available (This is a partial list. For full one type `ros2 topic list`):
- /camera/accel/imu_info
- /camera/color/camera_info
- /camera/color/image_raw
- /camera/depth/camera_info
- /camera/depth/color/points
- /camera/depth/image_rect_raw
- /camera/extrinsics/depth_to_color
- /camera/extrinsics/depth_to_infra1
- /camera/extrinsics/depth_to_infra2
- /camera/gyro/imu_info
- /camera/imu
- /camera/infra1/camera_info
- /camera/infra1/image_rect_raw
- /camera/infra2/camera_info
- /camera/infra2/image_rect_raw
- /camera/parameter_events
- /camera/rosout
- /parameter_events
- /rosout
- /tf_static

>Using an L515 device the list differs a little by adding a 4-bit confidence grade (published as a mono8 image):
>- /camera/confidence/camera_info
>- /camera/confidence/image_rect_raw
>
>It also replaces the 2 infrared topic sets with the single available one:
>- /camera/infra/camera_info
>- /camera/infra/image_raw

The "/camera" prefix is the namesapce specified in the given launch file.
When using D435 or D415, the gyro and accel topics wont be available. Likewise, other topics will be available when using T265 (see below).

### Available Parameters:
For the entire list of parameters type `ros2 param list`.

- **serial_no**: will attach to the device with the given serial number (*serial_no*) number. Default, attach to the first (in an inner list) RealSense device.
  - Note: serial number can also be defined with "_" prefix. For instance, serial number 831612073525 can be set in command line as `serial_no:=_831612073525`. That is a workaround until a better method will be found to ROS2's auto conversion of strings containing only digits into integers.
- **usb_port_id**: will attach to the device with the given USB port (*usb_port_id*). i.e 4-1, 4-2 etc. Default, ignore USB port when choosing a device.
- **device_type**: will attach to a device whose name includes the given *device_type* regular expression pattern. Default, ignore device type. For example, device_type:=d435 will match d435 and d435i. device_type=d435(?!i) will match d435 but not d435i.

- **rosbag_filename**: Will publish topics from rosbag file.
- **initial_reset**: On occasions the device was not closed properly and due to firmware issues needs to reset. If set to true, the device will reset prior to usage.
- **align_depth**: If set to true, will publish additional topics for the "aligned depth to color" image.: ```/camera/aligned_depth_to_color/image_raw```, ```/camera/aligned_depth_to_color/camera_info```.</br>
The pointcloud, if enabled, will be built based on the aligned_depth_to_color image.</br>
- **filters**: any of the following options, separated by commas:</br>
 - ```colorizer```: will color the depth image. On the depth topic an RGB image will be published, instead of the 16bit depth values .
 - ```pointcloud```: will add a pointcloud topic `/camera/depth/color/points`.
    * The texture of the pointcloud can be modified in rqt_reconfigure (see below) or using the parameters: `pointcloud_texture_stream` and `pointcloud_texture_index`. Run rqt_reconfigure to see available values for these parameters.</br>
    * The depth FOV and the texture FOV are not similar. By default, pointcloud is limited to the section of depth containing the texture. You can have a full depth to pointcloud, coloring the regions beyond the texture with zeros, by setting `allow_no_texture_points` to true.
    * pointcloud is of an unordered format by default. This can be changed by setting `ordered_pc` to true.
 - ```hdr_merge```: Allows depth image to be created by merging the information from 2 consecutive frames, taken with different exposure and gain values. The way to set exposure and gain values for each sequence in runtime is by first selecting the sequence id, using the `stereo_module.sequence_id` parameter and then modifying the `stereo_module.gain`, and `stereo_module.exposure`.</br> To view the effect on the infrared image for each sequence id use the `sequence_id_filter.sequence_id` parameter.</br> To initialize these parameters in start time use the following parameters:</br>
  `stereo_module.exposure.1`, `stereo_module.gain.1`, `stereo_module.exposure.2`, `stereo_module.gain.2`</br>
  \* For in-depth review of the subject please read the accompanying [white paper](https://dev.intelrealsense.com/docs/high-dynamic-range-with-stereoscopic-depth-cameras).

  
 - The following filters have detailed descriptions in : https://github.com/IntelRealSense/librealsense/blob/master/doc/post-processing-filters.md
   - ```disparity``` - convert depth to disparity before applying other filters and back.
   - ```spatial``` - filter the depth image spatially.
   - ```temporal``` - filter the depth image temporally.
   - ```hole_filling``` - apply hole-filling filter.
   - ```decimation``` - reduces depth scene complexity.
- **enable_sync**: gathers closest frames of different sensors, infra red, color and depth, to be sent with the same timetag. This happens automatically when such filters as pointcloud are enabled.
- ***<stream_type>*_width**, ***<stream_type>*_height**, ***<stream_type>*_fps**: <stream_type> can be any of *infra, color, fisheye, depth, gyro, accel, pose, confidence*. Sets the required format of the device. If the specified combination of parameters is not available by the device, the stream will be replaced with the default for that stream. Setting a value to 0, will choose the first format in the inner list. (i.e. consistent between runs but not defined).</br>*Note: for gyro accel and pose, only _fps option is meaningful.
- ***<stream_type>*_qos**, ***<stream_type>*_info_qos**, and ***<stream_type>*_extrinsics_qos**: <stream_type> can be any of *infra, color, fisheye, depth, gyro, accel, pose, confidence, pointcloud, imu*. Sets the QoS by which the topic is published. Available values are the following strings: SYSTEM_DEFAULT, PARAMETER_EVENTS, SERVICES_DEFAULT, PARAMETERS, DEFAULT, SENSOR_DATA, HID_DEFAULT (= DEFAULT with depth of 100), EXTRINSICS_DEFAULT (= DEFAULT with depth of 1 and transient local durabilty).
- **enable_*<stream_name>***: Choose whether to enable a specified stream or not. Default is true for images and false for orientation streams. <stream_name> can be any of *infra1, infra2, color, depth, fisheye, fisheye1, fisheye2, gyro, accel, pose, confidence*.

- ***<stream_name>*_frame_id**, ***<stream_name>*_optical_frame_id**, **aligned_depth_to_*<stream_name>*_frame_id**: Specify the different frame_id for the different frames. Especially important when using multiple cameras.

- **base_frame_id**: defines the frame_id all static transformations refers to.
- **odom_frame_id**: defines the origin coordinate system in ROS convention (X-Forward, Y-Left, Z-Up). pose topic defines the pose relative to that system.

- **unite_imu_method**: The D435i and T265 cameras have built in IMU components which produce 2 unrelated streams: *gyro* - which shows angular velocity and *accel* which shows linear acceleration. Each with it's own frequency. By default, 2 corresponding topics are available, each with only the relevant fields of the message sensor_msgs::Imu are filled out.
Setting *unite_imu_method* creates a new topic, *imu*, that replaces the default *gyro* and *accel* topics. The *imu* topic is published at the rate of the gyro. All the fields of the Imu message under the *imu* topic are filled out.
   - **linear_interpolation**: Every gyro message is attached by the an accel message interpolated to the gyro's timestamp.
   - **copy**: Every gyro message is attached by the last accel message.
- **clip_distance**: remove from the depth image all values above a given value (meters). Disable by giving negative value (default)
- **linear_accel_cov**, **angular_velocity_cov**: sets the variance given to the Imu readings. For the T265, these values are being modified by the inner confidence value.
- **hold_back_imu_for_frames**: Images processing takes time. Therefor there is a time gap between the moment the image arrives at the wrapper and the moment the image is published to the ROS environment. During this time, Imu messages keep on arriving and a situation is created where an image with earlier timestamp is published after Imu message with later timestamp. If that is a problem, setting *hold_back_imu_for_frames* to *true* will hold the Imu messages back while processing the images and then publish them all in a burst, thus keeping the order of publication as the order of arrival. Note that in either case, the timestamp in each message's header reflects the time of it's origin.
- **topic_odom_in**: For T265, add wheel odometry information through this topic. The code refers only to the *twist.linear* field in the message.
- **calib_odom_file**: For the T265 to include odometry input, it must be given a [configuration file](https://github.com/IntelRealSense/librealsense/blob/master/unit-tests/resources/calibration_odometry.json). Explanations can be found [here](https://github.com/IntelRealSense/librealsense/pull/3462). The calibration is done in ROS coordinates system.
- **publish_tf**: boolean, publish or not TF at all. Defaults to True.
- **tf_publish_rate**: double, positive values mean dynamic transform publication with specified rate, all other values mean static transform publication. Defaults to 0 
- **publish_odom_tf**: If True (default) publish TF from odom_frame to pose_frame.
- **infra_rgb**: When set to True (default: False), it configures the infrared camera to stream in RGB (color) mode, thus enabling the use of a RGB image in the same frame as the depth image, potentially avoiding frame transformation related errors. When this feature is required, you are additionally required to also enable `enable_infra:=true` for the infrared stream to be enabled.
  - **NOTE** The configuration required for `enable_infra` is independent of `enable_depth`
  - **NOTE** To enable the Infrared stream, you should enable `enable_infra:=true` NOT `enable_infra1:=true` nor `enable_infra2:=true`
  - **NOTE** This feature is only supported by Realsense sensors with RGB streams available from the `infra` cameras, which can be checked by observing the output of `rs-enumerate-devices`

### Available services:
- enable : Start/Stop all streaming sensors. Usage example: `ros2 service call /camera/enable std_srvs/srv/SetBool "{data: False}"`

## Using T265 ##

### Start the camera node
To start the camera node in ROS:

Once everything is installed, you may run 

```bash
ros2 launch realsense2_camera rs_launch.py enable_pointcloud:=true
```
Or  
```bash
ros2 launch realsense2_camera rs_launch.py enable_accel:=true enable_gyro:=true
```

For setting up the different parameters please check the link above for further information about the parameters available. 

Then, unit-tests can be run using the following command (use either python or python3):
```bash
cd ros2_ws
wget "https://librealsense.intel.com/rs-tests/TestData/outdoors_1color.bag" -P "records/"
wget "https://librealsense.intel.com/rs-tests/D435i_Depth_and_IMU_Stands_still.bag" -P "records/"
```

```bash
python3 src/realsense-ros/realsense2_camera/scripts/rs2_test.py --all
```

## RVIZ 

If you want to visualize the camera output on RVIZ you should run this command: 

```bash
rviz2 -d ros2_realsense_ws/src/realsense-ros/realsense2_camera/launch/default.rviz
```

You will notice that the Images are not displayed. This is due to the fact that the default.rviz settings are not correct. More specifically, the image subscribers created by rviz do not have the right Quality of Service settings (https://index.ros.org/doc/ros2/Concepts/About-Quality-of-Service-Settings/ ) 

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

