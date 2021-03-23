# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch realsense2_camera node."""
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

configurable_parameters = [{'name': 'camera_name', 'default': 'camera', 'description': 'camera unique name'},
                           {'name': 'serial_no', 'default': '', 'description': 'choose device by serial number'},
                           {'name': 'usb_port_id', 'default': '', 'description': 'choose device by usb port id'},
                           {'name': 'device_type', 'default': '', 'description': 'choose device by type'},
                           {'name': 'config_file', 'default': '', 'description': 'yaml config file'},
                           {'name': 'enable_pointcloud', 'default': 'false', 'description': 'enable pointcloud'},
                           {'name': 'unite_imu_method', 'default': '', 'description': '[copy|linear_interpolation]'},
                           {'name': 'json_file_path', 'default': '', 'description': 'allows advanced configuration'},
                           {'name': 'output', 'default': 'screen', 'description': 'pipe node output [screen|log]'},
                           {'name': 'depth_width', 'default': '640', 'description': 'depth image width'},
                           {'name': 'depth_height', 'default': '480', 'description': 'depth image height'},
                           {'name': 'enable_depth', 'default': 'true', 'description': 'enable depth stream'},
                           {'name': 'color_width', 'default': '640', 'description': 'color image width'},
                           {'name': 'color_height', 'default': '480', 'description': 'color image height'},
                           {'name': 'enable_color', 'default': 'true', 'description': 'enable color stream'},
                           {'name': 'infra_width', 'default': '640', 'description': 'infra width'},
                           {'name': 'infra_height', 'default': '480', 'description': 'infra width'},
                           {'name': 'enable_infra1', 'default': 'true', 'description': 'enable infra1 stream'},
                           {'name': 'enable_infra2', 'default': 'true', 'description': 'enable infra2 stream'},
                           {'name': 'infra_rgb', 'default': 'false', 'description': 'enable infra2 stream'},
                           {'name': 'fisheye_width', 'default': '848', 'description': 'fisheye width'},
                           {'name': 'fisheye_height', 'default': '800', 'description': 'fisheye width'},
                           {'name': 'enable_fisheye1', 'default': 'true', 'description': 'enable fisheye1 stream'},
                           {'name': 'enable_fisheye2', 'default': 'true', 'description': 'enable fisheye2 stream'},
                           {'name': 'confidence_width', 'default': '640', 'description': 'depth image width'},
                           {'name': 'confidence_height', 'default': '480', 'description': 'depth image height'},
                           {'name': 'enable_confidence', 'default': 'true', 'description': 'enable depth stream'},
                           {'name': 'fisheye_fps', 'default': '30.', 'description': ''},
                           {'name': 'depth_fps', 'default': '30.', 'description': ''},
                           {'name': 'confidence_fps', 'default': '30.', 'description': ''},
                           {'name': 'infra_fps', 'default': '30.', 'description': ''},
                           {'name': 'color_fps', 'default': '30.', 'description': ''},
                           {'name': 'gyro_fps', 'default': '400.', 'description': ''},
                           {'name': 'accel_fps', 'default': '250.', 'description': ''},
                           {'name': 'color_qos', 'default': 'SENSOR_DATA', 'description': 'QoS profile name'},
                           {'name': 'confidence_qos', 'default': 'SENSOR_DATA', 'description': 'QoS profile name'},
                           {'name': 'depth_qos', 'default': 'SENSOR_DATA', 'description': 'QoS profile name'},
                           {'name': 'fisheye_qos', 'default': 'SENSOR_DATA', 'description': 'QoS profile name'},
                           {'name': 'infra_qos', 'default': 'SENSOR_DATA', 'description': 'QoS profile name'},
                           {'name': 'enable_gyro', 'default': 'false', 'description': ''},
                           {'name': 'enable_accel', 'default': 'false', 'description': ''},
                           {'name': 'pointcloud_texture_stream', 'default': 'RS2_STREAM_COLOR',
                            'description': 'testure stream for pointcloud'},
                           {'name': 'pointcloud_texture_index', 'default': '0',
                            'description': 'testure stream index for pointcloud'},
                           {'name': 'enable_sync', 'default': 'false', 'description': ''},
                           {'name': 'align_depth', 'default': 'false', 'description': ''},
                           {'name': 'filters', 'default': '', 'description': ''},
                           {'name': 'clip_distance', 'default': '-2.', 'description': ''},
                           {'name': 'linear_accel_cov', 'default': '0.01', 'description': ''},
                           {'name': 'initial_reset', 'default': 'false', 'description': ''},
                           {'name': 'allow_no_texture_points', 'default': 'false', 'description': ''},
                           {'name': 'ordered_pc', 'default': 'false', 'description': ''},
                           {'name': 'calib_odom_file', 'default': '', 'description': ''},
                           {'name': 'topic_odom_in', 'default': '', 'description': 'topic for T265 wheel odometry'},
                           {'name': 'tf_publish_rate', 'default': '0.0', 'description': 'Rate of publishing static_tf'},
                           {'name': 'rosbag_filename', 'default': '',
                            'description': 'A realsense bagfile to run from as a device'},
                           {'name': 'base_frame_id', 'default': 'link', 'description': 'base frame id'},
                           {'name': 'depth_frame_id', 'default': 'depth_frame', 'description': 'depth frame id'},
                           {'name': 'depth_optical_frame_id', 'default': 'depth_optical_frame',
                            'description': 'depth optical frame id'},
                           {'name': 'color_frame_id', 'default': 'color_frame', 'description': 'color frame id'},
                           {'name': 'color_optical_frame_id', 'default': 'color_optical_frame',
                            'description': 'color optical frame id'},
                           {'name': 'infra1_frame_id', 'default': 'infra1_frame', 'description': 'infra1 frame id'},
                           {'name': 'infra1_optical_frame_id', 'default': 'infra1_optical_frame',
                            'description': 'infra1 optical frame id'},
                           {'name': 'infra2_frame_id', 'default': 'infra2_frame', 'description': 'infra2 frame id'},
                           {'name': 'infra2_optical_frame_id', 'default': 'infra2_optical_frame',
                            'description': 'infra2 optical frame id'},
                           ]


def declare_configurable_parameters(parameters):
    frames_id_list = ['base_frame_id',
                      'depth_frame_id',
                      'depth_optical_frame_id',
                      'color_frame_id',
                      'color_optical_frame_id',
                      'infra1_frame_id',
                      'infra1_optical_frame_id',
                      'infra2_frame_id',
                      'infra2_optical_frame_id']

    for a, param in enumerate(parameters):
        if param['name'] == 'camera_name':
            camera_name = 'camera_front' #change this to the right camera name to match the urdf file
            param['default'] = camera_name
        elif param['name'] in frames_id_list:
            if camera_name:
                param['default'] = camera_name + '_' + param['default']
            else:
                param['default'] = 'camera_' + param['default']

    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for
            param in parameters]


def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])


def generate_launch_description():
    return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
        # Realsense
        launch_ros.actions.Node(
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('config_file'), "' == ''"])),
            package='realsense2_camera',
            namespace=LaunchConfiguration("camera_name"),
            name=LaunchConfiguration("camera_name"),
            executable='realsense2_camera_node',
            parameters=[set_configurable_parameters(configurable_parameters)
                        ],
            output='screen',
            emulate_tty=True,
        ),
        launch_ros.actions.Node(
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('config_file'), "' != ''"])),
            package='realsense2_camera',
            namespace=LaunchConfiguration("camera_name"),
            name=LaunchConfiguration("camera_name"),
            executable='realsense2_camera_node',
            parameters=[set_configurable_parameters(configurable_parameters)
                , {LaunchConfiguration("config_file")}
                        ],
            output='screen',
            emulate_tty=True,
        ),
    ])
