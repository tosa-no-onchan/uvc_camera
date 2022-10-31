#import launch
#import launch_ros.actions

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uvc_camera',
            executable='uvc_single_stereo_node',
            output='screen',
            #name='uvc_camera_stereo',
            #remappings=[('string_topic', '/talker')],
            emulate_tty=True,
            parameters=[
                        {"width": 320 ,
                         "height": 240,
                         "fps": 15,
                         "frame": "wide_stereo",
                         "auto_focus" : False,
                         "focus_absolute": 0,
                         "left/device": "/dev/video0",
                         "right/device": "/dev/video1",
                         "left/camera_info_url": "file:///home/nishi/colcon_ws/src/uvc_camera/example-left.yaml",
                         "right/camera_info_url": "file:///home/nishi/colcon_ws/src/uvc_camera/example-right.yaml"}
            ]
        )
    ])

'''
#https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-CPP.html
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="cpp_parameters",
            executable="minimal_param_node",
            name="custom_minimal_param_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"my_parameter": "earth"}
            ]
        )
    ])
'''