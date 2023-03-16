#import launch
#import launch_ros.actions

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, LogInfo, OpaqueFunction

from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PythonExpression


def generate_launch_description():

    qos = LaunchConfiguration('qos')
    intra = LaunchConfiguration('intra')
    fps = LaunchConfiguration('fps')
    left_device = LaunchConfiguration('left/device')
    right_device = LaunchConfiguration('right/device')

    trace = LaunchConfiguration('trace')

    return LaunchDescription([

        DeclareLaunchArgument('intra',default_value='false', description='intra optional'),
        DeclareLaunchArgument('qos', default_value='0', description='QoS used for input sensor topics'),
        DeclareLaunchArgument('fps', default_value='15', description='fps'),
        DeclareLaunchArgument('left/device', default_value='/dev/video0', description='left/device'),
        DeclareLaunchArgument('right/device', default_value='/dev/video1', description='right/device'),
        DeclareLaunchArgument('trace',default_value='false', description='trace'),


        Node(
            package='uvc_camera',executable='uvc_single_stereo_node',output="screen",
            #name='uvc_camera_stereo',
            #remappings=[('string_topic', '/talker')],
            emulate_tty=True,
            parameters=[
                        {"intra": intra,
                         "qos": qos,
                         "width": 320 ,
                         "height": 240,
                         "fps": fps,
                         "frame": "wide_stereo",
                         "auto_focus" : False,
                         "focus_absolute": 0,
                         "left/device": left_device,
                         "right/device": right_device,
                         "left/camera_info_url": "file:///home/nishi/colcon_ws/src/uvc_camera/example-left.yaml",
                         "right/camera_info_url": "file:///home/nishi/colcon_ws/src/uvc_camera/example-right.yaml",
                         "trace": trace}
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