# launch/encoder_sender.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    bag = LaunchConfiguration('bag')
    input_topic = LaunchConfiguration('input_topic')
    bind_ip = LaunchConfiguration('bind_ip')
    port = LaunchConfiguration('port')

    return LaunchDescription([
        DeclareLaunchArgument('bag', default_value='/home/youngmo/Downloads/rosbag2_2024_09_24-14_28_57'),
        DeclareLaunchArgument('input_topic', default_value='/sensing/lidar/top/pointcloud'),
        DeclareLaunchArgument('bind_ip', default_value='127.0.0.1'),
        DeclareLaunchArgument('port', default_value='50051'),

        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag, '--clock', '--loop', '--rate', '1.0'],
            output='screen'
        ),

        Node(
            package='draco_bridge',
            executable='encoder_server',
            name='encoder_server',
            output='screen',
            parameters=[{
                'input_topic': input_topic,
                'output_topic': '/lidar_compressed',
                'tcp_bind_ip': bind_ip,
                'tcp_port': int(port.perform(None) or 50051),  # 일부 런처에서 int 캐스팅 이슈 회피용
            }]
        ),
    ])
