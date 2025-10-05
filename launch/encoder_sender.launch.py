# launch/encoder_sender.launch.py
import os
import glob
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    bag = LaunchConfiguration('bag')
    input_topic = LaunchConfiguration('input_topic')
    bind_ip = LaunchConfiguration('bind_ip')
    port = LaunchConfiguration('port')

    # ros2bagfile 디렉토리에서 첫 번째 rosbag 폴더 자동 찾기
    rosbag_dir = '/home/youngmo/ros2bagfile'
    rosbag_folders = glob.glob(os.path.join(rosbag_dir, 'rosbag2_*'))
    default_bag = rosbag_folders[0] if rosbag_folders else '/home/youngmo/ros2bagfile'

    return LaunchDescription([
        DeclareLaunchArgument('bag', default_value=default_bag),
        DeclareLaunchArgument('input_topic', default_value='/sensing/lidar/top/pointcloud'),
        DeclareLaunchArgument('bind_ip', default_value='auto'),  # 'auto'로 설정하면 자동 감지
        DeclareLaunchArgument('port', default_value='50051'),

        ExecuteProcess(
            cmd=['python3', '/home/youngmo/ros2_draco/scripts/play_all_bags.py'],
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
                'tcp_port': 50051,
            }]
        ),
    ])
