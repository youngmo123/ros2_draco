# launch/decoder_receiver.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    host = LaunchConfiguration('host')
    port = LaunchConfiguration('port')
    out_topic = LaunchConfiguration('output_topic')
    frame_id = LaunchConfiguration('frame_id')
    reliability = LaunchConfiguration('reliability')

    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='auto'),  # 'auto'로 설정하면 자동 감지
        DeclareLaunchArgument('port', default_value='50051'),
        DeclareLaunchArgument('output_topic', default_value='/sensing/lidar/points_raw'),
        DeclareLaunchArgument('frame_id', default_value='lidar_link'),
        DeclareLaunchArgument('reliability', default_value='reliable'),

        Node(
            package='draco_bridge',
            executable='decoder_client',
            name='decoder_client',
            output='screen',
            parameters=[{
                'tcp_server_ip': host,
                'tcp_server_port': int(port.perform(None) or 50051),
                'output_topic': out_topic,
                'frame_id': frame_id,
                'qos_reliability': reliability,
                'qos_depth': 10,
            }]
        ),
    ])
