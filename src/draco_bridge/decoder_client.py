# src/draco_bridge/decoder_client.py
import socket
import struct
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from draco_bridge.utils import decode_draco

class DecoderClient(Node):
    def __init__(self):
        super().__init__('draco_decoder_client')

        self.tcp_server_ip = self.declare_parameter('tcp_server_ip', '127.0.0.1').get_string_value()
        self.tcp_server_port = self.declare_parameter('tcp_server_port', 50051).get_parameter_value().integer_value
        self.output_topic = self.declare_parameter('output_topic', '/sensing/lidar/points_raw').get_string_value()
        self.frame_id = self.declare_parameter('frame_id', 'lidar_link').get_string_value()
        self.qos_reliability = self.declare_parameter('qos_reliability', 'reliable').get_string_value()
        self.qos_depth = self.declare_parameter('qos_depth', 10).get_parameter_value().integer_value

        # QoS 설정
        if self.qos_reliability == 'reliable':
            reliability = ReliabilityPolicy.RELIABLE
        else:
            reliability = ReliabilityPolicy.BEST_EFFORT

        pub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=self.qos_depth,
            reliability=reliability,
            durability=DurabilityPolicy.VOLATILE
        )
        self.pub = self.create_publisher(PointCloud2, self.output_topic, pub_qos)

        self.sock = None
        self.connected = False
        self.running = True
        self.template_msg = None  # 첫 번째 메시지를 템플릿으로 사용

        threading.Thread(target=self.connect_and_receive_loop, daemon=True).start()

    def connect_and_receive_loop(self):
        while self.running:
            try:
                if not self.connected:
                    self.connect_to_server()
                
                if self.connected:
                    self.receive_data()
                    
            except Exception as e:
                self.get_logger().error(f'Connection error: {e}')
                self.connected = False
                if self.sock:
                    self.sock.close()
                    self.sock = None
                
                if self.running:
                    self.get_logger().info(f'Reconnecting in 5 seconds...')
                    time.sleep(5)

    def connect_to_server(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(10.0)
            self.sock.connect((self.tcp_server_ip, self.tcp_server_port))
            self.connected = True
            self.get_logger().info(f'[Decoder] Connected to server {self.tcp_server_ip}:{self.tcp_server_port}')
            
        except Exception as e:
            self.get_logger().error(f'[Decoder] Failed to connect to server: {e}')
            self.connected = False
            if self.sock:
                self.sock.close()
                self.sock = None

    def receive_data(self):
        try:
            while self.connected and self.running:
                # 데이터 크기 헤더 수신 (4바이트)
                size_header = self._receive_exact_bytes(4)
                if not size_header:
                    break
                
                data_size = struct.unpack('<I', size_header)[0]
                
                # 실제 데이터 수신
                data = self._receive_exact_bytes(data_size)
                if not data:
                    break
                
                # 데이터 처리
                self.process_received_data(data)
                
        except Exception as e:
            self.get_logger().error(f'[Decoder] Error receiving data: {e}')
            self.connected = False

    def _receive_exact_bytes(self, num_bytes):
        """정확한 바이트 수만큼 수신"""
        data = b''
        while len(data) < num_bytes:
            try:
                chunk = self.sock.recv(num_bytes - len(data))
                if not chunk:
                    return None
                data += chunk
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f'[Decoder] Error receiving bytes: {e}')
                return None
        return data

    def process_received_data(self, data):
        """수신된 데이터 처리 및 포인트 클라우드 발행"""
        try:
            # 템플릿 메시지가 없으면 기본 템플릿 생성
            if self.template_msg is None:
                self.template_msg = PointCloud2()
                self.template_msg.header.frame_id = self.frame_id
                self.template_msg.is_bigendian = False
                self.template_msg.is_dense = True
                # 기본 필드 설정
                from sensor_msgs.msg import PointField
                self.template_msg.fields = [
                    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
                ]
            
            # Draco 압축 해제
            pointcloud_msg = decode_draco(data, self.template_msg)
            
            # 타임스탬프 업데이트
            pointcloud_msg.header.stamp = self.get_clock().now().to_msg()
            
            # 토픽 발행
            self.pub.publish(pointcloud_msg)
            
        except Exception as e:
            self.get_logger().error(f'[Decoder] Error processing received data: {e}')

    def destroy_node(self):
        """노드 종료 시 정리"""
        self.running = False
        self.connected = False
        
        if self.sock:
            self.sock.close()
        
        super().destroy_node()

def main():
    rclpy.init()
    node = DecoderClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()