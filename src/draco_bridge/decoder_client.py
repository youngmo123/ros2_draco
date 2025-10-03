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

def get_local_ip():
    """로컬 IP 주소 자동 감지"""
    try:
        # 외부 서버(Google DNS)에 연결 시도하여 로컬 IP 확인
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
        s.close()
        return local_ip
    except Exception:
        # 실패 시 localhost 반환
        return '127.0.0.1'

def discover_draco_server(port=50051, timeout=2.0, logger=None):
    """Draco 서버 자동 탐지"""
    try:
        # 로컬 네트워크 대역 스캔
        local_ip = get_local_ip()
        base_ip = '.'.join(local_ip.split('.')[:-1])  # 192.168.0.x에서 192.168.0 추출
        
        if logger:
            logger.info(f'[Decoder] Searching for Draco server in {base_ip}.x range...')
        
        for i in range(1, 255):  # 1-254 범위 스캔
            target_ip = f'{base_ip}.{i}'
            if target_ip == local_ip:
                continue  # 자기 자신은 건너뛰기
                
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(timeout)
                result = sock.connect_ex((target_ip, port))
                sock.close()
                
                if result == 0:  # 연결 성공
                    if logger:
                        logger.info(f'[Decoder] Found Draco server at {target_ip}:{port}')
                    return target_ip
                    
            except Exception:
                continue
                
        if logger:
            logger.warn(f'[Decoder] No Draco server found in network range')
        return None
        
    except Exception as e:
        if logger:
            logger.error(f'[Decoder] Server discovery failed: {e}')
        return None

class DecoderClient(Node):
    def __init__(self):
        super().__init__('draco_decoder_client')

        # 파라미터 선언
        self.declare_parameter('tcp_server_ip', 'auto')
        self.declare_parameter('tcp_server_port', 50051)
        self.declare_parameter('output_topic', '/sensing/lidar/points_raw')
        self.declare_parameter('frame_id', 'lidar_link')
        self.declare_parameter('qos_reliability', 'reliable')
        self.declare_parameter('qos_depth', 10)
        
        # 파라미터 값 가져오기
        default_ip = self.get_parameter('tcp_server_ip').get_parameter_value().string_value
        if default_ip == 'auto':
            # 먼저 서버 자동 탐지 시도
            self.tcp_server_ip = discover_draco_server(
                port=self.get_parameter('tcp_server_port').get_parameter_value().integer_value,
                timeout=1.0,
                logger=self.get_logger()
            )
            if self.tcp_server_ip is None:
                # 서버 탐지 실패 시 로컬 IP 사용 (같은 컴퓨터에서 테스트)
                self.tcp_server_ip = get_local_ip()
                self.get_logger().info(f'[Decoder] Auto-detected IP: {self.tcp_server_ip}')
        else:
            self.tcp_server_ip = default_ip
        
        self.tcp_server_port = self.get_parameter('tcp_server_port').get_parameter_value().integer_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.qos_reliability = self.get_parameter('qos_reliability').get_parameter_value().string_value
        self.qos_depth = self.get_parameter('qos_depth').get_parameter_value().integer_value

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