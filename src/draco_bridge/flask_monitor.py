#!/usr/bin/env python3

import json
import time
import threading
import socket
import subprocess
from datetime import datetime
from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO, emit
import psutil

app = Flask(__name__)
app.config['SECRET_KEY'] = 'draco_monitor_secret'
socketio = SocketIO(app, cors_allowed_origins="*")

class DracoMonitor:
    def __init__(self):
        self.encoder_status = "disconnected"
        self.decoder_status = "disconnected"
        self.compression_stats = {
            'original_size': 0,
            'compressed_size': 0,
            'compression_ratio': 0.0,
            'last_update': None
        }
        self.network_status = "disconnected"
        self.connection_count = 0
        self.is_monitoring = True
        self.network_bandwidth = {
            'bytes_sent_per_sec': 0,
            'bytes_recv_per_sec': 0,
            'total_bytes_sent': 0,
            'total_bytes_recv': 0
        }
        self.last_net_io = None
        self.last_decoder_report = None  # 마지막 디코더 상태 보고 시간
        
    def check_encoder_status(self):
        """인코더 서버 상태 확인"""
        try:
            # 포트 50051이 열려있는지 확인
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(1)
            result = sock.connect_ex(('127.0.0.1', 50051))
            sock.close()
            
            if result == 0:
                self.encoder_status = "connected"
            else:
                # 프로세스로도 확인
                for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                    try:
                        if 'encoder_server' in ' '.join(proc.info['cmdline'] or []):
                            self.encoder_status = "connected"
                            return
                    except (psutil.NoSuchProcess, psutil.AccessDenied):
                        continue
                self.encoder_status = "disconnected"
        except Exception:
            self.encoder_status = "disconnected"
    
    def check_decoder_status(self):
        """디코더 클라이언트 상태 확인"""
        try:
            # 마지막 보고 시간이 10초 이상 지났으면 disconnected로 처리
            if self.last_decoder_report and (time.time() - self.last_decoder_report) > 10:
                if self.decoder_status == "connected":
                    self.decoder_status = "disconnected"
                    print(f"Decoder timeout - marking as disconnected")
                return
            
            # 디코더 프로세스가 실행 중인지 확인
            for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                try:
                    cmdline = ' '.join(proc.info['cmdline'] or [])
                    if 'decoder_client' in cmdline or 'decoder_receiver' in cmdline:
                        # 프로세스는 있지만 최근 보고가 없으면 연결 불안정으로 처리
                        if not self.last_decoder_report or (time.time() - self.last_decoder_report) > 5:
                            if self.decoder_status == "connected":
                                self.decoder_status = "disconnected"
                        return
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue
            self.decoder_status = "disconnected"
        except Exception:
            self.decoder_status = "disconnected"
    
    def check_network_status(self):
        """네트워크 연결 상태 및 대역폭 확인"""
        try:
            # TCP 연결 수 확인
            connections = psutil.net_connections(kind='tcp')
            tcp_connections = [conn for conn in connections if conn.laddr.port == 50051]
            self.connection_count = len(tcp_connections)
            
            if self.connection_count > 0:
                self.network_status = "connected"
            else:
                self.network_status = "disconnected"
            
            # 네트워크 대역폭 계산
            current_net_io = psutil.net_io_counters()
            
            if self.last_net_io is not None:
                # 2초 간격으로 대역폭 계산
                time_diff = 2.0  # monitoring_loop에서 2초마다 호출
                bytes_sent_diff = current_net_io.bytes_sent - self.last_net_io.bytes_sent
                bytes_recv_diff = current_net_io.bytes_recv - self.last_net_io.bytes_recv
                
                self.network_bandwidth = {
                    'bytes_sent_per_sec': bytes_sent_diff / time_diff,
                    'bytes_recv_per_sec': bytes_recv_diff / time_diff,
                    'total_bytes_sent': current_net_io.bytes_sent,
                    'total_bytes_recv': current_net_io.bytes_recv
                }
            
            self.last_net_io = current_net_io
            
        except Exception:
            self.network_status = "disconnected"
    
    def get_system_info(self):
        """시스템 정보 수집"""
        try:
            # CPU, 메모리 사용률
            cpu_percent = psutil.cpu_percent(interval=1)
            memory = psutil.virtual_memory()
            
            # 네트워크 통계
            net_io = psutil.net_io_counters()
            
            return {
                'cpu_percent': cpu_percent,
                'memory_percent': memory.percent,
                'memory_used_gb': round(memory.used / (1024**3), 2),
                'memory_total_gb': round(memory.total / (1024**3), 2),
                'bytes_sent': net_io.bytes_sent,
                'bytes_recv': net_io.bytes_recv
            }
        except Exception:
            return {
                'cpu_percent': 0,
                'memory_percent': 0,
                'memory_used_gb': 0,
                'memory_total_gb': 0,
                'bytes_sent': 0,
                'bytes_recv': 0
            }
    
    def update_compression_stats(self, original_size, compressed_size):
        """압축 통계 업데이트"""
        self.compression_stats = {
            'original_size': original_size,
            'compressed_size': compressed_size,
            'compression_ratio': round((1 - compressed_size/original_size) * 100, 2) if original_size > 0 else 0,
            'last_update': datetime.now().strftime('%H:%M:%S')
        }
    
    def get_status_data(self):
        """전체 상태 데이터 반환"""
        return {
            'encoder_status': self.encoder_status,
            'decoder_status': self.decoder_status,
            'network_status': self.network_status,
            'connection_count': self.connection_count,
            'compression_stats': self.compression_stats,
            'network_bandwidth': self.network_bandwidth,
            'system_info': self.get_system_info(),
            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        }

# 전역 모니터 인스턴스
monitor = DracoMonitor()

@app.route('/')
def index():
    """메인 페이지"""
    return render_template('monitor.html')

@app.route('/api/status')
def api_status():
    """상태 API"""
    return jsonify(monitor.get_status_data())

@app.route('/api/compression_stats', methods=['POST'])
def api_compression_stats():
    """압축 통계 수신 API"""
    try:
        data = request.get_json()
        original_size = data.get('original_size', 0)
        compressed_size = data.get('compressed_size', 0)
        
        monitor.update_compression_stats(original_size, compressed_size)
        
        # 실시간 업데이트 전송
        socketio.emit('status_update', monitor.get_status_data())
        
        return jsonify({'status': 'success'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)})

@app.route('/api/decoder_status', methods=['POST'])
def api_decoder_status():
    """디코더 상태 수신 API"""
    try:
        data = request.get_json()
        decoder_status = data.get('decoder_status', 'disconnected')
        
        monitor.decoder_status = decoder_status
        monitor.last_decoder_report = time.time()  # 마지막 보고 시간 기록
        
        # 실시간 업데이트 전송
        socketio.emit('status_update', monitor.get_status_data())
        
        return jsonify({'status': 'success'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)})

@socketio.on('connect')
def handle_connect():
    """클라이언트 연결"""
    print('Client connected')
    emit('status_update', monitor.get_status_data())

@socketio.on('disconnect')
def handle_disconnect():
    """클라이언트 연결 해제"""
    print('Client disconnected')

def monitoring_loop():
    """모니터링 루프"""
    while monitor.is_monitoring:
        try:
            # 상태 확인
            monitor.check_encoder_status()
            monitor.check_decoder_status()
            monitor.check_network_status()
            
            # WebSocket으로 실시간 업데이트
            socketio.emit('status_update', monitor.get_status_data())
            
            time.sleep(2)  # 2초마다 업데이트
        except Exception as e:
            print(f"Monitoring error: {e}")
            time.sleep(5)

def start_monitoring():
    """모니터링 시작"""
    monitor_thread = threading.Thread(target=monitoring_loop, daemon=True)
    monitor_thread.start()

if __name__ == '__main__':
    start_monitoring()
    print("Draco Monitor starting...")
    print("Access the web interface at: http://localhost:5000")
    socketio.run(app, host='0.0.0.0', port=5000, debug=True, allow_unsafe_werkzeug=True)
