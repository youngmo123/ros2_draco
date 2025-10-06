#!/usr/bin/env python3
"""
네트워크 대역폭 측정 모듈
송신/수신 대역폭을 실시간으로 측정하고 iPerf와 유사한 결과를 제공
"""

import socket
import time
import threading
import json
import struct
import psutil
from datetime import datetime
from typing import Dict, Tuple, Optional

class BandwidthMonitor:
    def __init__(self, target_ip: str = "192.168.0.16", target_port: int = 5001):
        self.target_ip = target_ip
        self.target_port = target_port
        self.running = False
        self.test_duration = 10  # 10초 테스트
        self.test_interval = 30  # 30초마다 테스트
        
        # 측정 결과 저장
        self.last_results = {
            'upload_speed': 0.0,      # Mbps
            'download_speed': 0.0,    # Mbps
            'upload_bytes': 0,        # 총 송신 바이트
            'download_bytes': 0,      # 총 수신 바이트
            'upload_bps': 0.0,        # bps
            'download_bps': 0.0,      # bps
            'last_test_time': None,
            'test_status': 'idle'
        }
        
        # 네트워크 인터페이스 모니터링
        self.interface_stats = {}
        self.last_interface_check = None
        
    def start_monitoring(self):
        """대역폭 모니터링 시작"""
        self.running = True
        print(f"[BandwidthMonitor] Starting bandwidth monitoring...")
        print(f"[BandwidthMonitor] Target: {self.target_ip}:{self.target_port}")
        
        # 백그라운드 스레드에서 주기적 측정
        self.monitor_thread = threading.Thread(target=self._monitoring_loop, daemon=True)
        self.monitor_thread.start()
        
        # 네트워크 인터페이스 모니터링
        self.interface_thread = threading.Thread(target=self._interface_monitoring_loop, daemon=True)
        self.interface_thread.start()
    
    def stop_monitoring(self):
        """대역폭 모니터링 중지"""
        self.running = False
        print("[BandwidthMonitor] Stopping bandwidth monitoring...")
    
    def _monitoring_loop(self):
        """주기적으로 대역폭 테스트 수행"""
        while self.running:
            try:
                # 대역폭 테스트 수행
                self._perform_bandwidth_test()
                time.sleep(self.test_interval)
            except Exception as e:
                print(f"[BandwidthMonitor] Error in monitoring loop: {e}")
                time.sleep(5)
    
    def _interface_monitoring_loop(self):
        """네트워크 인터페이스 통계 모니터링"""
        while self.running:
            try:
                self._update_interface_stats()
                time.sleep(1)  # 1초마다 업데이트
            except Exception as e:
                print(f"[BandwidthMonitor] Error in interface monitoring: {e}")
                time.sleep(1)
    
    def _update_interface_stats(self):
        """네트워크 인터페이스 통계 업데이트"""
        try:
            current_time = time.time()
            
            # 현재 네트워크 통계 가져오기
            net_io = psutil.net_io_counters()
            
            if self.last_interface_check:
                time_diff = current_time - self.last_interface_check
                
                # 초당 바이트 계산
                bytes_sent_per_sec = (net_io.bytes_sent - self.interface_stats.get('bytes_sent', 0)) / time_diff
                bytes_recv_per_sec = (net_io.bytes_recv - self.interface_stats.get('bytes_recv', 0)) / time_diff
                
                # bps로 변환
                upload_bps = bytes_sent_per_sec * 8
                download_bps = bytes_recv_per_sec * 8
                
                # Mbps로 변환
                upload_mbps = upload_bps / 1_000_000
                download_mbps = download_bps / 1_000_000
                
                # 실시간 인터페이스 통계 업데이트
                self.last_results.update({
                    'upload_bps': upload_bps,
                    'download_bps': download_bps,
                    'upload_mbps_realtime': upload_mbps,
                    'download_mbps_realtime': download_mbps,
                    'upload_bytes': net_io.bytes_sent,
                    'download_bytes': net_io.bytes_recv
                })
            
            # 현재 통계 저장
            self.interface_stats = {
                'bytes_sent': net_io.bytes_sent,
                'bytes_recv': net_io.bytes_recv,
                'packets_sent': net_io.packets_sent,
                'packets_recv': net_io.packets_recv
            }
            self.last_interface_check = current_time
            
        except Exception as e:
            print(f"[BandwidthMonitor] Error updating interface stats: {e}")
    
    def _perform_bandwidth_test(self):
        """실제 대역폭 테스트 수행 (TCP 소켓 기반)"""
        try:
            print(f"[BandwidthMonitor] Starting bandwidth test to {self.target_ip}:{self.target_port}")
            self.last_results['test_status'] = 'testing'
            
            # 업로드 테스트
            upload_speed = self._test_upload()
            
            # 다운로드 테스트  
            download_speed = self._test_download()
            
            # 결과 업데이트
            self.last_results.update({
                'upload_speed': upload_speed,
                'download_speed': download_speed,
                'upload_bps': upload_speed * 1_000_000,  # Mbps to bps
                'download_bps': download_speed * 1_000_000,  # Mbps to bps
                'last_test_time': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                'test_status': 'completed'
            })
            
            print(f"[BandwidthMonitor] Test completed - Upload: {upload_speed:.2f} Mbps, Download: {download_speed:.2f} Mbps")
            
        except Exception as e:
            print(f"[BandwidthMonitor] Bandwidth test failed: {e}")
            self.last_results['test_status'] = 'failed'
    
    def _test_upload(self) -> float:
        """업로드 속도 테스트"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(self.test_duration + 5)
            sock.connect((self.target_ip, self.target_port))
            
            # 테스트 데이터 생성 (1MB)
            test_data = b'X' * 1024 * 1024
            
            start_time = time.time()
            total_bytes = 0
            
            # 테스트 시간 동안 데이터 전송
            while time.time() - start_time < self.test_duration:
                try:
                    sent = sock.send(test_data)
                    total_bytes += sent
                    if sent == 0:
                        break
                except socket.timeout:
                    break
            
            end_time = time.time()
            duration = end_time - start_time
            
            sock.close()
            
            # Mbps 계산
            bytes_per_second = total_bytes / duration
            mbps = (bytes_per_second * 8) / 1_000_000
            
            return mbps
            
        except Exception as e:
            print(f"[BandwidthMonitor] Upload test error: {e}")
            return 0.0
    
    def _test_download(self) -> float:
        """다운로드 속도 테스트"""
        try:
            # 간단한 다운로드 테스트 (로컬 파일 읽기 시뮬레이션)
            # 실제로는 서버에서 데이터를 받아야 하지만, 여기서는 네트워크 인터페이스 통계 사용
            
            # psutil을 사용한 네트워크 수신 속도 계산
            net_io = psutil.net_io_counters()
            bytes_recv = net_io.bytes_recv
            
            time.sleep(1)  # 1초 대기
            
            net_io_after = psutil.net_io_counters()
            bytes_recv_after = net_io_after.bytes_recv
            
            # 초당 바이트 계산
            bytes_per_second = bytes_recv_after - bytes_recv
            
            # Mbps 계산
            mbps = (bytes_per_second * 8) / 1_000_000
            
            return max(mbps, 0.0)
            
        except Exception as e:
            print(f"[BandwidthMonitor] Download test error: {e}")
            return 0.0
    
    def get_results(self) -> Dict:
        """현재 측정 결과 반환"""
        return self.last_results.copy()
    
    def get_formatted_results(self) -> Dict:
        """포맷된 측정 결과 반환"""
        results = self.get_results()
        
        return {
            'upload_speed_mbps': f"{results['upload_speed']:.2f}",
            'download_speed_mbps': f"{results['download_speed']:.2f}",
            'upload_speed_bps': f"{results['upload_bps']:,.0f}",
            'download_speed_bps': f"{results['download_bps']:,.0f}",
            'upload_speed_formatted': self._format_speed(results['upload_bps']),
            'download_speed_formatted': self._format_speed(results['download_bps']),
            'realtime_upload_mbps': f"{results.get('upload_mbps_realtime', 0):.2f}",
            'realtime_download_mbps': f"{results.get('download_mbps_realtime', 0):.2f}",
            'last_test_time': results['last_test_time'],
            'test_status': results['test_status'],
            'total_upload_bytes': f"{results['upload_bytes']:,}",
            'total_download_bytes': f"{results['download_bytes']:,}"
        }
    
    def _format_speed(self, bps: float) -> str:
        """속도를 읽기 쉬운 형태로 포맷"""
        if bps >= 1_000_000_000:  # Gbps
            return f"{bps / 1_000_000_000:.2f} Gbps"
        elif bps >= 1_000_000:    # Mbps
            return f"{bps / 1_000_000:.2f} Mbps"
        elif bps >= 1_000:        # Kbps
            return f"{bps / 1_000:.2f} Kbps"
        else:
            return f"{bps:.0f} bps"

# 테스트용 실행 코드
if __name__ == "__main__":
    monitor = BandwidthMonitor()
    
    try:
        monitor.start_monitoring()
        
        # 5분간 모니터링
        for i in range(30):  # 30 * 10초 = 5분
            time.sleep(10)
            results = monitor.get_formatted_results()
            print(f"\n=== 대역폭 측정 결과 ({i+1}/30) ===")
            print(f"업로드 속도: {results['upload_speed_formatted']}")
            print(f"다운로드 속도: {results['download_speed_formatted']}")
            print(f"실시간 업로드: {results['realtime_upload_mbps']} Mbps")
            print(f"실시간 다운로드: {results['realtime_download_mbps']} Mbps")
            print(f"테스트 상태: {results['test_status']}")
            print(f"마지막 테스트: {results['last_test_time']}")
    
    except KeyboardInterrupt:
        print("\n모니터링 중지 중...")
    
    finally:
        monitor.stop_monitoring()
