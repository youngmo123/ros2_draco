#!/usr/bin/env python3

import ctypes
import os
import numpy as np
from ctypes import Structure, POINTER, c_int, c_float, c_char_p, c_void_p, c_size_t

# Draco 라이브러리 경로 설정
DRACO_LIB_PATH = os.path.expanduser("~/draco/build/libdraco.a")

class DracoPointCloud(Structure):
    _fields_ = [
        ("num_points", c_int),
        ("data", c_void_p),
    ]

class DracoEncoder(Structure):
    _fields_ = [
        ("encoder", c_void_p),
    ]

class DracoDecoder(Structure):
    _fields_ = [
        ("decoder", c_void_p),
    ]

def load_draco_library():
    """Draco 라이브러리 로드"""
    try:
        # libdraco.a는 정적 라이브러리이므로 직접 로드할 수 없음
        # 대신 draco_encoder와 draco_decoder 실행 파일을 사용
        return True
    except Exception as e:
        print(f"Failed to load Draco library: {e}")
        return False

def encode_pointcloud_with_draco(points_array):
    """
    numpy 배열을 Draco로 압축
    points_array: (N, 3) 또는 (N, 4) 형태의 numpy 배열
    """
    try:
        # 임시 파일로 포인트 클라우드 저장
        import tempfile
        import subprocess
        
        # PLY 형식으로 포인트 클라우드 저장
        with tempfile.NamedTemporaryFile(mode='w', suffix='.ply', delete=False, encoding='utf-8') as f:
            f.write("ply\n")
            f.write("format ascii 1.0\n")
            f.write(f"element vertex {len(points_array)}\n")
            f.write("property float x\n")
            f.write("property float y\n")
            f.write("property float z\n")
            if points_array.shape[1] > 3:
                f.write("property float intensity\n")
            f.write("end_header\n")
            
            for point in points_array:
                if points_array.shape[1] > 3:
                    f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f} {point[3]:.6f}\n")
                else:
                    f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n")
            
            input_file = f.name
        
        # Draco 인코더로 압축
        with tempfile.NamedTemporaryFile(suffix='.drc', delete=False) as f:
            output_file = f.name
        
        # draco_encoder 실행 (무손실 압축 시도)
        # -qp 0, -cl 0, -pos 20: 최고 품질 설정
        # -qt 10: 속성 양자화 비트 (높을수록 정밀도 높음)
        encoder_path = os.path.expanduser("~/draco/build/draco_encoder")
        cmd = [encoder_path, "-i", input_file, "-o", output_file, "-qp", "0", "-cl", "0", "-pos", "20", "-qt", "10"]
        
        result = subprocess.run(cmd, capture_output=True, text=True)
        print(f"[DEBUG] Draco encoder command: {' '.join(cmd)}")
        print(f"[DEBUG] Draco encoder stdout: {result.stdout}")
        print(f"[DEBUG] Draco encoder stderr: {result.stderr}")
        print(f"[DEBUG] Draco encoder return code: {result.returncode}")
        
        if result.returncode != 0:
            raise Exception(f"Draco encoding failed: {result.stderr}")
        
        # 압축된 파일 읽기
        with open(output_file, 'rb') as f:
            compressed_data = f.read()
        
        # 임시 파일 삭제
        os.unlink(input_file)
        os.unlink(output_file)
        
        return compressed_data
        
    except Exception as e:
        print(f"Draco encoding error: {e}")
        return None

def decode_pointcloud_with_draco(compressed_data):
    """
    Draco 압축된 데이터를 numpy 배열로 해제
    """
    try:
        import tempfile
        import subprocess
        
        # 압축된 데이터를 임시 파일에 저장
        with tempfile.NamedTemporaryFile(suffix='.drc', delete=False) as f:
            f.write(compressed_data)
            input_file = f.name
        
        # Draco 디코더로 해제
        with tempfile.NamedTemporaryFile(suffix='.ply', delete=False) as f:
            output_file = f.name
        
        # draco_decoder 실행 (텍스트 PLY 출력으로 변경)
        decoder_path = os.path.expanduser("~/draco/build/draco_decoder")
        cmd = [decoder_path, "-i", input_file, "-o", output_file]
        
        result = subprocess.run(cmd, capture_output=True, text=True)
        print(f"[DEBUG] Draco decoder command: {' '.join(cmd)}")
        print(f"[DEBUG] Draco decoder stdout: {result.stdout}")
        print(f"[DEBUG] Draco decoder stderr: {result.stderr}")
        print(f"[DEBUG] Draco decoder return code: {result.returncode}")
        
        if result.returncode != 0:
            raise Exception(f"Draco decoding failed: {result.stderr}")
        
        # 해제된 PLY 파일 읽기 - 완전히 새로 작성
        points = []
        try:
            print(f"[DEBUG] Reading PLY file: {output_file}")
            
            # 파일 크기 확인
            file_size = os.path.getsize(output_file)
            print(f"[DEBUG] PLY file size: {file_size} bytes")
            
            if file_size == 0:
                print(f"[DEBUG] ERROR: PLY file is empty!")
                return None
            
            # 파일 내용을 한 번에 읽기
            with open(output_file, 'rb') as f:
                file_data = f.read()
            
            # 텍스트인지 바이너리인지 판단
            is_binary = b'format binary' in file_data[:200]
            print(f"[DEBUG] PLY format: {'binary' if is_binary else 'ASCII'}")
            
            if is_binary:
                # 바이너리 PLY 처리
                points = _parse_binary_ply(file_data)
            else:
                # ASCII PLY 처리
                points = _parse_ascii_ply(file_data.decode('utf-8', errors='ignore'))
            
            if points is None:
                print(f"[DEBUG] Failed to parse PLY file")
                return None
                
            print(f"[DEBUG] Successfully parsed {len(points)} points from PLY")
            if len(points) > 0:
                print(f"[DEBUG] PLY data sample (first 3 points): {points[:3]}")
                print(f"[DEBUG] PLY data stats - min: {[min(p[i] for p in points) for i in range(4)]}, max: {[max(p[i] for p in points) for i in range(4)]}")
                
        except Exception as e:
            print(f"[DEBUG] Error reading PLY file: {e}")
            return None
        
        # 임시 파일 삭제
        os.unlink(input_file)
        os.unlink(output_file)
        
        return np.array(points, dtype=np.float32)
        
    except Exception as e:
        print(f"Draco decoding error: {e}")
        return None

def _parse_ascii_ply(content):
    """ASCII PLY 파일 파싱"""
    lines = content.split('\n')
    print(f"[DEBUG] ASCII PLY has {len(lines)} lines")
    
    # 헤더 분석
    vertex_count = 0
    header_end = 0
    has_intensity = False
    
    for i, line in enumerate(lines):
        line = line.strip()
        if line.startswith("element vertex"):
            vertex_count = int(line.split()[-1])
            print(f"[DEBUG] Found {vertex_count} vertices")
        elif line.startswith("property") and "intensity" in line:
            has_intensity = True
            print(f"[DEBUG] Intensity property detected")
        elif line == "end_header":
            header_end = i + 1
            break
    
    print(f"[DEBUG] Header ends at line {header_end}")
    print(f"[DEBUG] Has intensity: {has_intensity}")
    
    # 포인트 데이터 읽기
    points = []
    points_read = 0
    for i in range(header_end, min(header_end + vertex_count, len(lines))):
        line = lines[i].strip()
        if not line:  # 빈 줄 건너뛰기
            continue
            
        try:
            coords = line.split()
            if len(coords) >= 3:
                x, y, z = float(coords[0]), float(coords[1]), float(coords[2])
                intensity = float(coords[3]) if len(coords) > 3 and has_intensity else 0.0
                
                # NaN이나 무한대 값 체크
                if not (np.isnan(x) or np.isnan(y) or np.isnan(z) or 
                       np.isinf(x) or np.isinf(y) or np.isinf(z)):
                    points.append([x, y, z, intensity])
                    points_read += 1
                else:
                    print(f"[DEBUG] Skipping invalid point {i}: x={x}, y={y}, z={z}")
            else:
                print(f"[DEBUG] Insufficient coordinates in line {i}: {len(coords)}")
        except (ValueError, IndexError) as e:
            print(f"[DEBUG] Error parsing line {i}: {e}")
            continue
    
    print(f"[DEBUG] Successfully read {points_read} points from ASCII PLY")
    return points

def _parse_binary_ply(file_data):
    """바이너리 PLY 파일 파싱"""
    import struct
    import io
    
    # 바이트 스트림으로 처리
    stream = io.BytesIO(file_data)
    
    # 헤더 읽기
    header_lines = []
    while True:
        line = stream.readline().decode('utf-8', errors='ignore')
        header_lines.append(line)
        if line.strip() == 'end_header':
            break
    
    # vertex 개수 찾기
    vertex_count = 0
    for line in header_lines:
        if line.startswith("element vertex"):
            vertex_count = int(line.split()[-1])
            break
    
    print(f"[DEBUG] Found {vertex_count} vertices in binary PLY")
    
    # 현재 위치가 헤더 끝
    current_pos = stream.tell()
    print(f"[DEBUG] Header ends at byte {current_pos}")
    
    # 바이너리 데이터 읽기 (x, y, z, intensity 각 4바이트 float)
    points = []
    points_read = 0
    
    for i in range(vertex_count):
        try:
            # 4개의 float32 읽기 (16바이트)
            data = stream.read(16)
            if len(data) == 16:
                x, y, z, intensity = struct.unpack('<ffff', data)
                # NaN이나 무한대 값 체크
                if not (np.isnan(x) or np.isnan(y) or np.isnan(z) or 
                       np.isinf(x) or np.isinf(y) or np.isinf(z)):
                    points.append([x, y, z, intensity])
                    points_read += 1
                else:
                    print(f"[DEBUG] Skipping invalid point {i}: x={x}, y={y}, z={z}")
            else:
                print(f"[DEBUG] Insufficient data for point {i}: {len(data)} bytes at position {stream.tell()}")
                # 데이터가 부족한 경우 중단
                break
        except Exception as e:
            print(f"[DEBUG] Error reading point {i}: {e}")
            break
    
    print(f"[DEBUG] Successfully read {points_read} points from binary PLY")
    return points
