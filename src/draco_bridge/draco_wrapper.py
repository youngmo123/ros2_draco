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
        
        # draco_encoder 실행
        encoder_path = os.path.expanduser("~/draco/build/draco_encoder")
        cmd = [encoder_path, "-i", input_file, "-o", output_file, "-qp", "14", "-cl", "10"]
        
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
        
        # draco_decoder 실행
        decoder_path = os.path.expanduser("~/draco/build/draco_decoder")
        cmd = [decoder_path, "-i", input_file, "-o", output_file]
        
        result = subprocess.run(cmd, capture_output=True, text=True)
        print(f"[DEBUG] Draco decoder command: {' '.join(cmd)}")
        print(f"[DEBUG] Draco decoder stdout: {result.stdout}")
        print(f"[DEBUG] Draco decoder stderr: {result.stderr}")
        print(f"[DEBUG] Draco decoder return code: {result.returncode}")
        
        if result.returncode != 0:
            raise Exception(f"Draco decoding failed: {result.stderr}")
        
        # 해제된 PLY 파일 읽기
        points = []
        try:
            with open(output_file, 'r', encoding='utf-8', errors='ignore') as f:
                lines = f.readlines()
                
                # 헤더 건너뛰기
                vertex_count = 0
                header_end = 0
                for i, line in enumerate(lines):
                    if line.startswith("element vertex"):
                        vertex_count = int(line.split()[-1])
                    elif line.strip() == "end_header":
                        header_end = i + 1
                        break
                
                # 포인트 데이터 읽기
                for i in range(header_end, header_end + vertex_count):
                    if i < len(lines):
                        try:
                            coords = lines[i].strip().split()
                            if len(coords) >= 3:
                                x, y, z = float(coords[0]), float(coords[1]), float(coords[2])
                                intensity = float(coords[3]) if len(coords) > 3 else 0.0
                                points.append([x, y, z, intensity])
                        except (ValueError, IndexError):
                            # 잘못된 데이터 라인은 건너뛰기
                            continue
        except Exception as e:
            print(f"Error reading PLY file: {e}")
            return None
        
        # 임시 파일 삭제
        os.unlink(input_file)
        os.unlink(output_file)
        
        return np.array(points, dtype=np.float32)
        
    except Exception as e:
        print(f"Draco decoding error: {e}")
        return None
