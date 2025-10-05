#!/usr/bin/env python3

import os
import glob
import subprocess
import time

def play_all_rosbags():
    """ros2bagfile 디렉토리의 모든 rosbag을 순차적으로 재생"""
    
    # rosbag 디렉토리 경로
    rosbag_dir = '/home/youngmo/ros2bagfile'
    
    # 모든 폴더 찾기 (디렉토리만)
    all_items = glob.glob(os.path.join(rosbag_dir, '*'))
    rosbag_folders = sorted([item for item in all_items if os.path.isdir(item)])
    
    if not rosbag_folders:
        print(f"No rosbag folders found in {rosbag_dir}")
        return
    
    print(f"Found {len(rosbag_folders)} rosbag folders:")
    for folder in rosbag_folders:
        print(f"  - {os.path.basename(folder)}")
    
    # 각 rosbag 폴더를 순차적으로 재생
    for i, bag_folder in enumerate(rosbag_folders, 1):
        print(f"\n{'='*60}")
        print(f"Playing rosbag {i}/{len(rosbag_folders)}: {os.path.basename(bag_folder)}")
        print(f"{'='*60}")
        
        try:
            # ros2 bag play 명령 실행
            cmd = ['ros2', 'bag', 'play', bag_folder, '--clock', '--rate', '1.0']
            print(f"Executing: {' '.join(cmd)}")
            
            # 프로세스 실행
            process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            
            # 실시간 출력
            for line in process.stdout:
                print(line.strip())
            
            # 프로세스 완료 대기
            return_code = process.wait()
            
            if return_code == 0:
                print(f"✅ Successfully played: {os.path.basename(bag_folder)}")
            else:
                print(f"❌ Error playing: {os.path.basename(bag_folder)} (return code: {return_code})")
                
        except KeyboardInterrupt:
            print(f"\n⚠️ Interrupted while playing: {os.path.basename(bag_folder)}")
            process.terminate()
            break
        except Exception as e:
            print(f"❌ Exception while playing {os.path.basename(bag_folder)}: {e}")
        
        # 마지막 폴더가 아니면 잠시 대기
        if i < len(rosbag_folders):
            print(f"\nWaiting 3 seconds before next rosbag...")
            time.sleep(3)
    
    print(f"\n🎉 Finished playing all {len(rosbag_folders)} rosbag folders!")

if __name__ == '__main__':
    play_all_rosbags()
