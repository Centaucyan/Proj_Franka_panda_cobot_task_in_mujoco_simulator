#!/usr/bin/env python3
"""
Phase 01 환경 자가진단 스크립트
ROS2, MuJoCo, PyTorch, Gymnasium, OpenCV, Open3D 임포트 및 기본 작동 테스트
"""

import sys

def check_environment():
    print("=" * 60)
    print("🚀 [Phase 01] 개발 환경 및 라이브러리 무결성 검증 시작")
    print("=" * 60)
    
    # 1. Python 버전 확인
    print(f"[1/7] Python 버전: {sys.version.split()[0]} (경로: {sys.executable})")
    assert sys.version_info >= (3, 10), "Python 3.10 이상이 필요합니다."
    
    # 2. MuJoCo 물리 엔진 확인
    try:
        import mujoco
        print(f"[2/7] ✅ MuJoCo 정상 로드 - 버전: {mujoco.__version__}")
    except ImportError as e:
        print(f"[2/7] ❌ MuJoCo 로드 실패: {e}")
        return False

    # 3. PyTorch 및 디바이스 확인
    try:
        import torch
        device = "CUDA (GPU)" if torch.cuda.is_available() else "CPU"
        print(f"[3/7] ✅ PyTorch 정상 로드 - 버전: {torch.__version__} (연산 디바이스: {device})")
    except ImportError as e:
        print(f"[3/7] ❌ PyTorch 로드 실패: {e}")
        return False

    # 4. Gymnasium 및 RL 라이브러리 확인
    try:
        import gymnasium as gym
        import stable_baselines3 as sb3
        print(f"[4/7] ✅ Gymnasium ({gym.__version__}) & Stable-Baselines3 ({sb3.__version__}) 정상 로드")
    except ImportError as e:
        print(f"[4/7] ❌ RL 라이브러리 로드 실패: {e}")
        return False

    # 5. 비전 및 3D 데이터 라이브러리 확인
    try:
        import cv2
        import open3d as o3d
        import transforms3d
        print(f"[5/7] ✅ OpenCV ({cv2.__version__}) & Open3D ({o3d.__version__}) 정상 로드")
    except ImportError as e:
        print(f"[5/7] ❌ 비전 라이브러리 로드 실패: {e}")
        return False

    # 6. ROS2 rclpy 확인
    try:
        import rclpy
        rclpy.init()
        node = rclpy.create_node("phase01_verification_node")
        node.destroy_node()
        rclpy.shutdown()
        print(f"[6/7] ✅ ROS2 rclpy 노드 생성 및 통신 정상 동작")
    except Exception as e:
        print(f"[6/7] ❌ ROS2 rclpy 검증 실패: {e}")
        print("      👉 해결법: 'source /opt/ros/humble/setup.bash'를 먼저 실행했는지 확인하세요.")
        return False

    # 7. 워크스페이스 패키지 인식 확인
    import subprocess
    result = subprocess.run(["ros2", "pkg", "list"], stdout=subprocess.PIPE, text=True)
    if "franka_logistics_msgs" in result.stdout:
        print(f"[7/7] ✅ ROS2 워크스페이스 'franka_logistics_msgs' 패키지 인식 성공")
    else:
        print(f"[7/7] ⚠️ 'franka_logistics_msgs' 패키지가 ros2 pkg list에 없습니다.")
        print("      👉 해결법: 'source ros2_ws/install/setup.bash'를 실행하세요.")
        return False

    print("=" * 60)
    print("🎉 [축하합니다!] Phase 01의 모든 개발 환경이 완벽하게 준비되었습니다.")
    print("=" * 60)
    return True

if __name__ == "__main__":
    success = check_environment()
    sys.exit(0 if success else 1)
