# [Phase 01 실습 가이드] 개발 가상환경 구축 및 ROS2 워크스페이스 준비

* **문서 번호**: GUIDE-LOGISTICS-PHASE-01
* **관련 마일스톤**: [20260831_02_development_roadmap.md](./20260831_02_development_roadmap.md) - Phase 01
* **작성일**: 2026-08-31
* **버전**: v1.0.0
* **작업 환경**: Ubuntu 22.04 LTS (x86_64) / ROS2 Humble / Miniconda (Python 3.10) / MuJoCo 3.6.x

---

## 1. 개요 및 학습 목표 (Overview & Objectives)

본 단계는 Franka Panda 듀얼 로봇 물류 분류 시스템 개발의 첫 걸음으로, **ROS2 Humble, MuJoCo 물리 시뮬레이터, PyTorch/Gymnasium 강화학습 라이브러리가 공존하는 통합 Python 3.10 가상환경**을 구축하고, 추후 커스텀 인터페이스들이 탑재될 **`franka_logistics_msgs` 패키지 뼈대와 ROS2 워크스페이스를 검증**하는 단계입니다.

### 🎯 핵심 학습 목표
1. **ROS2와 Conda 가상환경의 공존 원리 이해**: ROS2 시스템 환경과 Conda Python 가상환경 간의 충돌 원인과 해결법 습득.
2. **Conda 기반 의존성 패키지 구축**: MuJoCo 3.6.x, Gymnasium, PyTorch, Stable-Baselines3, Open3D 등 로봇 시뮬레이션 및 AI 필수 패키지 구성.
3. **ROS2 인터페이스 패키지 뼈대 생성**: `ament_cmake` 기반의 커스텀 메시지 패키지 생성 및 빌드 파이프라인 검증.
4. **자가 진단 스크립트를 통한 환경 무결성 검증**: Python 바인딩 및 ROS2 통신 환경이 완벽히 준비되었는지 확인.

---

## 2. 이론적 배경 (Theoretical Background)

### 2.1 ROS2 Humble과 Conda 가상환경(Python 3.10)의 관계
Ubuntu 22.04 LTS의 기본 시스템 Python 버전은 **Python 3.10**입니다. ROS2 Humble 역시 시스템 Python 3.10을 기반으로 동작합니다. 

그러나 로보틱스 및 AI 프로젝트에서는 다음과 같은 이유로 **Conda 가상환경**을 사용합니다:
* **의존성 격리**: MuJoCo, PyTorch(CUDA), Stable-Baselines3, Open3D 등의 버전 충돌 방지
* **재현성(Reproducibility)**: `environment.yml`을 통해 팀원이나 다른 서버에서도 동일한 환경을 100% 재현

> [!WARNING]
> **Conda 환경 활성화 시 주의점**:
> Conda 환경을 활성화하면 시스템 경로(`PATH`, `PYTHONPATH`)의 우선순위가 변경됩니다. ROS2 Humble의 C++ 및 Python 라이브러리를 빌드할 때(`colcon build`), Conda의 Python 인터프리터와 ROS2의 빌드 도구가 꼬이지 않도록 환경 활성화 순서와 빌드 옵션을 명확히 관리해야 합니다.

### 2.2 왜 `franka_logistics_msgs`는 `ament_cmake`로 생성하는가?
ROS2에서 파이썬 노드만 만들더라도 **사용자 정의 메시지(msg), 서비스(srv), 액션(action) 패키지는 반드시 `ament_cmake` 빌드 타입으로 생성**해야 합니다.

```text
[ .msg / .srv / .action 인터페이스 파일 ]
                  │
                  ▼ (rosidl_default_generators)
         ┌────────┴────────┐
         ▼                 ▼
   [ C++ 헤더/바인딩 ]   [ Python 모듈/바인딩 ]
   (.hpp / .so)         (.py / .so)
```

ROS2의 `rosidl_default_generators`는 CMake 기반의 빌드 시스템을 통해서만 C++ 헤더 파일과 Python C-Extension 모듈을 동시에 자동 생성할 수 있기 때문입니다.

### 2.3 왜 지금은 모든 메시지를 만들지 않고 뼈대만 만드는가? (Just-In-Time)
전체 메시지를 미리 만들어두면 개발자가 각 필드와 메시지 타입의 필요성을 체감하기 어렵습니다. 따라서 본 단계에서는 **빌드 파이프라인이 정상 작동하는 패키지 껍데기만 빌드**해 두고, 이후 Phase에서 필요한 시점에 메시지를 하나씩 추가하며 학습합니다:
* **Phase 04 진입 시**: `SortItem.action` 추가 (Pick & Place 비동기 제어)
* **Phase 05 진입 시**: `BinStatus.msg`, `BinStatusArray.msg` 추가 (적재함 차지율 브로드캐스팅)
* **Phase 06 진입 시**: `SafetyStatus.msg` 추가 (안전 구역 침범 및 E-Stop 상태 전파)
* **Phase 07 진입 시**: `MesNotification.srv` 추가 (상위 MES 시스템 동기 통신)

---

## 3. 단계별 실습 진행 가이드 (Step-by-Step Implementation)

> [!NOTE]
> **경로 표기 기준**: 본 문서의 모든 명령어와 경로는 OS나 드라이브명(`Z:\`, `C:\`, `/home/user/` 등)에 구애받지 않도록 **프로젝트 루트 디렉토리(`Proj_Franka_panda_cobot_task_in_ros2_and_mujoco_simulator/`)**를 기준으로 표기합니다.

### 📋 디렉토리 레이아웃 안내
본 프로젝트의 디렉토리 구조는 다음과 같이 구성됩니다:
```text
Proj_Franka_panda_cobot_task_in_ros2_and_mujoco_simulator/
├── documents/
│   ├── development_roadmap/
│   │   ├── 20260831_02_development_roadmap.md
│   │   └── rm_phase01_environment_and_workspace_setup.md
│   └── scenario_and_prd/
├── environment.yml                  # [Step 1] 가상환경 명세서
├── ros2_ws/                         # [Step 2] ROS2 워크스페이스
│   └── src/
│       └── franka_logistics_msgs/   # [Step 3] 커스텀 인터페이스 패키지
└── test_phase01_env.py              # [Step 5] 통합 환경 검증 스크립트
```

---

### 🔹 Step 1: Miniconda 가상환경 정의 및 생성 (`environment.yml`)

프로젝트 루트 디렉토리(`Proj_Franka_panda_cobot_task_in_ros2_and_mujoco_simulator/`)에 `environment.yml` 파일을 작성합니다.

#### 1) `environment.yml` 파일 작성
```yaml
name: ros2_mujoco_panda_py3_10
channels:
  - conda-forge
  - defaults
dependencies:
  - python=3.10
  - pip
  - numpy>=1.24.0,<2.0.0
  - scipy>=1.10.0
  - pyyaml
  - pip:
      # 물리 엔진 및 시뮬레이션
      - mujoco>=3.6.0
      
      # 강화학습 및 딥러닝
      - torch>=2.0.0
      - gymnasium>=0.29.0
      - stable-baselines3>=2.2.0
      
      # 비전 및 3D 데이터 처리
      - opencv-python>=4.8.0
      - open3d>=0.17.0
      - transforms3d>=0.4.1
      
      # ROS2 빌드 및 유틸리티
      - colcon-common-extensions
      - setuptools==58.2.0
      - rich
      - matplotlib
```

> [!NOTE]
> `setuptools==58.2.0`으로 고정하는 이유: 상위 setuptools 버전에서 ROS2 Python 패키지 빌드 시 발생하는 `EasyInstallDeprecationWarning` 경고 및 호환성 문제를 방지하기 위함입니다.

#### 2) 가상환경 생성 및 활성화
터미널에서 아래 명령을 실행합니다:
```bash
# 1. Conda 가상환경 생성 (약 2~3분 소요)
conda env create -f environment.yml

# 2. 가상환경 활성화
conda activate ros2_mujoco_panda_py3_10

# 3. Python 버전 확인 (Python 3.10.x 출력 확인)
python --version
```

---

### 🔹 Step 2: ROS2 워크스페이스 구조 생성

프로젝트 디렉토리(`Proj_Franka_panda_cobot_task_in_ros2_and_mujoco_simulator/`) 내에 표준 ROS2 워크스페이스 폴더(`ros2_ws/src`)를 생성합니다.

```bash
# 프로젝트 루트 디렉토리로 이동 (본인 환경의 프로젝트 경로)
cd Proj_Franka_panda_cobot_task_in_ros2_and_mujoco_simulator

# 워크스페이스 디렉토리 생성
mkdir -p ros2_ws/src
```

---

### 🔹 Step 3: `franka_logistics_msgs` 패키지 뼈대 생성

커스텀 메시지들을 담을 `ament_cmake` 패키지를 생성하고 기본 빌드 설정을 구성합니다.

#### 1) 패키지 생성 명령어 실행
```bash
# src 디렉토리로 이동
cd Proj_Franka_panda_cobot_task_in_ros2_and_mujoco_simulator/ros2_ws/src

# ament_cmake 타입으로 패키지 생성
ros2 pkg create --build-type ament_cmake franka_logistics_msgs \
  --description "Custom ROS2 Interfaces for Dual Franka Logistics System" \
  --license Apache-2.0
```

#### 2) `package.xml` 파일 수정
`ros2_ws/src/franka_logistics_msgs/package.xml` 파일을 열어 인터페이스 생성에 필요한 필수 의존성 태그들을 추가합니다:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>franka_logistics_msgs</name>
  <version>0.1.0</version>
  <description>Custom ROS2 Interfaces for Dual Franka Logistics System</description>
  <maintainer email="tae@todo.todo">tae</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <exec_depend>rosidl_default_runtime</exec_depend>
  <exec_depend>action_msgs</exec_depend>
  <exec_depend>builtin_interfaces</exec_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

#### 3) `CMakeLists.txt` 파일 수정
`ros2_ws/src/franka_logistics_msgs/CMakeLists.txt` 파일을 열어 다음과 같이 설정합니다:

```cmake
cmake_minimum_required(VERSION 3.8)
project(franka_logistics_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 기본 의존성 탐색
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# ==============================================================================
# 인터페이스 파일 등록 (현재는 빈 뼈대, Phase 04~07에서 파일 생성 후 아래에 주석 해제)
# ==============================================================================
# rosidl_generate_interfaces(${PROJECT_NAME}
#   # Phase 05 추가 예정
#   # "msg/BinStatus.msg"
#   # "msg/BinStatusArray.msg"
#   # Phase 06 추가 예정
#   # "msg/SafetyStatus.msg"
#   # Phase 07 추가 예정
#   # "srv/MesNotification.srv"
#   # Phase 04 추가 예정
#   # "action/SortItem.action"
#   DEPENDENCIES action_msgs builtin_interfaces
# )

ament_package()
```

---

### 🔹 Step 4: 워크스페이스 초기 빌드 및 환경 로드

인터페이스 패키지가 에러 없이 빌드 시스템에 통합되는지 확인합니다.

```bash
# 워크스페이스 디렉토리로 이동
cd Proj_Franka_panda_cobot_task_in_ros2_and_mujoco_simulator/ros2_ws

# ROS2 Humble 기본 언더레이 환경 로드
source /opt/ros/humble/setup.bash

# Conda 가상환경 활성화 상태 확인
conda activate ros2_mujoco_panda_py3_10

# 패키지 빌드 실행
colcon build --packages-select franka_logistics_msgs --symlink-install

# 빌드 결과 워크스페이스 오버레이 환경 로드
source install/setup.bash
```

> **성공 출력 예시**:
> ```text
> Starting >>> franka_logistics_msgs
> Finished <<< franka_logistics_msgs [0.85s]
> 
> Summary: 1 package finished [1.02s]
> ```

---

### 🔹 Step 5: 종합 환경 자가진단 스크립트 작성 및 실행

프로젝트 루트에 `test_phase01_env.py` 스크립트를 작성하여 모든 라이브러리가 정상적으로 동작하는지 점검합니다.

#### 1) `test_phase01_env.py` 파일 작성
```python
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
```

#### 2) 자가진단 스크립트 실행
```bash
python test_phase01_env.py
```

모든 항목에 `✅` 마크와 함께 최종 축하 메시지가 출력되면 환경 구성이 완료된 것입니다.

---

## 4. 자주 발생하는 문제 및 해결법 (Troubleshooting)

### Q1. `colcon build` 실행 시 `colcon: command not found` 에러가 발생합니다.
* **원인**: Conda 가상환경 내에 colcon 도구가 없거나 PATH가 지정되지 않음.
* **해결법**:
  ```bash
  conda activate ros2_mujoco_panda_py3_10
  pip install colcon-common-extensions
  ```

### Q2. `import rclpy` 실행 시 `ModuleNotFoundError: No module named 'rclpy'` 에러가 발생합니다.
* **원인**: ROS2 Humble 언더레이 환경 변수가 로드되지 않은 상태에서 파이썬을 실행함.
* **해결법**:
  ```bash
  source /opt/ros/humble/setup.bash
  python test_phase01_env.py
  ```

### Q3. 매번 터미널을 열 때마다 source 명령어를 치기 번거롭습니다.
* **해결법**: `~/.bashrc` 하단에 편리한 alias나 자동 로드를 설정합니다:
  ```bash
  # 1. ROS2 Humble 기본 환경 로드
  echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

  # 2. 프로젝트 전용 활성화 alias 등록 (본인의 프로젝트 절대 경로로 지정)
  # 예: 프로젝트 루트(Proj_Franka_panda_cobot_task_in_ros2_and_mujoco_simulator)에서 아래 명령 실행
  echo "alias act_panda='conda activate ros2_mujoco_panda_py3_10 && source $(pwd)/ros2_ws/install/setup.bash'" >> ~/.bashrc

  source ~/.bashrc
  ```
  이후 새 터미널에서 `act_panda` 한 줄만 치면 Conda 가상환경과 ROS2 워크스페이스가 한 번에 준비됩니다.

---

## 5. Phase 01 완료 체크리스트 (Self Checklist)

다음 항목들을 모두 완료했는지 확인한 후 다음 단계(Phase 02)로 진행하세요:

- [ ] `environment.yml`을 통해 `ros2_mujoco_panda_py3_10` Conda 가상환경이 정상 생성되었는가?
- [ ] `ros2_ws/src/franka_logistics_msgs` 패키지가 생성되고 `package.xml`, `CMakeLists.txt`가 설정되었는가?
- [ ] `colcon build --packages-select franka_logistics_msgs`가 에러 없이 성공(Finished)하는가?
- [ ] `test_phase01_env.py` 실행 시 7개 진단 항목이 모두 통과(Pass)하는가?

---

**다음 단계**: [Phase 02] MuJoCo 3.6.x 3D 가상 씬 및 물류 환경 모델링 (`scene_dual_panda_logistics.xml`)
