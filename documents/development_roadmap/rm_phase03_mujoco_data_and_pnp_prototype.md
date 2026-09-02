# [Phase 03 실습 가이드] MuJoCo Python API 데이터 탐색 & 단계별 P&P 프로토타이핑

* **문서 번호**: GUIDE-LOGISTICS-PHASE-03
* **관련 마일스톤**: [20260902_01_development_roadmap.md](./20260902_01_development_roadmap.md) - Phase 03
* **작성일**: 2026-09-02
* **버전**: v1.1.0
* **작업 환경**: Ubuntu 22.04 LTS (x86_64) / Conda (`ros2_mujoco_panda_py3_10`, Python 3.10) / MuJoCo 3.6.x

---

## 1. 개요 및 학습 목표 (Overview & Objectives)

본 단계는 ROS2 노드로 패키징하기에 앞서, **MuJoCo 3.6.x 물리 엔진이 내부적으로 생성하고 요구하는 날것(Raw) 데이터의 구조를 완전히 이해**하고, 순수 Python 기반으로 **로봇 팔이 물체를 집어서 적재함에 넣는 3단계 점진적 Pick & Place(P&P) 동작을 먼저 성공**시키는 단계입니다.

블랙박스 형태의 추상화된 ROS2 노드를 만들기 전에, 물리 엔진의 입출력 데이터(`qpos`, `ctrl`, RGB/Depth 배열, LiDAR 레이캐스트)를 개발자가 직접 눈으로 확인하고 만져봄으로써 향후 ROS2 Sim Bridge(Phase 04)와 모션 제어(Phase 05) 구현의 탄탄한 토대를 마련합니다.

```
┌────────────────────────────────────────────────────────────────────────────────┐
│                           [ Phase 03 점진적 학습 및 검증 흐름 ]                │
│                                                                                │
│  [1. Raw 데이터 I/O 분석] ──► [2. DLS 역기구학/파지 튜닝] ──► [3. 3단계 P&P 검증]   │
│   • qpos/qvel (관절각/속도)      • 목표 (X,Y,Z) -> q 도출        • Step 1: R1 단독 P&P│
│   • ctrl (액추에이터 제어)       • 그리퍼 마찰력/접촉 안정화     • Step 2: R2 단독 P&P│
│   • RGB/Depth (비전 배열)       • 부드러운 S-Curve 보간          • Step 3: 듀얼 동시구동│
│   • mj_ray (LiDAR 광선)                                                        │
└────────────────────────────────────────────────────────────────────────────────┘
```

### 🎯 핵심 학습 목표
1. **MuJoCo C-Binding 핵심 데이터 아키텍처 습득**: `mjModel`(정적 제원)과 `mjData`(동적 물리 상태), `qpos`, `qvel`, `ctrl`, `site_xpos`의 메모리 구조와 인덱싱 원리 체득.
2. **수치적 역기구학(DLS Inverse Kinematics) 원리 구현**: 자코비안 행렬($J$)과 특이점 감쇠 계수($\lambda$), 관절 리밋 클램핑을 이용해 목표 3D 위치를 7-자유도 관절 각도로 실시간 역산하는 알고리즘 이해.
3. **부드러운 웨이포인트 궤적 보간(S-Curve Interpolation)**: 급격한 스텝 점프 없이 관절이 부드럽게 목표 지점으로 이동하도록 하는 모션 프로파일 생성.
4. **그리퍼 접촉 역학(Contact Dynamics) 튜닝**: 파지 시 물품 미끄러짐 및 튕겨나감(Explosion) 현상을 방지하는 마찰 계수와 서보 제어값(`ctrl=255/0`) 검증.
5. **3단계 점진적 P&P 시뮬레이션 성공**: **[로봇 1 단독] $\rightarrow$ [로봇 2 단독] $\rightarrow$ [듀얼 로봇 동시 구동]**으로 이어지는 단계별 모션 시퀀스를 3D 뷰어로 직접 관찰 및 검증.

---

## 2. 이론적 배경 (Theoretical Background)

### 2.1 MuJoCo Python C-Binding 데이터 아키텍처

MuJoCo는 성능을 극대화하기 위해 C언어 구조체 기반으로 설계되어 있으며, Python 바인딩은 이를 **NumPy 다차원 배열 포인터**로 직접 노출합니다.

```text
┌─────────────────────────────────────────────────────────────────────────────┐
│                           mjModel (정적 모델 제원)                           │
│  • nq   : 일반화 좌표 총 개수 (자유도 및 쿼터니언 포함, 본 프로젝트: 67)       │
│  • nv   : 속도 벡터 총 개수 (속도 자유도, 본 프로젝트: 60)                     │
│  • nu   : 제어 입력 액추에이터 총 개수 (본 프로젝트: 16)                        │
│  • opt  : 타임스텝(timestep=0.001s), 중력(gravity=0 0 -9.81)                  │
└──────────────────────────────────────┬──────────────────────────────────────┘
                                       │ mj_step(model, data) 1kHz 연산
┌──────────────────────────────────────▼──────────────────────────────────────┐
│                           mjData (동적 물리 상태)                            │
│  • data.qpos : 현재 관절 위치 및 물체 위치/쿼터니언 배열 [67 float64]          │
│  • data.qvel : 현재 관절 각속도 및 물체 선/각속도 배열 [60 float64]           │
│  • data.ctrl : 액추에이터 목표 제어 입력 배열 [16 float64]                      │
│  • data.site_xpos : 씬 내 정의된 모든 사이트의 글로벌 3D 좌표 [nsite x 3]      │
│  • data.contact   : 현재 활성화된 접촉점 정보 구조체 배열 [ncon개]              │
└─────────────────────────────────────────────────────────────────────────────┘
```

> [!IMPORTANT]
> **`mj_forward(model, data)` 호출의 중요성**:
> `mujoco.mj_resetDataKeyframe()`으로 초기화한 직후에는 사이트 좌표(`site_xpos`)와 바디 위치(`xpos`)가 갱신되지 않아 `(0, 0, 0)`으로 읽힐 수 있습니다. 따라서 위치를 읽거나 IK를 풀기 전에 반드시 `mujoco.mj_forward(model, data)`를 먼저 호출하여 기구학 트리를 계산해야 합니다.

---

### 2.2 감쇠 최소자승 역기구학(DLS IK)과 관절 한계 클램핑

#### 1) 감쇠 최소자승법 (Damped Least Squares, DLS)
목표 위치 $x_{\text{target}}$과 엔드이펙터 사이트 위치 $x_{\text{current}}$ 사이의 오차 벡터를 $e = x_{\text{target}} - x_{\text{current}}$라 할 때, 감쇠 계수 $\lambda > 0$를 적용한 DLS 업데이트 수식은 다음과 같습니다:
$$\Delta q = J^T \left( J J^T + \lambda^2 I \right)^{-1} e$$

* **독립된 `ik_data` 인스턴스 사용**: 실시간 물리 연산 중인 `data.qpos`를 직접 오염시키지 않도록, IK 연산 전용 가상 데이터 객체를 복사하여 안전하게 계산합니다.
* **관절 한계(Joint Limits) 준수**: 계산된 각 관절 각도 $q_i$가 XML 모델에 정의된 가동 범위 $[q_{\min}, q_{\max}]$를 벗어나지 않도록 클램핑(`np.clip`)하여 로봇 팔꿈치가 바닥 쪽으로 뒤틀리는 현상을 원천 차단합니다.

#### 2) 부드러운 웨이포인트 보간 (S-Curve Interpolation)
스텝마다 목표 관절각을 계단식으로 급격히 변경하면 모터에 과도한 충격 토크가 가해집니다. 이를 방지하기 위해 코사인 기반 S-Curve 가감속 프로파일을 적용합니다:
$$s(t) = \frac{1}{2} \left( 1 - \cos\left(\pi \cdot \frac{t}{T_{\text{duration}}}\right) \right), \quad q(t) = (1 - s(t)) q_{\text{start}} + s(t) q_{\text{target}}$$

---

## 3. 단계별 실습 진행 가이드 (Step-by-Step Implementation)

### 📋 Phase 03 생성 파일 레이아웃 안내
```text
Proj_Franka_panda_cobot_task_in_ros2_and_mujoco_simulator/
├── documents/
│   └── development_roadmap/
│       ├── 20260902_01_development_roadmap.md
│       └── rm_phase03_mujoco_data_and_pnp_prototype.md # 본 실습 가이드
└── test/
    ├── phase01_test_env.py
    ├── phase02_test_scene.py
    ├── phase02_view_scene.py
    ├── phase03_explore_data.py                         # [Step 1] 원시 데이터 심층 분석
    └── phase03_pnp_prototype.py                        # [Step 2] 3단계 점진적 P&P 프로토타입
```

---

### 🔹 Step 1: MuJoCo 원시 데이터 I/O 심층 탐색 스크립트 작성 (`test/phase03_explore_data.py`)

`test/phase03_explore_data.py` 파일을 생성하고 아래 코드를 작성합니다.

#### 1) `test/phase03_explore_data.py` 전체 소스 코드
```python
#!/usr/bin/env python3
"""
Phase 03 MuJoCo 원시(Raw) 데이터 I/O 심층 탐색 스크립트
==============================================================================
본 스크립트는 ROS2 노드로 패키징하기 전에, MuJoCo 물리 엔진이 내부적으로
생성하고 요구하는 핵심 데이터 구조(관절, 제어, 카메라 렌더링, LiDAR 광선, 접촉력)를
직접 추출하여 터미널 및 시각화 데이터로 출력하고 확인하는 학습용 스크립트입니다.
==============================================================================
"""

import os
import sys
import numpy as np

def explore_mujoco_data():
    print("=" * 75)
    print("🔍 [Phase 03] MuJoCo 3.6.x 원시(Raw) 데이터 I/O 심층 분석 및 탐색")
    print("=" * 75)

    # 1. 씬 파일 로드
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.abspath(os.path.join(script_dir, ".."))
    xml_path = os.path.join(
        project_root,
        "ros2_ws", "src", "franka_logistics_description", "mjcf", "scene_dual_panda_logistics.xml"
    )

    if not os.path.exists(xml_path):
        print(f"❌ 오류: 씬 파일이 존재하지 않습니다: {xml_path}")
        return False

    try:
        import mujoco
        model = mujoco.MjModel.from_xml_path(xml_path)
        data = mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model, data, 0)
        mujoco.mj_forward(model, data)  # 기하 트리 갱신
        print(f"✅ MuJoCo MjModel & MjData 로드 완료 (모델명: {model.names.decode('utf-8').split(chr(0))[0]})")
    except ImportError:
        print("❌ 오류: mujoco 모듈을 찾을 수 없습니다. 'conda activate ros2_mujoco_panda_py3_10'을 확인하세요.")
        return False
    except Exception as e:
        print(f"❌ 오류: 모델 컴파일 실패: {e}")
        return False

    # --------------------------------------------------------------------------
    # 1. 관절 상태 데이터 (Joint States: data.qpos, data.qvel)
    # --------------------------------------------------------------------------
    print("\n" + "-" * 75)
    print("📊 [1] 로봇 관절 상태 데이터 (qpos / qvel) 구조 분석")
    print("-" * 75)
    print(f"• 일반화 좌표 총 차원 (model.nq): {model.nq}개 float")
    print(f"• 속도 벡터 총 차원   (model.nv): {model.nv}개 float")
    print(f"• 관절 총 개수        (model.njnt): {model.njnt}개")

    print("\n[Robot 1 (좌측) 관절 상태 - qpos[0:9]]")
    r1_joint_names = [f"r1_joint{i+1}" for i in range(7)] + ["r1_finger_joint1", "r1_finger_joint2"]
    for i, name in enumerate(r1_joint_names):
        print(f"  • {name:<18}: qpos[{i:02d}] = {data.qpos[i]:+8.4f} rad/m, qvel[{i:02d}] = {data.qvel[i]:+8.4f} rad/s")

    print("\n[Robot 2 (우측) 관절 상태 - qpos[9:18]]")
    r2_joint_names = [f"r2_joint{i+1}" for i in range(7)] + ["r2_finger_joint1", "r2_finger_joint2"]
    for i, name in enumerate(r2_joint_names):
        idx = i + 9
        print(f"  • {name:<18}: qpos[{idx:02d}] = {data.qpos[idx]:+8.4f} rad/m, qvel[{idx:02d}] = {data.qvel[idx]:+8.4f} rad/s")

    print("\n💡 ROS2 매핑: data.qpos[0:9] -> /robot1/joint_states (sensor_msgs/JointState.position)")

    # --------------------------------------------------------------------------
    # 2. 액추에이터 제어 데이터 (Actuator Controls: data.ctrl)
    # --------------------------------------------------------------------------
    print("\n" + "-" * 75)
    print("⚡ [2] 액추에이터 제어 명령 데이터 (data.ctrl) 구조 분석")
    print("-" * 75)
    print(f"• 전체 제어 입력 차원 (model.nu): {model.nu}개 (기대치: 16개)")
    
    print("\n[Robot 1 액추에이터 제어 범위 (model.actuator_ctrlrange)]")
    for i in range(8):
        act_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        ctrl_min, ctrl_max = model.actuator_ctrlrange[i]
        curr_ctrl = data.ctrl[i]
        unit = "rad" if i < 7 else "grip (0~255)"
        print(f"  • [{i:02d}] {act_name:<16}: 현재 입력={curr_ctrl:6.2f} | 제어범위=[{ctrl_min:6.2f}, {ctrl_max:6.2f}] ({unit})")

    print("\n[Robot 2 액추에이터 제어 범위 (model.actuator_ctrlrange)]")
    for i in range(8, 16):
        act_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        ctrl_min, ctrl_max = model.actuator_ctrlrange[i]
        curr_ctrl = data.ctrl[i]
        unit = "rad" if i < 15 else "grip (0~255)"
        print(f"  • [{i:02d}] {act_name:<16}: 현재 입력={curr_ctrl:6.2f} | 제어범위=[{ctrl_min:6.2f}, {ctrl_max:6.2f}] ({unit})")

    print("\n💡 ROS2 매핑: 상위 제어 노드의 목표 지령 -> data.ctrl[0:8] / data.ctrl[8:16] 에 1kHz로 인가")

    # --------------------------------------------------------------------------
    # 3. 3D 기하체 및 주요 사이트 좌표 (Sites & Bodies)
    # --------------------------------------------------------------------------
    print("\n" + "-" * 75)
    print("📍 [3] 3D 작업 공간 주요 사이트(Site) 및 물품 위치 (World Frame)")
    print("-" * 75)
    
    sites_to_inspect = [
        "r1_ee_site", "r2_ee_site",
        "site_bin_A_Red", "site_bin_B_Blue", "site_bin_C_Green",
        "site_bin_A_Green", "site_bin_B_Red", "site_bin_C_Blue",
        "site_conveyor_r1", "site_conveyor_r2",
        "lidar_top_frame", "lidar_bottom_frame"
    ]
    
    for site_name in sites_to_inspect:
        sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        if sid >= 0:
            pos = data.site_xpos[sid]
            print(f"  • 사이트 [{site_name:<20}]: (X={pos[0]:+6.3f}, Y={pos[1]:+6.3f}, Z={pos[2]:+6.3f}) m")

    items_to_inspect = ["item_A_Red", "item_B_Blue", "item_C_Green", "item_A_Green", "item_B_Red", "item_C_Blue", "item_unclassified_1"]
    print("\n[물류 아이템 7종 초기 위치]")
    for item_name in items_to_inspect:
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, item_name)
        if bid >= 0:
            pos = data.xpos[bid]
            print(f"  • 물품 [{item_name:<20}]: (X={pos[0]:+6.3f}, Y={pos[1]:+6.3f}, Z={pos[2]:+6.3f}) m")

    # --------------------------------------------------------------------------
    # 4. 가상 Wrist 비전 센서 오프스크린 렌더링 (RGB / Depth)
    # --------------------------------------------------------------------------
    print("\n" + "-" * 75)
    print("📷 [4] 가상 Wrist 카메라 오프스크린 렌더링 데이터 추출")
    print("-" * 75)
    try:
        renderer = mujoco.Renderer(model, height=480, width=640)
        
        renderer.update_scene(data, camera="r1_wrist_camera")
        rgb_img = renderer.render()
        
        renderer.enable_depth_rendering()
        renderer.update_scene(data, camera="r1_wrist_camera")
        depth_img = renderer.render()
        renderer.disable_depth_rendering()
        
        print(f"  • RGB 이미지 데이터 형태 : {rgb_img.shape} (dtype: {rgb_img.dtype}) -> 값 범위: [{rgb_img.min()}, {rgb_img.max()}]")
        print(f"  • Depth 맵 데이터 형태   : {depth_img.shape} (dtype: {depth_img.dtype}) -> 거리 범위: [{depth_img.min():.3f}m, {depth_img.max():.3f}m]")
        print("  ✅ OpenGL 오프스크린 렌더러 정상 작동 (640x480 RealSense 해상도)")
        print("  💡 ROS2 매핑: rgb_img -> sensor_msgs/Image (rgb8), depth_img -> sensor_msgs/Image (32FC1)")
    except Exception as e:
        print(f"  ⚠️ 오프스크린 렌더러 테스트 중 알림: {e}")

    # --------------------------------------------------------------------------
    # 5. 2D 안전 LiDAR 레이캐스팅 (Ray-casting) 데이터 추출
    # --------------------------------------------------------------------------
    print("\n" + "-" * 75)
    print("📡 [5] 2D 안전 LiDAR 레이캐스팅 (mj_ray) 거리 계측 데이터 추출")
    print("-" * 75)
    
    top_lidar_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "lidar_top_frame")
    lidar_pos = data.site_xpos[top_lidar_id]
    
    angles_deg = [0, 45, 90, 135, 180, 225, 270, 315]
    print(f"• 상단 LiDAR 위치: (X={lidar_pos[0]:.2f}, Y={lidar_pos[1]:.2f}, Z={lidar_pos[2]:.2f}) m")
    print("• 대표 8방향 광선 투사 계측 결과:")
    
    geom_id_buf = np.zeros(1, dtype=np.int32)
    for deg in angles_deg:
        rad = np.radians(deg)
        ray_dir = np.array([np.cos(rad), np.sin(rad), 0.0], dtype=np.float64)
        dist = mujoco.mj_ray(model, data, lidar_pos, ray_dir, None, 1, -1, geom_id_buf)
        hit_geom = geom_id_buf[0]
        geom_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, hit_geom) if hit_geom >= 0 else "None (Out of range)"
        print(f"  - 각도 {deg:3d}° ({ray_dir[0]:+5.2f}, {ray_dir[1]:+5.2f}): 거리 = {dist:5.3f} m (충돌 지오메트리: {geom_name})")

    print("💡 ROS2 매핑: 레이캐스팅 거리 배열 -> sensor_msgs/LaserScan.ranges (20Hz 발행)")

    # --------------------------------------------------------------------------
    # 6. 접촉 역학 데이터 (Contact Dynamics: data.ncon, data.contact)
    # --------------------------------------------------------------------------
    print("\n" + "-" * 75)
    print("🤝 [6] 물리 엔진 접촉 역학 (Contacts) 상태 분석")
    print("-" * 75)
    
    for _ in range(50):
        mujoco.mj_step(model, data)
        
    print(f"• 현재 활성화된 접촉점 수 (data.ncon): {data.ncon}개")
    contact_count = min(data.ncon, 5)
    print(f"• 대표 접촉 쌍 (상위 {contact_count}개):")
    for i in range(contact_count):
        con = data.contact[i]
        g1_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, con.geom1) or f"geom_{con.geom1}"
        g2_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, con.geom2) or f"geom_{con.geom2}"
        print(f"  - Contact [{i}]: [{g1_name}] <---> [{g2_name}] | 거리={con.dist:+7.4f}m")

    print("=" * 75)
    print("🎉 [탐색 완료] MuJoCo 3.6.x의 모든 원시 데이터가 성공적으로 분석되었습니다.")
    print("=" * 75)
    return True

if __name__ == "__main__":
    success = explore_mujoco_data()
    sys.exit(0 if success else 1)
```

#### 2) 스크립트 실행 명령
```bash
# 1. Conda 가상환경 활성화
conda activate ros2_mujoco_panda_py3_10

# 2. 탐색 스크립트 실행
python test/phase03_explore_data.py
```

---

### 🔹 Step 2: 3단계 점진적 P&P 프로토타입 스크립트 작성 (`test/phase03_pnp_prototype.py`)

`test/phase03_pnp_prototype.py` 파일을 생성하고 아래 코드를 작성합니다.

#### 1) `test/phase03_pnp_prototype.py` 전체 소스 코드
```python
#!/usr/bin/env python3
"""
Phase 03 점진적 Pick & Place 프로토타입 검증 스크립트
==============================================================================
[3단계 점진적 테스트 모드]
1. mode='r1'  : Robot 1 (좌측) 단독 P&P (item_A_Red -> bin_A_Red, Robot 2는 Home 대기)
2. mode='r2'  : Robot 2 (우측) 단독 P&P (item_C_Blue -> bin_C_Blue, Robot 1은 Home 대기)
3. mode='dual': Robot 1 & Robot 2 동시 구동 P&P (다중 로봇 액추에이터 동시 제어 검증)

[핵심 알고리즘]
- DLS(Damped Least Squares) 수치적 역기구학(IK) + 관절 리밋 클램핑
- 부드러운 S-Curve 코사인 궤적 보간기
- 8단계 모션 시퀀서: Approach -> Descend -> Grasp -> Lift -> Transfer -> Place -> Release -> Home
==============================================================================
"""

import os
import sys
import time
import argparse
import numpy as np

def solve_dls_ik(model, site_name, target_pos, current_full_qpos, arm_joint_indices, arm_qpos_indices):
    """
    독립된 MjData 인스턴스를 사용하여 실시간 시뮬레이션 데이터를 오염시키지 않고
    관절 리밋을 완벽히 준수하는 Damped Least Squares (DLS) IK 솔버
    """
    import mujoco
    ik_data = mujoco.MjData(model)
    ik_data.qpos[:] = current_full_qpos.copy()
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
    
    q_arm = current_full_qpos[arm_qpos_indices].copy()
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    
    damping = 0.05       # 감쇠 계수 (lambda)
    step_size = 0.4      # 반복 수렴 학습률
    tolerance = 0.002    # 수렴 오차 허용치 (2mm)
    
    for _ in range(80):
        mujoco.mj_forward(model, ik_data)
        current_pos = ik_data.site_xpos[site_id]
        error = target_pos - current_pos
        
        if np.linalg.norm(error) < tolerance:
            break
            
        mujoco.mj_jacSite(model, ik_data, jacp, jacr, site_id)
        J = jacp[:, arm_joint_indices]  # (3, 7)
        
        # DLS 수식: J^T * (J * J^T + lambda^2 * I)^(-1) * error
        A = J @ J.T + (damping ** 2) * np.eye(3)
        dq = J.T @ np.linalg.solve(A, error)
        
        q_arm += step_size * dq
        
        # 관절 각도 리밋 클램핑 (Joint Limits Clamping)
        for i, j_idx in enumerate(arm_joint_indices):
            jnt_id = model.dof_jntid[j_idx]
            if model.jnt_limited[jnt_id]:
                q_min, q_max = model.jnt_range[jnt_id]
                q_arm[i] = np.clip(q_arm[i], q_min, q_max)
                
        ik_data.qpos[arm_qpos_indices] = q_arm
        
    return q_arm

class RobotController:
    """단일 로봇 P&P 상태 머신 및 S-Curve 액추에이터 제어기"""
    def __init__(self, prefix, model, data):
        import mujoco
        self.prefix = prefix
        self.model = model
        self.data = data
        
        if prefix == "r1":
            self.arm_qpos_idx = list(range(0, 7))
            self.arm_nv_idx = list(range(0, 7))
            self.ctrl_idx = list(range(0, 8))
            self.ee_site = "r1_ee_site"
            self.target_item = "item_A_Red"
            self.target_bin_site = "site_bin_A_Red"
            self.home_qpos = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
        else: # r2
            self.arm_qpos_idx = list(range(9, 16))
            self.arm_nv_idx = list(range(9, 16))
            self.ctrl_idx = list(range(8, 16))
            self.ee_site = "r2_ee_site"
            self.target_item = "item_C_Blue"
            self.target_bin_site = "site_bin_C_Blue"
            self.home_qpos = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
            
        self.state = "INIT"
        self.state_timer = 0.0
        self.state_duration = 1.0
        
        self.start_qpos = self.home_qpos.copy()
        self.target_qpos = self.home_qpos.copy()
        self.gripper_ctrl = 0.0  # 0: Open, 255: Close
        
        # 물품 및 적재함 3D 좌표 탐색 (반드시 mj_forward 이후 좌표 읽기)
        item_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, self.target_item)
        bin_sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, self.target_bin_site)
        
        self.item_pos = data.xpos[item_bid].copy()
        self.bin_pos = data.site_xpos[bin_sid].copy()

    def set_motion(self, next_state, target_3d_pos, duration, gripper=None):
        self.state = next_state
        self.state_timer = 0.0
        self.state_duration = duration
        self.start_qpos = self.target_qpos.copy()
        
        if target_3d_pos is not None:
            self.target_qpos = solve_dls_ik(
                self.model, self.ee_site, target_3d_pos, self.data.qpos, self.arm_nv_idx, self.arm_qpos_idx
            )
        else:
            self.target_qpos = self.home_qpos.copy()
            
        if gripper is not None:
            self.gripper_ctrl = gripper

    def update(self, dt):
        self.state_timer += dt
        
        if self.state == "INIT":
            if self.state_timer > 1.0:
                approach_pos = self.item_pos + np.array([0, 0, 0.12])
                print(f"[{self.prefix.upper()}] 1단계: 물품 상단 접근 (Approach)")
                self.set_motion("APPROACH", approach_pos, duration=2.0)

        elif self.state == "APPROACH":
            if self.state_timer >= self.state_duration:
                grasp_pos = self.item_pos + np.array([0, 0, 0.015])
                print(f"[{self.prefix.upper()}] 2단계: 물품 파지 위치 하강 (Descend)")
                self.set_motion("DESCEND", grasp_pos, duration=1.2)

        elif self.state == "DESCEND":
            if self.state_timer >= self.state_duration:
                print(f"[{self.prefix.upper()}] 3단계: 그리퍼 파지 (Grasp Close)")
                self.state = "GRASP"
                self.state_timer = 0.0
                self.state_duration = 0.8
                self.gripper_ctrl = 255.0  # 닫기

        elif self.state == "GRASP":
            if self.state_timer >= self.state_duration:
                lift_pos = self.item_pos + np.array([0, 0, 0.18])
                print(f"[{self.prefix.upper()}] 4단계: 물품 들어올리기 (Lift)")
                self.set_motion("LIFT", lift_pos, duration=1.5)

        elif self.state == "LIFT":
            if self.state_timer >= self.state_duration:
                transfer_pos = self.bin_pos + np.array([0, 0, 0.15])
                print(f"[{self.prefix.upper()}] 5단계: 적재함 상단 이송 (Transfer to Bin)")
                self.set_motion("TRANSFER", transfer_pos, duration=2.5)

        elif self.state == "TRANSFER":
            if self.state_timer >= self.state_duration:
                place_pos = self.bin_pos + np.array([0, 0, 0.08])
                print(f"[{self.prefix.upper()}] 6단계: 적재함 안착 하강 (Place)")
                self.set_motion("PLACE", place_pos, duration=1.2)

        elif self.state == "PLACE":
            if self.state_timer >= self.state_duration:
                print(f"[{self.prefix.upper()}] 7단계: 물품 해제 (Release Open)")
                self.state = "RELEASE"
                self.state_timer = 0.0
                self.state_duration = 0.8
                self.gripper_ctrl = 0.0  # 열기

        elif self.state == "RELEASE":
            if self.state_timer >= self.state_duration:
                print(f"[{self.prefix.upper()}] 8단계: 기본 자세 복귀 (Return Home)")
                self.set_motion("RETRACT", None, duration=2.5)

        elif self.state == "RETRACT":
            if self.state_timer >= self.state_duration:
                self.state = "DONE"
                print(f"[{self.prefix.upper()}] 🎉 P&P 사이클 완료! (Pick & Place Success)")

        # 1kHz S-Curve 부드러운 궤적 보간 (Cosine Interpolation)
        alpha = min(1.0, self.state_timer / max(0.001, self.state_duration))
        s = 0.5 * (1.0 - np.cos(np.pi * alpha))
        current_cmd_qpos = (1.0 - s) * self.start_qpos + s * self.target_qpos

        # 액추에이터 제어 명령 인가 (data.ctrl)
        for i, q_val in enumerate(current_cmd_qpos):
            act_idx = self.ctrl_idx[i]
            self.data.ctrl[act_idx] = q_val
        self.data.ctrl[self.ctrl_idx[7]] = self.gripper_ctrl

def run_pnp_prototype(mode="r1", view=True):
    print("=" * 75)
    print(f"🚀 [Phase 03] 점진적 P&P 프로토타입 실행 - 모드: [{mode.upper()}]")
    print("=" * 75)

    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.abspath(os.path.join(script_dir, ".."))
    xml_path = os.path.join(
        project_root,
        "ros2_ws", "src", "franka_logistics_description", "mjcf", "scene_dual_panda_logistics.xml"
    )

    try:
        import mujoco
        import mujoco.viewer
        model = mujoco.MjModel.from_xml_path(xml_path)
        data = mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model, data, 0)
        mujoco.mj_forward(model, data)  # ★ 기하 트리 및 좌표 갱신 필수
    except Exception as e:
        print(f"❌ 오류: 모델 로드 실패: {e}")
        return False

    controllers = []
    if mode in ["r1", "dual"]:
        controllers.append(RobotController("r1", model, data))
    if mode in ["r2", "dual"]:
        controllers.append(RobotController("r2", model, data))

    dt = model.opt.timestep  # 0.001s (1ms)

    print(f"• 활성화된 로봇 제어기: {[c.prefix.upper() for c in controllers]}")
    print("• 시뮬레이션을 시작합니다. 3D 창을 확인하세요...")

    if view:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            # 1단계: P&P 모션 시뮬레이션 루프
            while viewer.is_running():
                step_start = time.time()
                
                # 1kHz 실시간 제어기 업데이트
                for ctrl in controllers:
                    ctrl.update(dt)
                
                # 1kHz 물리 적분 전진
                mujoco.mj_step(model, data)
                viewer.sync()
                
                # 모든 로봇의 P&P 작업이 완료되면 알림 출력 후 대기 루프로 진입
                if all(c.state == "DONE" for c in controllers):
                    print("\n" + "=" * 75)
                    print(f"🎉 [{mode.upper()} 모드] P&P 작업이 성공적으로 완료되었습니다!")
                    print("💡 3D 뷰어 창을 마우스로 자유롭게 조작해 보세요. 창을 닫으면 프로그램이 종료됩니다.")
                    print("=" * 75)
                    break
                
                elapsed = time.time() - step_start
                if dt > elapsed:
                    time.sleep(dt - elapsed)
            
            # 2단계: 작업 완료 후 사용자가 창을 닫을 때까지 뷰어 유지 루프
            while viewer.is_running():
                step_start = time.time()
                mujoco.mj_step(model, data)  # 로봇이 자세를 유지하도록 물리 연산 지속
                viewer.sync()
                elapsed = time.time() - step_start
                if dt > elapsed:
                    time.sleep(dt - elapsed)
    else:
        while True:
            for ctrl in controllers:
                ctrl.update(dt)
            mujoco.mj_step(model, data)
            if all(c.state == "DONE" for c in controllers):
                print("\n" + "=" * 75)
                print(f"🎉 [{mode.upper()} 모드] P&P 프로토타입 시뮬레이션이 성공적으로 완료되었습니다.")
                print("=" * 75)
                break

    return True

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Phase 03 Pick & Place Prototype")
    parser.add_argument("--mode", type=str, choices=["r1", "r2", "dual"], default="r1",
                        help="테스트 모드 선택: r1 (좌측 로봇), r2 (우측 로봇), dual (동시 구동)")
    parser.add_argument("--no-render", action="store_true", help="3D 뷰어 창 없이 고속 헤드리스 실행")
    args = parser.parse_args()

    run_pnp_prototype(mode=args.mode, view=not args.no_render)
```

---

### 🔹 Step 3: [테스트 1] Robot 1 (좌측 로봇) 단독 Pick & Place 실행

```bash
python test/phase03_pnp_prototype.py --mode r1
```
* **동작 확인**: 좌측 로봇이 `item_A_Red`로 부드럽게 접근 $\rightarrow$ 파지 $\rightarrow$ 들어올리기 $\rightarrow$ 좌측 상단 `bin_A_Red` 적재함에 안착 $\rightarrow$ Home 복귀하는지 3D 뷰어로 관찰합니다.

---

### 🔹 Step 4: [테스트 2] Robot 2 (우측 로봇) 단독 Pick & Place 실행

```bash
python test/phase03_pnp_prototype.py --mode r2
```
* **동작 확인**: 우측 로봇이 `item_C_Blue`를 집어 우측 하단 `bin_C_Blue` 적재함에 안착시키는지 확인합니다.

---

### 🔹 Step 5: [테스트 3] Robot 1 & Robot 2 동시 구동(Concurrent) P&P 실행

```bash
python test/phase03_pnp_prototype.py --mode dual
```
* **동작 확인**: 두 대의 로봇 팔(16개 액추에이터)이 1kHz 루프 내에서 병렬로 구동되는 다중 로봇 제어를 확인합니다.

---

## 4. 자주 발생하는 문제 및 해결법 (Troubleshooting)

### Q1. DLS 역기구학(IK) 계산 시 관절이 급격히 회전하거나 테이블을 뚫고 쳐집니다.
* **원인**:
  1. 모델 로드 직후 `mujoco.mj_forward(model, data)`를 호출하지 않아 물품/적재함의 초기 좌표가 `(0, 0, 0)`(바닥 중앙)으로 읽힌 경우.
  2. IK 솔버가 실시간 시뮬레이션 중인 `data.qpos`를 직접 덮어써서 물리 엔진의 연속성이 깨진 경우.
  3. 관절 한계(Joint Limits) 클램핑이 없어 Joint 4가 양수(위/아래 뒤집힘)로 꺾인 경우.
* **해결법**:
  * 모델 로드 직후 반드시 `mujoco.mj_forward(model, data)`를 호출합니다.
  * IK 계산 시 독립된 `ik_data` 인스턴스를 사용하고 `np.clip`으로 관절 리밋을 유지합니다.

### Q2. 그리퍼로 물건을 쥐는 순간 물건이 튀어 오르거나(Explosion) 미끄러집니다.
* **원인**: 핑거 패드와 물품 간 접촉 침투(Penetration) 시 반발 탄성 계수가 너무 크거나 마찰력이 부족한 경우입니다.
* **해결법**:
  * `scene_dual_panda_logistics.xml`의 `fingertip_pad_collision_1` 및 `item_box` 기본 속성에 `solref="0.005 1"`, `solimp="0.95 0.99 0.001"`, `friction="1.5 0.01 0.001"`이 정상 적용되어 있는지 확인합니다.

### Q3. 헤드리스(CLI 전용) 환경에서 3D 뷰어 창 없이 빠르게 연산만 테스트하고 싶습니다.
* **해결법**: `--no-render` 옵션을 주어 백그라운드 초고속 물리 연산으로 검증합니다:
  ```bash
  python test/phase03_pnp_prototype.py --mode dual --no-render
  ```

---

## 5. Phase 03 완료 체크리스트 (Self Checklist)

다음 항목들을 모두 확인한 후 다음 단계(Phase 04)로 진행하세요:

- [ ] `python test/phase03_explore_data.py`를 실행하여 `qpos`, `ctrl`, RGB/Depth, LiDAR 데이터가 정상 출력되는가?
- [ ] `python test/phase03_pnp_prototype.py --mode r1` 실행 시 Robot 1이 물품을 집어 적재함에 정상 안착시키는가?
- [ ] `python test/phase03_pnp_prototype.py --mode r2` 실행 시 Robot 2가 물품을 집어 적재함에 정상 안착시키는가?
- [ ] `python test/phase03_pnp_prototype.py --mode dual` 실행 시 두 로봇이 동시에 P&P 모션을 수행하는가?
- [ ] MuJoCo 물리 데이터 구조와 ROS2 메시지 간의 1:1 매핑 원리를 완전히 이해하였는가?

---

**다음 단계**: [Phase 04] MuJoCo ↔ ROS2 Sim Bridge 노드 구현 (`franka_logistics_sim`)
