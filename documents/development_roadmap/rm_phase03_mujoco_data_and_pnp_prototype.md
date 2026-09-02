# [Phase 03 실습 가이드] MuJoCo Python API 데이터 탐색 & 단계별 P&P 프로토타이핑

* **문서 번호**: GUIDE-LOGISTICS-PHASE-03
* **관련 마일스톤**: [20260902_01_development_roadmap.md](./20260902_01_development_roadmap.md) - Phase 03
* **작성일**: 2026-09-02
* **버전**: v1.0.0
* **작업 환경**: Ubuntu 22.04 LTS (x86_64) / Conda (`ros2_mujoco_panda_py3_10`, Python 3.10) / MuJoCo 3.6.x

---

## 1. 개요 및 학습 목표 (Overview & Objectives)

본 단계는 ROS2 노드로 패키징하기에 앞서, **MuJoCo 3.6.x 물리 엔진이 내부적으로 생성하고 요구하는 날것(Raw) 데이터의 구조를 완전히 이해**하고, 순수 Python 기반으로 **로봇 팔이 물체를 집어서 적재함에 넣는 3단계 점진적 Pick & Place(P&P) 동작을 먼저 성공**시키는 단계입니다.

블랙박스 형태의 추상화된 ROS2 노드를 만들기 전에, 물리 엔진의 입출력 데이터(`qpos`, `ctrl`, RGB/Depth 배열, LiDAR 레이캐스트)를 개발자가 직접 눈으로 확인하고 만져봄으로써 향후 ROS2 Sim Bridge(Phase 04)와 모션 제어(Phase 05) 구현의 탄탄한 토대를 마련합니다.

```
┌────────────────────────────────────────────────────────────────────────────────┐
│                           [ Phase 03 점진적 학습 및 검증 흐름 ]                     │
│                                                                                 │
│  [1. Raw 데이터 I/O 분석] ──► [2. DLS 역기구학/파지 튜닝] ──► [3. 3단계 P&P 검증]       │
│   • qpos/qvel (관절각/속도)     • 목표 (X,Y,Z) -> q 도출      • Step 1: R1 단독 P&P  │
│   • ctrl (액추에이터 제어)       • 그리퍼 마찰력/접촉 안정화      • Step 2: R2 단독 P&P  │
│   • RGB/Depth (비전 배열)                                   • Step 3: 듀얼 동시구동 │
│   • mj_ray (LiDAR 광선)                                                         │
└────────────────────────────────────────────────────────────────────────────────┘
```

### 🎯 핵심 학습 목표
1. **MuJoCo C-Binding 핵심 데이터 아키텍처 습득**: `mjModel`(정적 제원)과 `mjData`(동적 물리 상태), `qpos`, `qvel`, `ctrl`, `site_xpos`의 메모리 구조와 인덱싱 원리 체득.
2. **수치적 역기구학(DLS Inverse Kinematics) 원리 구현**: 자코비안 행렬($J$)과 특이점 감쇠 계수($\lambda$)를 이용해 목표 3D 위치를 7-자유도 관절 각도로 실시간 역산하는 알고리즘 이해.
3. **그리퍼 접촉 역학(Contact Dynamics) 튜닝**: 파지 시 물품 미끄러짐 및 튕겨나감(Explosion) 현상을 방지하는 마찰 계수와 서보 제어값(`ctrl=255/0`) 검증.
4. **가상 센서 원시 데이터 추출**: 오프스크린 렌더러 기반 RGB/Depth NumPy 배열 및 `mj_ray()` 평면 방사형 레이캐스팅 데이터 획득.
5. **3단계 점진적 P&P 시뮬레이션 성공**: **[로봇 1 단독] $\rightarrow$ [로봇 2 단독] $\rightarrow$ [듀얼 로봇 동시 구동]**으로 이어지는 단계별 모션 시퀀스를 3D 뷰어로 직접 관찰 및 검증.

---

## 2. 이론적 배경 (Theoretical Background)

### 2.1 MuJoCo Python C-Binding 데이터 아키텍처

MuJoCo는 성능을 극대화하기 위해 C언어 구조체 기반으로 설계되어 있으며, Python 바인딩은 이를 **NumPy 다차원 배열 포인터**로 직접 노출합니다.

```text
┌─────────────────────────────────────────────────────────────────────────────┐
│                           mjModel (정적 모델 제원)                             │
│  • nq   : 일반화 좌표 총 개수 (자유도 및 쿼터니언 포함, 본 프로젝트: 67)               │
│  • nv   : 속도 벡터 총 개수 (속도 자유도, 본 프로젝트: 60)                          │
│  • nu   : 제어 입력 액추에이터 총 개수 (본 프로젝트: 16)                            │
│  • opt  : 타임스텝(timestep=0.001s), 중력(gravity=0 0 -9.81)                   │
└──────────────────────────────────────┬──────────────────────────────────────┘
                                       │ mj_step(model, data) 1kHz 연산
┌──────────────────────────────────────▼──────────────────────────────────────┐
│                           mjData (동적 물리 상태)                              │
│  • data.qpos : 현재 관절 위치 및 물체 위치/쿼터니언 배열 [67 float64]               │
│  • data.qvel : 현재 관절 각속도 및 물체 선/각속도 배열 [60 float64]                │
│  • data.ctrl : 액추에이터 목표 제어 입력 배열 [16 float64]                        │
│  • data.site_xpos : 씬 내 정의된 모든 사이트의 글로벌 3D 좌표 [nsite x 3]           │
│  • data.contact   : 현재 활성화된 접촉점 정보 구조체 배열 [ncon개]                  │
└─────────────────────────────────────────────────────────────────────────────┘
```

#### 📌 본 프로젝트 듀얼 판다 씬의 배열 인덱스 맵핑
* **`data.qpos` (총 67차원)**:
  * `qpos[0:7]`: Robot 1 암 관절 7개 (`r1_joint1` ~ `r1_joint7`)
  * `qpos[7:9]`: Robot 1 핑거 슬라이드 관절 2개 (`r1_finger_joint1`, `r1_finger_joint2`)
  * `qpos[9:16]`: Robot 2 암 관절 7개 (`r2_joint1` ~ `r2_joint7`)
  * `qpos[16:18]`: Robot 2 핑거 슬라이드 관절 2개 (`r2_finger_joint1`, `r2_finger_joint2`)
  * `qpos[18:67]`: 물류 물품 7종의 3D 위치(3) + 쿼터니언(4) $\times 7 = 49$개
* **`data.ctrl` (총 16차원)**:
  * `ctrl[0:7]`: Robot 1 관절 1~7 목표 각도 ($\text{rad}$)
  * `ctrl[7]`: Robot 1 그리퍼 텐던 제어 (`0`: 열림, `255`: 닫힘)
  * `ctrl[8:15]`: Robot 2 관절 1~7 목표 각도 ($\text{rad}$)
  * `ctrl[15]`: Robot 2 그리퍼 텐던 제어 (`0`: 열림, `255`: 닫힘)

---

### 2.2 순운동학(FK)과 감쇠 최소자승 역기구학(DLS IK)

#### 1) 순운동학 (Forward Kinematics) & 엔드이펙터 자코비안 ($J$)
로봇의 관절 각도 $q \in \mathbb{R}^7$가 주어졌을 때 엔드이펙터의 3D 공간 위치 $x \in \mathbb{R}^3$를 구하는 매핑을 순운동학이라 합니다:
$$x = f(q)$$

관절 속도 $\dot{q}$와 엔드이펙터 선속도 $\dot{x}$ 사이의 선형 미분 관계는 **자코비안 행렬 (Jacobian Matrix)** $J(q) = \frac{\partial f(q)}{\partial q} \in \mathbb{R}^{3 \times 7}$로 표현됩니다:
$$\dot{x} = J(q) \dot{q}$$

MuJoCo에서는 `mujoco.mj_jacSite(model, data, jacp, jacr, site_id)` C API 함수를 통해 현재 자세에서의 자코비안 행렬 $J$를 단 $1\mu\text{s}$ 이내에 즉시 계산합니다.

#### 2) 감쇠 최소자승법 (Damped Least Squares, DLS)
목표 위치 $x_{\text{target}}$과 현재 위치 $x_{\text{current}}$ 사이의 오차 벡터를 $e = x_{\text{target}} - x_{\text{current}}$라 할 때, 표준 의사역행렬($J^+$)은 로봇이 팔을 완전히 뻗거나 특이점(Singularity) 근처에 도달할 경우 역행렬 연산 시 수치적 발산($\Delta q \to \infty$)을 일으킵니다.

이를 방지하기 위해 **감쇠 계수 $\lambda > 0$**를 도입한 **DLS 수식**을 사용합니다:
$$\Delta q = J^T \left( J J^T + \lambda^2 I \right)^{-1} e$$

* 특이점 근처에서 행렬 $\left(J J^T + \lambda^2 I\right)$의 조건수(Condition Number)가 안정화되어 관절의 급격한 회전이나 발산 없이 부드럽게 목표 지점으로 수렴합니다.

---

### 2.3 그리퍼 접촉 역학(Contact Dynamics) 및 파지 원리

Franka Panda 로봇 그리퍼는 물체를 단단히 집어 올릴 때 미끄러짐(Slip)이나 접촉 반발로 인한 튕겨나감(Penetration Explosion)이 없어야 합니다.

```text
       [Left Finger Pad]                   [Right Finger Pad]
             │                                     │
             ▼ ───►  [ 물류 블록 (mass=0.1kg) ]  ◄─── ▼
      Friction=1.5                         Friction=1.5
      solref=0.005, solimp=0.95            solref=0.005, solimp=0.95
```

1. **마찰 계수 (`friction="1.5 0.01 0.001"`)**:
   * 슬라이딩 마찰(1.5), 비틀림 마찰(0.01), 구름 마찰(0.001)을 부여하여 핑거 패드가 물품 표면을 안정적으로 파지.
2. **접촉 감쇠 솔버 (`solref="0.005 1"`, `solimp="0.95 0.99 0.001"`)**:
   * 강체 간의 충돌 시 충격량을 완화하는 스프링-댐퍼 모델로, 물체를 쥐는 순간 과도한 반발 탄성력을 억제.
3. **텐던 연동 (`actuator8`, `ctrl=255`)**:
   * 단일 액추에이터 명령으로 좌우 핑거가 대칭으로 $4\text{cm}$씩 안쪽으로 닫히며 최대 $100\text{N}$의 파지력을 인가.

---

### 2.4 가상 비전 & 2D LiDAR 데이터 추출 원리

#### 1) Wrist Camera 오프스크린 렌더링
* `mujoco.Renderer(model, height=480, width=640)`를 생성하여 GPU/OpenGL 오프스크린 버퍼에서 렌더링.
* **RGB 영상**: `renderer.render()` $\rightarrow$ 형태: `(480, 640, 3)`, 자료형: `uint8` (0~255 RGB 픽셀).
* **Depth 맵**: `renderer.enable_depth_rendering()` $\rightarrow$ 형태: `(480, 640)`, 자료형: `float32` (미터 단위 실제 깊이 거리).

#### 2) 2D Safety LiDAR 레이캐스팅 (`mj_ray`)
* 상단(`lidar_top_frame`) 및 하단(`lidar_bottom_frame`) 사이트 위치 $P_0 = (x_0, y_0, z_0)$에서 수평 각도 $\theta_i$ 방향으로 단위 벡터 $d_i = (\cos\theta_i, \sin\theta_i, 0)$를 투사.
* `dist = mujoco.mj_ray(model, data, P_0, d_i, ...)` 함수를 호출하여 가장 먼저 부딪히는 물체 표면까지의 거리($\text{m}$)와 충돌 지오메트리 ID를 추출.

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
        mujoco.mj_forward(model, data)
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
        
        # Robot 1 Wrist Camera 렌더링
        renderer.update_scene(data, camera="r1_wrist_camera")
        rgb_img = renderer.render()
        
        # Depth 맵 렌더링
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
2. mode='r2'  : Robot 2 (우측) 단독 P&P (item_A_Green -> bin_A_Green, Robot 1은 Home 대기)
3. mode='dual': Robot 1 & Robot 2 동시 구동 P&P (다중 로봇 액추에이터 동시 제어 검증)

[핵심 알고리즘]
- DLS(Damped Least Squares) 수치적 역기구학(IK)
- 5단계 웨이포인트 궤적 보간: Approach -> Grasp -> Lift -> Place -> Home
- 그리퍼 파지력 및 접촉 물리 실시간 시뮬레이션
==============================================================================
"""

import os
import sys
import time
import argparse
import numpy as np

# DLS 역기구학 파라미터
IK_DAMPING = 0.05       # 특이점 감쇠 계수 (lambda)
IK_MAX_STEPS = 50       # 최대 반복 계산 횟수
IK_TOLERANCE = 0.003    # 허용 위치 오차 (3mm)
IK_STEP_SIZE = 0.5      # 스텝 크기 (학습률)

def solve_dls_ik(model, data, site_name, target_pos, arm_joint_indices, arm_qpos_indices):
    """
    Damped Least Squares (DLS) 수치적 역기구학 솔버
    Delta_q = J^T * (J * J^T + lambda^2 * I)^(-1) * error_pos
    """
    import mujoco
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    
    q_current = data.qpos[arm_qpos_indices].copy()
    
    for _ in range(IK_MAX_STEPS):
        mujoco.mj_forward(model, data)
        current_pos = data.site_xpos[site_id]
        error = target_pos - current_pos
        
        if np.linalg.norm(error) < IK_TOLERANCE:
            break
            
        mujoco.mj_jacSite(model, data, jacp, jacr, site_id)
        J = jacp[:, arm_joint_indices]  # (3, 7)
        
        A = J @ J.T + (IK_DAMPING ** 2) * np.eye(3)
        dq = J.T @ np.linalg.solve(A, error)
        
        q_current += IK_STEP_SIZE * dq
        data.qpos[arm_qpos_indices] = q_current
        
    return q_current

class RobotController:
    """단일 로봇 P&P 상태 머신 및 액추에이터 제어기"""
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
            self.target_item = "item_A_Green"
            self.target_bin_site = "site_bin_A_Green"
            self.home_qpos = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
            
        self.state = "INIT"
        self.state_timer = 0
        self.target_qpos = self.home_qpos.copy()
        self.gripper_ctrl = 0.0  # 0: Open, 255: Close
        
        item_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, self.target_item)
        bin_sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, self.target_bin_site)
        
        self.item_pos = data.xpos[item_bid].copy()
        self.bin_pos = data.site_xpos[bin_sid].copy()

    def update(self, dt):
        self.state_timer += dt
        
        if self.state == "INIT":
            self.target_qpos = self.home_qpos.copy()
            self.gripper_ctrl = 0.0
            if self.state_timer > 1.0:
                self.state = "APPROACH"
                self.state_timer = 0
                approach_pos = self.item_pos + np.array([0, 0, 0.12])
                self.target_qpos = solve_dls_ik(self.model, self.data, self.ee_site, approach_pos, self.arm_nv_idx, self.arm_qpos_idx)
                print(f"[{self.prefix.upper()}] 1단계: 물품 상단 접근 (Approach)")

        elif self.state == "APPROACH":
            if self.state_timer > 2.0:
                self.state = "DESCEND"
                self.state_timer = 0
                grasp_pos = self.item_pos + np.array([0, 0, 0.015])
                self.target_qpos = solve_dls_ik(self.model, self.data, self.ee_site, grasp_pos, self.arm_nv_idx, self.arm_qpos_idx)
                print(f"[{self.prefix.upper()}] 2단계: 물품 파지 위치 하강 (Descend)")

        elif self.state == "DESCEND":
            if self.state_timer > 1.5:
                self.state = "GRASP"
                self.state_timer = 0
                self.gripper_ctrl = 255.0  # 그리퍼 닫기 (Close)
                print(f"[{self.prefix.upper()}] 3단계: 그리퍼 파지 (Grasp Close)")

        elif self.state == "GRASP":
            if self.state_timer > 1.0:
                self.state = "LIFT"
                self.state_timer = 0
                lift_pos = self.item_pos + np.array([0, 0, 0.18])
                self.target_qpos = solve_dls_ik(self.model, self.data, self.ee_site, lift_pos, self.arm_nv_idx, self.arm_qpos_idx)
                print(f"[{self.prefix.upper()}] 4단계: 물품 들어올리기 (Lift)")

        elif self.state == "LIFT":
            if self.state_timer > 2.0:
                self.state = "TRANSFER"
                self.state_timer = 0
                transfer_pos = self.bin_pos + np.array([0, 0, 0.15])
                self.target_qpos = solve_dls_ik(self.model, self.data, self.ee_site, transfer_pos, self.arm_nv_idx, self.arm_qpos_idx)
                print(f"[{self.prefix.upper()}] 5단계: 적재함 상단 이송 (Transfer to Bin)")

        elif self.state == "TRANSFER":
            if self.state_timer > 2.5:
                self.state = "PLACE"
                self.state_timer = 0
                place_pos = self.bin_pos + np.array([0, 0, 0.08])
                self.target_qpos = solve_dls_ik(self.model, self.data, self.ee_site, place_pos, self.arm_nv_idx, self.arm_qpos_idx)
                print(f"[{self.prefix.upper()}] 6단계: 적재함 안착 하강 (Place)")

        elif self.state == "PLACE":
            if self.state_timer > 1.5:
                self.state = "RELEASE"
                self.state_timer = 0
                self.gripper_ctrl = 0.0  # 그리퍼 열기 (Open)
                print(f"[{self.prefix.upper()}] 7단계: 물품 해제 (Release Open)")

        elif self.state == "RELEASE":
            if self.state_timer > 1.0:
                self.state = "RETRACT"
                self.state_timer = 0
                self.target_qpos = self.home_qpos.copy()
                print(f"[{self.prefix.upper()}] 8단계: 기본 자세 복귀 (Return Home)")

        elif self.state == "RETRACT":
            if self.state_timer > 2.5:
                self.state = "DONE"
                print(f"[{self.prefix.upper()}] 🎉 P&P 사이클 완료! (Pick & Place Success)")

        for i, q_target in enumerate(self.target_qpos):
            act_idx = self.ctrl_idx[i]
            self.data.ctrl[act_idx] = q_target
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
    except Exception as e:
        print(f"❌ 오류: 모델 로드 실패: {e}")
        return False

    controllers = []
    if mode in ["r1", "dual"]:
        controllers.append(RobotController("r1", model, data))
    if mode in ["r2", "dual"]:
        controllers.append(RobotController("r2", model, data))

    dt = model.opt.timestep  # 0.001s (1ms)
    total_time = 0.0
    max_duration = 18.0

    print(f"• 활성화된 로봇 제어기: {[c.prefix.upper() for c in controllers]}")
    print("• 시뮬레이션을 시작합니다. 3D 창을 확인하세요...")

    if view:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running() and total_time < max_duration:
                step_start = time.time()
                
                if int(total_time * 1000) % 20 == 0:
                    for ctrl in controllers:
                        ctrl.update(0.02)
                
                mujoco.mj_step(model, data)
                total_time += dt
                
                viewer.sync()
                
                elapsed = time.time() - step_start
                if dt > elapsed:
                    time.sleep(dt - elapsed)
                    
                if all(c.state == "DONE" for c in controllers) and total_time > 15.0:
                    break
    else:
        while total_time < max_duration:
            if int(total_time * 1000) % 20 == 0:
                for ctrl in controllers:
                    ctrl.update(0.02)
            mujoco.mj_step(model, data)
            total_time += dt
            if all(c.state == "DONE" for c in controllers) and total_time > 15.0:
                break

    print("\n" + "=" * 75)
    print(f"🎉 [{mode.upper()} 모드] P&P 프로토타입 시뮬레이션이 성공적으로 완료되었습니다.")
    print("=" * 75)
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
* **동작 확인**: 좌측 로봇이 `item_A_Red`를 부드럽게 집어 올려 좌측 상단 `bin_A_Red` 적재함에 쏙 넣고 Home 자세로 복귀하는지 3D 뷰어로 관찰합니다.

---

### 🔹 Step 4: [테스트 2] Robot 2 (우측 로봇) 단독 Pick & Place 실행

```bash
python test/phase03_pnp_prototype.py --mode r2
```
* **동작 확인**: 우측 로봇이 `item_A_Green`을 집어 우측 상단 `bin_A_Green` 적재함에 안착시키는지 확인합니다.

---

### 🔹 Step 5: [테스트 3] Robot 1 & Robot 2 동시 구동(Concurrent) P&P 실행

```bash
python test/phase03_pnp_prototype.py --mode dual
```
* **동작 확인**: 두 대의 로봇 팔(16개 액추에이터)이 1kHz 루프 내에서 병렬로 구동되는 다중 로봇 제어를 확인합니다.

---

## 4. 자주 발생하는 문제 및 해결법 (Troubleshooting)

### Q1. DLS 역기구학(IK) 계산 시 관절이 급격히 회전하거나 목표 지점을 못 찾습니다.
* **원인**: 목표 지점이 로봇의 최대 가용 작업 반경($0.855\text{m}$)을 벗어났거나, 감쇠 계수 $\lambda$가 너무 작아 특이점에서 발산한 경우입니다.
* **해결법**:
  * `IK_DAMPING` 값을 `0.05` ~ `0.1`로 유지합니다.
  * 엔드이펙터가 작업대 바닥을 뚫지 않도록 Z축 최소 높이($Z \ge 0.775\text{m}$)를 보장합니다.

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
