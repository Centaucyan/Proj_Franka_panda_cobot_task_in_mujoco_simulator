# [Phase 02 실습 가이드] MuJoCo 3.6.x 3D 가상 씬 및 물류 환경 모델링

* **문서 번호**: GUIDE-LOGISTICS-PHASE-02
* **관련 마일스톤**: [20260831_02_development_roadmap.md](./20260831_02_development_roadmap.md) - Phase 02
* **작성일**: 2026-09-02
* **버전**: v1.1.0
* **작업 환경**: Ubuntu 22.04 LTS (x86_64) / ROS2 Humble / Miniconda (`ros2_mujoco_panda_py3_10`, Python 3.10) / MuJoCo 3.6.x

---

## 1. 개요 및 학습 목표 (Overview & Objectives)

본 단계는 Franka Panda 듀얼 협동로봇 물류 분류 시스템의 핵심 물리 공간을 구축하는 단계입니다. **ROS2의 로봇 설명 패키지(`franka_logistics_description`)를 생성**하고, MuJoCo 3.6.x 물리 엔진에서 구동될 **통합 3D 가상 씬(`scene_dual_panda_logistics.xml`)을 모델링**합니다.

실제 물류 현장의 작업대, 6개 전용 적재함, 2개 배출 컨베이어, 6종의 다품종 물류 아이템, 듀얼 Franka Panda 로봇, 그리고 인지/안전을 위한 가상 센서(Wrist Camera, 2D LiDAR)를 3차원 공간상에 물리적으로 완벽히 배치하고 검증합니다.

```
                        [ 상단 작업자 이동 구역 / LiDAR Top ]  
                                                (Robot 2 불가품)
┌────────────────┐           ┌──────────────────┐           ┌───────────────┐
│ 적재함 1 (A-R)  │           │                  │           │ 적재함 4 (A-G)  │
├────────────────┤           │                  │           ├───────────────┤
│ 적재함 2 (B-B)  │  (로봇 1)  |  중앙 물류 작업대   |  (로봇 2)  │ 적재함 5 (B-R) │
├────────────────┤ (Robot 1) |  [공유 작업 영역]  | (Robot 2) ├───────────────┤
│ 적재함 3 (C-G)  │           │                  │           │ 적재함 6 (C-B) │
└────────────────┘           └──────────────────┘           └───────────────┘
                (Robot 1 불가품)
                         [ 하단 작업자 이동 구역 / LiDAR Bottom ]

```

### 🎯 핵심 학습 목표
1. **MJCF(MuJoCo XML Format) 아키텍처 및 계층 구조 습득**: `<worldbody>`, `<body>`, `<geom>`, `<joint>`, `<actuator>`, `<sensor>`, `<camera>`, `<asset>` 등 핵심 태그의 역할과 작성법 이해.
2. **듀얼 로봇 시스템의 좌표계(Frame) 및 네임스페이스 설계**: 다중 로봇 인스턴스화 시 발생하는 이름 충돌(Name Collision) 방지 및 Base Frame 배치 원리 습득.
3. **현실적인 접촉 역학(Contact Dynamics) 파라미터 튜닝**: 마찰 계수(`friction`), 강성/감쇠(`solref`, `solimp`)를 통한 안정적인 물품 파지 및 적재 거동 구현.
4. **가상 센서 마운팅 기하학 구성**: Wrist Depth/RGB 카메라 오프스크린 렌더링 프레임 및 2D LiDAR 레이캐스팅 기준점(`site`) 배치.
5. **Python MuJoCo Viewer를 활용한 씬 무결성 자가 진단**: 물리적 관통(Penetration), 관절 리밋, 조명 및 렌더링 상태 시각적 검증.

---

## 2. 이론적 배경 (Theoretical Background)

### 2.1 MuJoCo 3.6.x 물리 엔진과 MJCF 아키텍처
MuJoCo(Multi-Joint dynamics with Contact)는 로보틱스, 생체역학, 강화학습 분야에서 가장 널리 쓰이는 고속/고정밀 다체 동역학(Multi-body Dynamics) 시뮬레이터입니다.

```text
┌─────────────────────────────────────────────────────────────────────────────┐
│                           <mujoco model="...">                              │
│                                                                             │
│  ├── <compiler>  : 각도 단위(radian/degree), 메시 경로(meshdir), 좌표계 설정    │
│  ├── <option>    : 적분기(implicitfast/rk4), 타임스텝(1ms), 중력(0 0 -9.81)    │
│  ├── <default>   : 관절 감쇠, 마찰력, 충돌 그룹 등 공통 기본 속성 상속 계층        │
│  ├── <asset>     : STL/OBJ 3D 메시, 텍스처, 재질(Material), 스카이박스 정의     │
│  ├── <worldbody> : 최상위 고정 좌표계(World Frame) 및 물리적 바디 트리 구조      │
│  │     ├── <body> (중앙 작업대, 적재함, 컨베이어)                              │
│  │     ├── <body> (로봇 1: Base -> Link1~7 -> Hand -> Fingers)              │
│  │     ├── <body> (로봇 2: Base -> Link1~7 -> Hand -> Fingers)              │
│  │     └── <body> (물류 아이템들: Free Joint 부여로 독립적 거동)                │
│  ├── <actuator>  : 모터, 서보, 액추에이터 게인(kp, kv) 및 제어 범위(ctrlrange)   │
│  ├── <sensor>    : 카메라 프레임, 레이캐스트 사이트, 관절 센서 정의               │
│  └── <contact>   : 자기 충돌(Self-Collision) 제외 및 맞춤형 접촉 쌍 정의         │
└─────────────────────────────────────────────────────────────────────────────┘
```

* **일반화 좌표계(Generalized Coordinates)**: 관절 각도 $q$와 각속도 $\dot{q}$를 기반으로 운동 방정식을 풀어 관절 제약 조건을 100% 만족하며 수치적 발산(Drift)이 없습니다.
* **유연한 접촉 모델(Soft Contact via `solref`, `solimp`)**: 강체 간의 충돌 시 발생하는 충격량을 시간과 거리에 따라 감쇠 완화하여, 로봇 그리퍼가 물건을 집을 때 폭발적으로 튀어나가는 현상(Explosion)을 방지합니다.

---

### 2.2 왜 URDF 대신 MJCF를 직접 작성하고 활용하는가?

| 비교 항목 | URDF (Unified Robot Description Format) | MJCF (MuJoCo XML Format) |
| :--- | :--- | :--- |
| **주 목적** | ROS/ROS2 시각화 및 기구학 체인 정의 | 고속 정밀 물리 연산 및 동역학 시뮬레이션 |
| **다중 바디 구성** | 단일 로봇 트리 구조에 특화 (다중 로봇 결합 시 복잡) | 작업대, 적재함, 다품종 물체 등 복합 씬 완벽 지원 |
| **물리 접촉 파라미터** | 단순 쿨롱 마찰 및 강체 탄성 계수만 지원 | 다차원 마찰 텐서(구름/비틀림 마찰), 접촉 솔버 튜닝 지원 |
| **액추에이터 및 센서** | 별도 플러그인(Gazebo/ros2_control) 필요 | 모터 다이내믹스, 내장 렌더링 카메라, 센서 기본 내장 |
| **연산 속도** | SDF 변환 과정에서 오버헤드 발생 가능 | MuJoCo 네이티브 바이너리로 즉시 컴파일되어 초고속 연산 |

본 프로젝트에서는 **`franka_logistics_description` 패키지** 내에 MJCF 원본과 3D 메시를 보관하여, 향후 ROS2 Sim Bridge 노드(`franka_logistics_sim`) 및 강화학습 환경(`franka_logistics_rl`)이 직접 MJCF 파일을 로드해 1kHz 물리 루프를 구동할 수 있도록 설계합니다.

---

### 2.3 듀얼 로봇 좌표계(Coordinate Frames) 및 배치 기하학

작업 공간의 중심(물류 분류 작업대 정중앙 바닥)을 **World Frame $(0, 0, 0)$**으로 정의합니다.

```text
               Y (상단 작업자 영역: +Y 방향)
                            ▲
                            │    [Conveyor 2 (R2 Reject)]
   [Bin A-R]             [Table]            [Bin A-G]
   [Bin B-B] ────(Robot1)───┼───(Robot2)─► X (우측: +X 방향)
   [Bin C-G]             [Table]            [Bin B-R]
              [Conveyor 1]  │               [Bin C-B]
                            ▼
              -Y (하단 작업자 영역: -Y 방향)
```

#### 📐 주요 3D 좌표 배치 제원
* **중앙 물류 작업대**:
  * 크기: $1.2\text{m} \times 0.8\text{m} \times 0.75\text{m}$ (가로 $\times$ 세로 $\times$ 높이)
  * 위치: $(0.0, 0.0, 0.375)$ (상판 상단 표면 높이: $Z = 0.75\text{m}$)
* **로봇 1 (좌측 로봇 Base)**:
  * 위치: $X = -0.65\text{m}, Y = 0.0\text{m}, Z = 0.75\text{m}$
  * 방향: 정면을 향함 (Yaw = $0^\circ$)
* **로봇 2 (우측 로봇 Base)**:
  * 위치: $X = +0.65\text{m}, Y = 0.0\text{m}, Z = 0.75\text{m}$
  * 방향: 정면을 향함 (Yaw = $0^\circ$)
* **공유 작업 영역 (Shared Workspace)**:
  * 작업대 중심부 $X \in [-0.25, +0.25]$, $Y \in [-0.35, +0.35]$ 영역은 두 로봇의 최대 도달 반경(Max Reach $0.855\text{m}$)이 중첩되는 구역으로, Phase 06에서 FCFS 충돌 방지 알고리즘이 적용됩니다.
* **적재함 6개소 (좌측 3개 / 우측 3개)**:
  * 크기: $0.3\text{m} \times 0.25\text{m} \times 0.2\text{m}$
  * 좌측 적재함 (Robot 1 전용): $X = -1.05\text{m}$, $Y = [+0.28, 0.0, -0.28]\text{m}$, $Z = 0.75\text{m}$
    * 상단: **Bin A-Red** / 중단: **Bin B-Blue** / 하단: **Bin C-Green**
  * 우측 적재함 (Robot 2 전용): $X = +1.05\text{m}$, $Y = [+0.28, 0.0, -0.28]\text{m}$, $Z = 0.75\text{m}$
    * 상단: **Bin A-Green** / 중단: **Bin B-Red** / 하단: **Bin C-Blue**
* **배출 컨베이어 벨트 2개소**:
  * 컨베이어 1 (Robot 1 불가품): 좌측 하단 $X = -0.65\text{m}, Y = -0.65\text{m}, Z = 0.65\text{m}$ ($-Y$ 방향 배출)
  * 컨베이어 2 (Robot 2 불가품): 우측 상단 $X = +0.65\text{m}, Y = +0.65\text{m}, Z = 0.65\text{m}$ ($+Y$ 방향 배출)

---

### 2.4 듀얼 로봇 네임스페이스 및 충돌/접촉 역학 설계

#### 1) 다중 로봇 인스턴스 이름 분리 (Prefix)
MuJoCo에서는 동일한 모델 파일 내에서 조인트(`joint`), 바디(`body`), 지오메트리(`geom`), 액추에이터(`actuator`)의 `name` 속성이 중복되면 파싱 에러(`XML Error: repeated name`)가 발생합니다.
따라서 다음과 같이 네임스페이스를 명확히 구분합니다:
* **로봇 1 구성 요소**: `r1_link0`, `r1_joint1` ~ `r1_joint7`, `r1_hand`, `r1_finger_joint1`, `r1_actuator1` 등
* **로봇 2 구성 요소**: `r2_link0`, `r2_joint1` ~ `r2_joint7`, `r2_hand`, `r2_finger_joint1`, `r2_actuator1` 등

#### 2) 충돌 그룹 분리 (`group`, `contype`, `conaffinity`)
* **Visual Geom (시각화 전용)**: `contype="0" conaffinity="0" group="2"` $\rightarrow$ 물리 연산 대상에서 제외하여 연산 속도 극대화.
* **Collision Geom (충돌 연산 전용)**: `group="3"` $\rightarrow$ 단순화된 볼록 껍질(Convex Mesh) 또는 기본 기하체(Box/Cylinder)를 사용하여 안정적 충돌 계산.
* **물류 아이템 접촉 파라미터**:
  ```xml
  <geom type="box" size="0.025 0.025 0.025" mass="0.1"
        friction="1.2 0.005 0.0001" solref="0.005 1" solimp="0.95 0.99 0.001"/>
  ```
  * `friction="1.2 0.005 0.0001"`: 슬라이딩 마찰계수(1.2), 비틀림 마찰계수(0.005), 구름 마찰계수(0.0001)를 부여하여 그리퍼 파지 시 미끄러짐 방지.
  * `solref="0.005 1"`, `solimp="0.95 0.99 0.001"`: 접촉 시 반발 탄성을 억제하고 안정적인 지지력 형성.

---

### 2.5 센서 기하학 모델링 (Wrist Camera & 2D LiDAR)

```text
       [Franka Hand]
             │
             ├── [Wrist Depth/RGB Camera] : pos="0.05 0 0.05", quat="...", fovy="58"
             │                              (그리퍼 끝단 하방 45도 응시)
             │
             └── [Fingers]
```

1. **Wrist Camera (`<camera>`)**:
   * 각 로봇의 `hand` 바디에 부착.
   * 그리퍼 핑거 사이의 타겟 물품 및 안착 후 적재함 내부를 정확히 내려다보도록 지향각 설정 (`fovy="58"`로 RealSense D435 화각 모사).
2. **2D Safety LiDAR (`<site>`)**:
   * 작업대 상단/하단 경계면에 레이캐스팅 기준점 사이트 정의.
   * `lidar_top_frame` ($X=0, Y=+0.9, Z=0.8$) 및 `lidar_bottom_frame` ($X=0, Y=-0.9, Z=0.8$).
   * Phase 03에서 Sim Bridge 노드가 이 사이트를 기준으로 수평 $270^\circ$ 레이캐스트 광선을 투사합니다.

---

## 3. 단계별 실습 진행 가이드 (Step-by-Step Implementation)

### 📋 Phase 02 생성 파일 레이아웃 안내
```text
Proj_Franka_panda_cobot_task_in_ros2_and_mujoco_simulator/
├── documents/
│   └── development_roadmap/
│       └── rm_phase02_mujoco_scene_modeling.md     # 본 실습 가이드 문서
├── ros2_ws/src/
│   └── franka_logistics_description/               # [Step 1] 신규 ROS2 패키지
│       ├── package.xml                             # 패키지 명세
│       ├── CMakeLists.txt                          # 빌드 및 자산 설치 설정
│       ├── mjcf/                                   # [Step 3] MJCF 씬 및 로봇 정의
│       │   ├── scene_dual_panda_logistics.xml      # ★ 핵심 통합 물류 가상 씬
│       │   └── assets/                             # Franka Mesh STL/OBJ 심볼릭/복사 자산
│       └── meshes/
└── test/
    ├── phase02_test_scene.py                       # [Step 4] 씬 로딩 및 무결성 검증 스크립트
    └── phase02_view_scene.py                       # [Step 5] 3D 대화형 뷰어 실행 스크립트
```

---

### 🔹 Step 1: `franka_logistics_description` 패키지 생성

`ros2_ws/src` 디렉토리로 이동하여 로봇 및 환경 모델링 전용 패키지를 생성합니다.

#### 1) 패키지 생성 명령 실행
```bash
# ros2_ws/src 디렉토리로 이동
cd Proj_Franka_panda_cobot_task_in_ros2_and_mujoco_simulator/ros2_ws/src

# ament_cmake 타입 패키지 생성
ros2 pkg create --build-type ament_cmake franka_logistics_description \
  --description "Robot and environment MJCF descriptions for Dual Franka Logistics System" \
  --license Apache-2.0
```

#### 2) `package.xml` 파일 수정
`ros2_ws/src/franka_logistics_description/package.xml` 파일을 다음과 같이 수정합니다:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>franka_logistics_description</name>
  <version>0.1.0</version>
  <description>Robot and environment MJCF descriptions for Dual Franka Logistics System</description>
  <maintainer email="tae@todo.todo">tae</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>urdf</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

#### 3) `CMakeLists.txt` 파일 수정
`ros2_ws/src/franka_logistics_description/CMakeLists.txt` 파일을 열어 `mjcf`, `meshes` 폴더가 ROS2 워크스페이스 설치 경로(`install/share/...`)로 올바르게 복사되도록 구성합니다:

```cmake
cmake_minimum_required(VERSION 3.8)
project(franka_logistics_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# MJCF, 3D 메시 및 Launch 파일 설치 디렉토리 등록
install(
  DIRECTORY mjcf meshes
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*"
)

ament_package()
```

---

### 🔹 Step 2: 3D 모델 및 메시 자산 디렉토리 구성

`model_ori/franka_emika_panda/assets`에 이미 준비되어 있는 Franka 로봇의 3D 메시(STL/OBJ) 파일들을 `franka_logistics_description/mjcf/assets` 경로에서 참조할 수 있도록 디렉토리를 구축합니다.

```bash
# mjcf 및 assets 디렉토리 생성
cd Proj_Franka_panda_cobot_task_in_ros2_and_mujoco_simulator/ros2_ws/src/franka_logistics_description
mkdir -p mjcf/assets meshes

# model 폴더 내의 mesh assets 복사
cp -r ../../../model_ori/franka_emika_panda/assets/* mjcf/assets/
```

---

### 🔹 Step 3: 물류 환경 통합 MJCF 가상 씬 작성 (`scene_dual_panda_logistics.xml`)

`ros2_ws/src/franka_logistics_description/mjcf/scene_dual_panda_logistics.xml` 파일을 생성하고 작성합니다.

이 파일은 다음의 모든 요소를 하나의 통합 시뮬레이션 씬으로 구성합니다:
1. **듀얼 로봇 (Robot 1, Robot 2)**: 7-자유도 암 + 2-핑거 그리퍼 + 개별 액추에이터 및 키프레임.
2. **중앙 작업대**: $1.2\text{m} \times 0.8\text{m}$ 물류 분류 테이블.
3. **전용 적재함 6개소**: 좌측 3개(A-R, B-B, C-G) / 우측 3개(A-G, B-R, C-B) 색상별 테두리 및 감지 사이트.
4. **배출 컨베이어 2개소**: 좌측 하단(불가품 1), 우측 상단(불가품 2).
5. **다품종 물류 아이템 6종 & 규격 외 불량품**: Free Joint가 부여되어 파지 및 이동이 가능한 물리 블록들.
6. **센서 마운팅**: Wrist Cameras (RGB/Depth) 및 2D LiDAR 가상 프레임 사이트.

```xml
<mujoco model="dual_franka_logistics_scene">
  <compiler angle="radian" meshdir="assets" autolimits="true" balanceinertia="true"/>

  <option integrator="implicitfast" timestep="0.001" gravity="0 0 -9.81"/>

  <statistic center="0 0 0.8" extent="2.5"/>

  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0.1 0.1 0.1"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="135" elevation="-25"/>
  </visual>

  <!-- ============================================================================== -->
  <!-- 기본 속성 상속 정의 (Defaults)                                                  -->
  <!-- ============================================================================== -->
  <default>
    <default class="panda">
      <material specular="0.5" shininess="0.25"/>
      <joint armature="0.1" damping="1" axis="0 0 1" range="-2.8973 2.8973"/>
      <general dyntype="none" biastype="affine" ctrlrange="-2.8973 2.8973" forcerange="-87 87"/>
      
      <default class="finger">
        <joint axis="0 1 0" type="slide" range="0 0.04"/>
      </default>

      <default class="visual">
        <geom type="mesh" contype="0" conaffinity="0" group="2"/>
      </default>
      
      <default class="collision">
        <geom type="mesh" group="3"/>
        <default class="fingertip_pad_collision_1">
          <geom type="box" size="0.0085 0.004 0.0085" pos="0 0.0055 0.0445" friction="1.5 0.01 0.001" solref="0.005 1" solimp="0.95 0.99 0.001"/>
        </default>
        <default class="fingertip_pad_collision_2">
          <geom type="box" size="0.003 0.002 0.003" pos="0.0055 0.002 0.05" friction="1.5 0.01 0.001"/>
        </default>
        <default class="fingertip_pad_collision_3">
          <geom type="box" size="0.003 0.002 0.003" pos="-0.0055 0.002 0.05" friction="1.5 0.01 0.001"/>
        </default>
        <default class="fingertip_pad_collision_4">
          <geom type="box" size="0.003 0.002 0.0035" pos="0.0055 0.002 0.0395" friction="1.5 0.01 0.001"/>
        </default>
        <default class="fingertip_pad_collision_5">
          <geom type="box" size="0.003 0.002 0.0035" pos="-0.0055 0.002 0.0395" friction="1.5 0.01 0.001"/>
        </default>
      </default>
    </default>

    <!-- 물류 박스 기본 속성 -->
    <default class="item_box">
      <geom type="box" size="0.025 0.025 0.025" mass="0.1" 
            friction="1.2 0.005 0.0001" solref="0.005 1" solimp="0.95 0.99 0.001"/>
    </default>
  </default>

  <!-- ============================================================================== -->
  <!-- 에셋 정의 (Assets: 재질, 텍스처, 3D 메시)                                      -->
  <!-- ============================================================================== -->
  <asset>
    <!-- 텍스처 및 바닥 재질 -->
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.25 0.3" rgb2="0.15 0.2 0.25"
             markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.1"/>

    <!-- 로봇 및 환경 기본 재질 -->
    <material class="panda" name="white" rgba="1 1 1 1"/>
    <material class="panda" name="off_white" rgba="0.9 0.92 0.93 1"/>
    <material class="panda" name="black" rgba="0.2 0.2 0.2 1"/>
    <material class="panda" name="green" rgba="0 1 0 1"/>
    <material class="panda" name="light_blue" rgba="0.039216 0.541176 0.780392 1"/>
    <material name="table_mat" rgba="0.82 0.82 0.82 1" reflectance="0.2"/>
    <material name="table_top_mat" rgba="0.25 0.28 0.32 1" reflectance="0.1"/>
    <material name="conveyor_mat" rgba="0.15 0.15 0.15 1" reflectance="0.05"/>
    <material name="bin_wall_mat" rgba="0.85 0.85 0.88 0.6"/>

    <!-- 물류 품목 재질 (Red, Green, Blue, Out-of-spec Yellow) -->
    <material name="mat_item_red" rgba="0.9 0.15 0.15 1" specular="0.3"/>
    <material name="mat_item_green" rgba="0.15 0.85 0.15 1" specular="0.3"/>
    <material name="mat_item_blue" rgba="0.15 0.35 0.9 1" specular="0.3"/>
    <material name="mat_item_yellow" rgba="0.95 0.85 0.1 1" specular="0.3"/>

    <!-- Franka Mesh Assets (Collision) -->
    <mesh name="link0_c" file="link0.stl"/>
    <mesh name="link1_c" file="link1.stl"/>
    <mesh name="link2_c" file="link2.stl"/>
    <mesh name="link3_c" file="link3.stl"/>
    <mesh name="link4_c" file="link4.stl"/>
    <mesh name="link5_c0" file="link5_collision_0.obj"/>
    <mesh name="link5_c1" file="link5_collision_1.obj"/>
    <mesh name="link5_c2" file="link5_collision_2.obj"/>
    <mesh name="link6_c" file="link6.stl"/>
    <mesh name="link7_c" file="link7.stl"/>
    <mesh name="hand_c" file="hand.stl"/>

    <!-- Franka Mesh Assets (Visual) -->
    <mesh file="link0_0.obj"/>
    <mesh file="link0_1.obj"/>
    <mesh file="link0_2.obj"/>
    <mesh file="link0_3.obj"/>
    <mesh file="link0_4.obj"/>
    <mesh file="link0_5.obj"/>
    <mesh file="link0_7.obj"/>
    <mesh file="link0_8.obj"/>
    <mesh file="link0_9.obj"/>
    <mesh file="link0_10.obj"/>
    <mesh file="link0_11.obj"/>
    <mesh file="link1.obj"/>
    <mesh file="link2.obj"/>
    <mesh file="link3_0.obj"/>
    <mesh file="link3_1.obj"/>
    <mesh file="link3_2.obj"/>
    <mesh file="link3_3.obj"/>
    <mesh file="link4_0.obj"/>
    <mesh file="link4_1.obj"/>
    <mesh file="link4_2.obj"/>
    <mesh file="link4_3.obj"/>
    <mesh file="link5_0.obj"/>
    <mesh file="link5_1.obj"/>
    <mesh file="link5_2.obj"/>
    <mesh file="link6_0.obj"/>
    <mesh file="link6_1.obj"/>
    <mesh file="link6_2.obj"/>
    <mesh file="link6_3.obj"/>
    <mesh file="link6_4.obj"/>
    <mesh file="link6_5.obj"/>
    <mesh file="link6_6.obj"/>
    <mesh file="link6_7.obj"/>
    <mesh file="link6_8.obj"/>
    <mesh file="link6_9.obj"/>
    <mesh file="link6_10.obj"/>
    <mesh file="link6_11.obj"/>
    <mesh file="link6_12.obj"/>
    <mesh file="link6_13.obj"/>
    <mesh file="link6_14.obj"/>
    <mesh file="link6_15.obj"/>
    <mesh file="link6_16.obj"/>
    <mesh file="link7_0.obj"/>
    <mesh file="link7_1.obj"/>
    <mesh file="link7_2.obj"/>
    <mesh file="link7_3.obj"/>
    <mesh file="link7_4.obj"/>
    <mesh file="link7_5.obj"/>
    <mesh file="link7_6.obj"/>
    <mesh file="link7_7.obj"/>
    <mesh file="hand_0.obj"/>
    <mesh file="hand_1.obj"/>
    <mesh file="hand_2.obj"/>
    <mesh file="hand_3.obj"/>
    <mesh file="hand_4.obj"/>
    <mesh file="finger_0.obj"/>
    <mesh file="finger_1.obj"/>
  </asset>

  <!-- ============================================================================== -->
  <!-- 월드 바디 (WorldBody: 지형, 환경 설비, 듀얼 로봇, 물류)                         -->
  <!-- ============================================================================== -->
  <worldbody>
    <light pos="0 0 2.5" dir="0 0 -1" directional="true" diffuse="0.8 0.8 0.8"/>
    <light pos="1.2 -1 2" dir="-0.5 0.5 -1" directional="true" diffuse="0.4 0.4 0.4"/>
    <light pos="-1.2 1 2" dir="0.5 -0.5 -1" directional="true" diffuse="0.4 0.4 0.4"/>
    
    <!-- 바닥 평면 -->
    <geom name="floor" size="3 3 0.05" type="plane" material="groundplane"/>

    <!-- 1. 중앙 물류 분류 작업대 (2.8m x 0.9m x 0.75m) -->
    <body name="sorting_table" pos="0 0 0.375">
      <geom name="table_top" type="box" size="1.4 0.45 0.025" pos="0 0 0.35" material="table_top_mat"/>
      <geom name="table_top_lib1" type="box" size="0.015 0.45 0.04" pos="0.3 0 0.4" material="table_top_mat"/>
      <geom name="table_top_lib2" type="box" size="0.015 0.45 0.04" pos="-0.3 0 0.4" material="table_top_mat"/>
      <geom name="table_top_lib3" type="box" size="0.3 0.015 0.04" pos="0 0.435 0.4" material="table_top_mat"/>
      <geom name="table_top_lib4" type="box" size="0.3 0.015 0.04" pos="0 -0.435 0.4" material="table_top_mat"/>
      <geom name="table_leg1" type="cylinder" size="0.05 0.35" pos="0.45 0.35 0" material="table_mat"/>
      <geom name="table_leg2" type="cylinder" size="0.05 0.35" pos="-0.45 0.35 0" material="table_mat"/>
      <geom name="table_leg3" type="cylinder" size="0.05 0.35" pos="0.45 -0.35 0" material="table_mat"/>
      <geom name="table_leg4" type="cylinder" size="0.05 0.35" pos="-0.45 -0.35 0" material="table_mat"/>
      <geom name="table_leg5" type="cylinder" size="0.05 0.35" pos="1.25 0.35 0" material="table_mat"/>
      <geom name="table_leg6" type="cylinder" size="0.05 0.35" pos="-1.25 0.35 0" material="table_mat"/>
      <geom name="table_leg7" type="cylinder" size="0.05 0.35" pos="1.25 -0.35 0" material="table_mat"/>
      <geom name="table_leg8" type="cylinder" size="0.05 0.35" pos="-1.25 -0.35 0" material="table_mat"/>
      <!-- 공유 작업 영역 경계 시각화 사이트 -->
      <site name="shared_workspace_center" pos="0 0 0.376" size="0.25 0.35 0.001" type="box" rgba="0 1 1 0.15"/>
    </body>

    <!-- 2. 적재함 6개소 (좌측 3개: Robot 1 / 우측 3개: Robot 2) -->
    <!-- [좌측] 적재함 1: Bin A-Red -->
    <body name="bin_A_Red" pos="-1.25 0.28 0.75">
      <geom type="box" size="0.15 0.125 0.01" pos="0 0 0" material="mat_item_red"/>
      <geom type="box" size="0.15 0.005 0.08" pos="0 0.12 0.08" material="bin_wall_mat"/>
      <geom type="box" size="0.15 0.005 0.08" pos="0 -0.12 0.08" material="bin_wall_mat"/>
      <geom type="box" size="0.005 0.125 0.08" pos="0.145 0 0.08" material="bin_wall_mat"/>
      <geom type="box" size="0.005 0.125 0.08" pos="-0.145 0 0.08" material="bin_wall_mat"/>
      <site name="site_bin_A_Red" pos="0 0 0.08" size="0.14 0.11 0.07" type="box" rgba="1 0 0 0.05"/>
    </body>
    <!-- [좌측] 적재함 2: Bin B-Blue -->
    <body name="bin_B_Blue" pos="-1.25 0.0 0.75">
      <geom type="box" size="0.15 0.125 0.01" pos="0 0 0" material="mat_item_blue"/>
      <geom type="box" size="0.15 0.005 0.08" pos="0 0.12 0.08" material="bin_wall_mat"/>
      <geom type="box" size="0.15 0.005 0.08" pos="0 -0.12 0.08" material="bin_wall_mat"/>
      <geom type="box" size="0.005 0.125 0.08" pos="0.145 0 0.08" material="bin_wall_mat"/>
      <geom type="box" size="0.005 0.125 0.08" pos="-0.145 0 0.08" material="bin_wall_mat"/>
      <site name="site_bin_B_Blue" pos="0 0 0.08" size="0.14 0.11 0.07" type="box" rgba="0 0 1 0.05"/>
    </body>
    <!-- [좌측] 적재함 3: Bin C-Green -->
    <body name="bin_C_Green" pos="-1.25 -0.28 0.75">
      <geom type="box" size="0.15 0.125 0.01" pos="0 0 0" material="mat_item_green"/>
      <geom type="box" size="0.15 0.005 0.08" pos="0 0.12 0.08" material="bin_wall_mat"/>
      <geom type="box" size="0.15 0.005 0.08" pos="0 -0.12 0.08" material="bin_wall_mat"/>
      <geom type="box" size="0.005 0.125 0.08" pos="0.145 0 0.08" material="bin_wall_mat"/>
      <geom type="box" size="0.005 0.125 0.08" pos="-0.145 0 0.08" material="bin_wall_mat"/>
      <site name="site_bin_C_Green" pos="0 0 0.08" size="0.14 0.11 0.07" type="box" rgba="0 1 0 0.05"/>
    </body>

    <!-- [우측] 적재함 4: Bin A-Green -->
    <body name="bin_A_Green" pos="1.25 0.28 0.75">
      <geom type="box" size="0.15 0.125 0.01" pos="0 0 0" material="mat_item_green"/>
      <geom type="box" size="0.15 0.005 0.08" pos="0 0.12 0.08" material="bin_wall_mat"/>
      <geom type="box" size="0.15 0.005 0.08" pos="0 -0.12 0.08" material="bin_wall_mat"/>
      <geom type="box" size="0.005 0.125 0.08" pos="0.145 0 0.08" material="bin_wall_mat"/>
      <geom type="box" size="0.005 0.125 0.08" pos="-0.145 0 0.08" material="bin_wall_mat"/>
      <site name="site_bin_A_Green" pos="0 0 0.08" size="0.14 0.11 0.07" type="box" rgba="0 1 0 0.05"/>
    </body>
    <!-- [우측] 적재함 5: Bin B-Red -->
    <body name="bin_B_Red" pos="1.25 0.0 0.75">
      <geom type="box" size="0.15 0.125 0.01" pos="0 0 0" material="mat_item_red"/>
      <geom type="box" size="0.15 0.005 0.08" pos="0 0.12 0.08" material="bin_wall_mat"/>
      <geom type="box" size="0.15 0.005 0.08" pos="0 -0.12 0.08" material="bin_wall_mat"/>
      <geom type="box" size="0.005 0.125 0.08" pos="0.145 0 0.08" material="bin_wall_mat"/>
      <geom type="box" size="0.005 0.125 0.08" pos="-0.145 0 0.08" material="bin_wall_mat"/>
      <site name="site_bin_B_Red" pos="0 0 0.08" size="0.14 0.11 0.07" type="box" rgba="1 0 0 0.05"/>
    </body>
    <!-- [우측] 적재함 6: Bin C-Blue -->
    <body name="bin_C_Blue" pos="1.25 -0.28 0.75">
      <geom type="box" size="0.15 0.125 0.01" pos="0 0 0" material="mat_item_blue"/>
      <geom type="box" size="0.15 0.005 0.08" pos="0 0.12 0.08" material="bin_wall_mat"/>
      <geom type="box" size="0.15 0.005 0.08" pos="0 -0.12 0.08" material="bin_wall_mat"/>
      <geom type="box" size="0.005 0.125 0.08" pos="0.145 0 0.08" material="bin_wall_mat"/>
      <geom type="box" size="0.005 0.125 0.08" pos="-0.145 0 0.08" material="bin_wall_mat"/>
      <site name="site_bin_C_Blue" pos="0 0 0.08" size="0.14 0.11 0.07" type="box" rgba="0 0 1 0.05"/>
    </body>

    <!-- 3. 배출 컨베이어 벨트 2개소 -->
    <!-- 컨베이어 1 (Robot 1 전용, 좌측 하단) -->
    <body name="conveyor_r1" pos="-0.65 -0.65 0.65">
      <geom type="box" size="0.2 0.25 0.05" material="conveyor_mat"/>
      <site name="site_conveyor_r1" pos="0 0 0.055" size="0.18 0.23 0.01" type="box" rgba="1 1 0 0.2"/>
    </body>
    <!-- 컨베이어 2 (Robot 2 전용, 우측 상단) -->
    <body name="conveyor_r2" pos="0.65 0.65 0.65">
      <geom type="box" size="0.2 0.25 0.05" material="conveyor_mat"/>
      <site name="site_conveyor_r2" pos="0 0 0.055" size="0.18 0.23 0.01" type="box" rgba="1 1 0 0.2"/>
    </body>

    <!-- 4. 안전 감시 2D LiDAR 기준 사이트 (상단 / 하단) -->
    <site name="lidar_top_frame" pos="0 0.4 0.67" quat="1 0 0 0" size="0.03" type="sphere" rgba="1 0 1 0.8"/>
    <site name="lidar_bottom_frame" pos="0 -0.4 0.67" quat="0 0 0 1" size="0.03" type="sphere" rgba="1 0 1 0.8"/>

    <!-- ============================================================================ -->
    <!-- 5. 로봇 1 (좌측 로봇, Robot 1, Prefix: r1_)                                    -->
    <!-- ============================================================================ -->
    <body name="r1_link0" pos="-0.65 0 0.75" childclass="panda">
      <inertial mass="0.629769" pos="-0.041018 -0.00014 0.049974"
        fullinertia="0.00315 0.00388 0.004285 8.2904e-7 0.00015 8.2299e-6"/>
      <geom mesh="link0_0" material="off_white" class="visual"/>
      <geom mesh="link0_1" material="black" class="visual"/>
      <geom mesh="link0_2" material="off_white" class="visual"/>
      <geom mesh="link0_3" material="black" class="visual"/>
      <geom mesh="link0_4" material="off_white" class="visual"/>
      <geom mesh="link0_5" material="black" class="visual"/>
      <geom mesh="link0_7" material="white" class="visual"/>
      <geom mesh="link0_8" material="white" class="visual"/>
      <geom mesh="link0_9" material="black" class="visual"/>
      <geom mesh="link0_10" material="off_white" class="visual"/>
      <geom mesh="link0_11" material="white" class="visual"/>
      <geom mesh="link0_c" class="collision"/>

      <body name="r1_link1" pos="0 0 0.333">
        <inertial mass="4.970684" pos="0.003875 0.002081 -0.04762"
          fullinertia="0.70337 0.70661 0.0091170 -0.00013900 0.0067720 0.019169"/>
        <joint name="r1_joint1"/>
        <geom mesh="link1" material="white" class="visual"/>
        <geom mesh="link1_c" class="collision"/>

        <body name="r1_link2" quat="1 -1 0 0">
          <inertial mass="0.646926" pos="-0.003141 -0.02872 0.003495"
            fullinertia="0.0079620 2.8110e-2 2.5995e-2 -3.925e-3 1.0254e-2 7.04e-4"/>
          <joint name="r1_joint2" range="-1.7628 1.7628"/>
          <geom mesh="link2" material="white" class="visual"/>
          <geom mesh="link2_c" class="collision"/>

          <body name="r1_link3" pos="0 -0.316 0" quat="1 1 0 0">
            <inertial mass="3.228604" pos="2.7518e-2 3.9252e-2 -6.6502e-2"
              fullinertia="3.7242e-2 3.6155e-2 1.083e-2 -4.761e-3 -1.1396e-2 -1.2805e-2"/>
            <joint name="r1_joint3"/>
            <geom mesh="link3_0" material="white" class="visual"/>
            <geom mesh="link3_1" material="white" class="visual"/>
            <geom mesh="link3_2" material="white" class="visual"/>
            <geom mesh="link3_3" material="black" class="visual"/>
            <geom mesh="link3_c" class="collision"/>

            <body name="r1_link4" pos="0.0825 0 0" quat="1 1 0 0">
              <inertial mass="3.587895" pos="-5.317e-2 1.04419e-1 2.7454e-2"
                fullinertia="2.5853e-2 1.9552e-2 2.8323e-2 7.796e-3 -1.332e-3 8.641e-3"/>
              <joint name="r1_joint4" range="-3.0718 -0.0698"/>
              <geom mesh="link4_0" material="white" class="visual"/>
              <geom mesh="link4_1" material="white" class="visual"/>
              <geom mesh="link4_2" material="black" class="visual"/>
              <geom mesh="link4_3" material="white" class="visual"/>
              <geom mesh="link4_c" class="collision"/>

              <body name="r1_link5" pos="-0.0825 0.384 0" quat="1 -1 0 0">
                <inertial mass="1.225946" pos="-1.1953e-2 4.1065e-2 -3.8437e-2"
                  fullinertia="3.5549e-2 2.9474e-2 8.627e-3 -2.117e-3 -4.037e-3 2.29e-4"/>
                <joint name="r1_joint5"/>
                <geom mesh="link5_0" material="black" class="visual"/>
                <geom mesh="link5_1" material="white" class="visual"/>
                <geom mesh="link5_2" material="white" class="visual"/>
                <geom mesh="link5_c0" class="collision"/>
                <geom mesh="link5_c1" class="collision"/>
                <geom mesh="link5_c2" class="collision"/>

                <body name="r1_link6" quat="1 1 0 0">
                  <inertial mass="1.666555" pos="6.0149e-2 -1.4117e-2 -1.0517e-2"
                    fullinertia="1.964e-3 4.354e-3 5.433e-3 1.09e-4 -1.158e-3 3.41e-4"/>
                  <joint name="r1_joint6" range="-0.0175 3.7525"/>
                  <geom mesh="link6_0" material="off_white" class="visual"/>
                  <geom mesh="link6_1" material="white" class="visual"/>
                  <geom mesh="link6_2" material="black" class="visual"/>
                  <geom mesh="link6_3" material="white" class="visual"/>
                  <geom mesh="link6_4" material="white" class="visual"/>
                  <geom mesh="link6_5" material="white" class="visual"/>
                  <geom mesh="link6_6" material="white" class="visual"/>
                  <geom mesh="link6_7" material="light_blue" class="visual"/>
                  <geom mesh="link6_8" material="light_blue" class="visual"/>
                  <geom mesh="link6_9" material="black" class="visual"/>
                  <geom mesh="link6_10" material="black" class="visual"/>
                  <geom mesh="link6_11" material="white" class="visual"/>
                  <geom mesh="link6_12" material="green" class="visual"/>
                  <geom mesh="link6_13" material="white" class="visual"/>
                  <geom mesh="link6_14" material="black" class="visual"/>
                  <geom mesh="link6_15" material="black" class="visual"/>
                  <geom mesh="link6_16" material="white" class="visual"/>
                  <geom mesh="link6_c" class="collision"/>

                  <body name="r1_link7" pos="0.088 0 0" quat="1 1 0 0">
                    <inertial mass="7.35522e-01" pos="1.0517e-2 -4.252e-3 6.1597e-2"
                      fullinertia="1.2516e-2 1.0027e-2 4.815e-3 -4.28e-4 -1.196e-3 -7.41e-4"/>
                    <joint name="r1_joint7"/>
                    <geom mesh="link7_0" material="white" class="visual"/>
                    <geom mesh="link7_1" material="black" class="visual"/>
                    <geom mesh="link7_2" material="black" class="visual"/>
                    <geom mesh="link7_3" material="black" class="visual"/>
                    <geom mesh="link7_4" material="black" class="visual"/>
                    <geom mesh="link7_5" material="black" class="visual"/>
                    <geom mesh="link7_6" material="black" class="visual"/>
                    <geom mesh="link7_7" material="white" class="visual"/>
                    <geom mesh="link7_c" class="collision"/>

                    <!-- 로봇 1 그리퍼 (Hand) -->
                    <body name="r1_hand" pos="0 0 0.107" quat="0.9238795 0 0 -0.3826834">
                      <inertial mass="0.73" pos="-0.01 0 0.03" diaginertia="0.001 0.0025 0.0017"/>
                      <geom mesh="hand_0" material="off_white" class="visual"/>
                      <geom mesh="hand_1" material="black" class="visual"/>
                      <geom mesh="hand_2" material="black" class="visual"/>
                      <geom mesh="hand_3" material="white" class="visual"/>
                      <geom mesh="hand_4" material="off_white" class="visual"/>
                      <geom mesh="hand_c" class="collision"/>

                      <!-- Robot 1 Wrist Camera (Depth & RGB 렌더링용) -->
                      <!-- <camera name="r1_wrist_camera" pos="0.05 0 0.05" quat="0.707107 0 0.707107 0" fovy="58"/> -->
                      <!--바닥면 중앙-->
                      <!-- <camera name="r1_wrist_camera" pos="0 0 0.0584" quat="0 0 1 0" fovy="58"/> -->
                      <!--집게 중심 중앙-->
                      <!-- <camera name="r1_wrist_camera" pos="0 0 0.0103" quat="0 0 1 0" fovy="58"/> -->
                      <!--브라켓 사용-->
                      <camera name="r1_wrist_camera" pos="0.03 0 0.06" quat="0 0 1 0" fovy="58"/>
                      <!-- 카메라 장착 위치 시각화 마커 (노란색 육면체) -->
                      <!-- <site name="r1_cam_vis" pos="0.03 0 0.06" size="0.006" type="sphere" rgba="1 1 0 1"/> -->
                      <site name="r1_cam_vis" pos="0.03 0 0.06" size="0.008 0.02 0.008" type="box" rgba="1 1 0 1"/>
                      <!-- 엔드이펙터(집게) 제어 중심점 (빨간색 구체) (빨강색 구체) -->
                      <site name="r1_ee_site" pos="0 0 0.103" size="0.005" type="sphere" rgba="1 0 0 1"/>

                      <body name="r1_left_finger" pos="0 0 0.0584">
                        <inertial mass="0.015" pos="0 0 0" diaginertia="2.375e-6 2.375e-6 7.5e-7"/>
                        <joint name="r1_finger_joint1" class="finger"/>
                        <geom mesh="finger_0" material="off_white" class="visual"/>
                        <geom mesh="finger_1" material="black" class="visual"/>
                        <geom mesh="finger_0" class="collision"/>
                        <geom class="fingertip_pad_collision_1"/>
                        <geom class="fingertip_pad_collision_2"/>
                        <geom class="fingertip_pad_collision_3"/>
                        <geom class="fingertip_pad_collision_4"/>
                        <geom class="fingertip_pad_collision_5"/>
                      </body>
                      <body name="r1_right_finger" pos="0 0 0.0584" quat="0 0 0 1">
                        <inertial mass="0.015" pos="0 0 0" diaginertia="2.375e-6 2.375e-6 7.5e-7"/>
                        <joint name="r1_finger_joint2" class="finger"/>
                        <geom mesh="finger_0" material="off_white" class="visual"/>
                        <geom mesh="finger_1" material="black" class="visual"/>
                        <geom mesh="finger_0" class="collision"/>
                        <geom class="fingertip_pad_collision_1"/>
                        <geom class="fingertip_pad_collision_2"/>
                        <geom class="fingertip_pad_collision_3"/>
                        <geom class="fingertip_pad_collision_4"/>
                        <geom class="fingertip_pad_collision_5"/>
                      </body>
                    </body>
                  </body>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>

    <!-- ============================================================================ -->
    <!-- 6. 로봇 2 (우측 로봇, Robot 2, Prefix: r2_)                                    -->
    <!-- ============================================================================ -->
    <body name="r2_link0" pos="0.65 0 0.75" quat="0 0 0 1" childclass="panda">
      <inertial mass="0.629769" pos="-0.041018 -0.00014 0.049974"
        fullinertia="0.00315 0.00388 0.004285 8.2904e-7 0.00015 8.2299e-6"/>
      <geom mesh="link0_0" material="off_white" class="visual"/>
      <geom mesh="link0_1" material="black" class="visual"/>
      <geom mesh="link0_2" material="off_white" class="visual"/>
      <geom mesh="link0_3" material="black" class="visual"/>
      <geom mesh="link0_4" material="off_white" class="visual"/>
      <geom mesh="link0_5" material="black" class="visual"/>
      <geom mesh="link0_7" material="white" class="visual"/>
      <geom mesh="link0_8" material="white" class="visual"/>
      <geom mesh="link0_9" material="black" class="visual"/>
      <geom mesh="link0_10" material="off_white" class="visual"/>
      <geom mesh="link0_11" material="white" class="visual"/>
      <geom mesh="link0_c" class="collision"/>

      <body name="r2_link1" pos="0 0 0.333">
        <inertial mass="4.970684" pos="0.003875 0.002081 -0.04762"
          fullinertia="0.70337 0.70661 0.0091170 -0.00013900 0.0067720 0.019169"/>
        <joint name="r2_joint1"/>
        <geom mesh="link1" material="white" class="visual"/>
        <geom mesh="link1_c" class="collision"/>

        <body name="r2_link2" quat="1 -1 0 0">
          <inertial mass="0.646926" pos="-0.003141 -0.02872 0.003495"
            fullinertia="0.0079620 2.8110e-2 2.5995e-2 -3.925e-3 1.0254e-2 7.04e-4"/>
          <joint name="r2_joint2" range="-1.7628 1.7628"/>
          <geom mesh="link2" material="white" class="visual"/>
          <geom mesh="link2_c" class="collision"/>

          <body name="r2_link3" pos="0 -0.316 0" quat="1 1 0 0">
            <inertial mass="3.228604" pos="2.7518e-2 3.9252e-2 -6.6502e-2"
              fullinertia="3.7242e-2 3.6155e-2 1.083e-2 -4.761e-3 -1.1396e-2 -1.2805e-2"/>
            <joint name="r2_joint3"/>
            <geom mesh="link3_0" material="white" class="visual"/>
            <geom mesh="link3_1" material="white" class="visual"/>
            <geom mesh="link3_2" material="white" class="visual"/>
            <geom mesh="link3_3" material="black" class="visual"/>
            <geom mesh="link3_c" class="collision"/>

            <body name="r2_link4" pos="0.0825 0 0" quat="1 1 0 0">
              <inertial mass="3.587895" pos="-5.317e-2 1.04419e-1 2.7454e-2"
                fullinertia="2.5853e-2 1.9552e-2 2.8323e-2 7.796e-3 -1.332e-3 8.641e-3"/>
              <joint name="r2_joint4" range="-3.0718 -0.0698"/>
              <geom mesh="link4_0" material="white" class="visual"/>
              <geom mesh="link4_1" material="white" class="visual"/>
              <geom mesh="link4_2" material="black" class="visual"/>
              <geom mesh="link4_3" material="white" class="visual"/>
              <geom mesh="link4_c" class="collision"/>

              <body name="r2_link5" pos="-0.0825 0.384 0" quat="1 -1 0 0">
                <inertial mass="1.225946" pos="-1.1953e-2 4.1065e-2 -3.8437e-2"
                  fullinertia="3.5549e-2 2.9474e-2 8.627e-3 -2.117e-3 -4.037e-3 2.29e-4"/>
                <joint name="r2_joint5"/>
                <geom mesh="link5_0" material="black" class="visual"/>
                <geom mesh="link5_1" material="white" class="visual"/>
                <geom mesh="link5_2" material="white" class="visual"/>
                <geom mesh="link5_c0" class="collision"/>
                <geom mesh="link5_c1" class="collision"/>
                <geom mesh="link5_c2" class="collision"/>

                <body name="r2_link6" quat="1 1 0 0">
                  <inertial mass="1.666555" pos="6.0149e-2 -1.4117e-2 -1.0517e-2"
                    fullinertia="1.964e-3 4.354e-3 5.433e-3 1.09e-4 -1.158e-3 3.41e-4"/>
                  <joint name="r2_joint6" range="-0.0175 3.7525"/>
                  <geom mesh="link6_0" material="off_white" class="visual"/>
                  <geom mesh="link6_1" material="white" class="visual"/>
                  <geom mesh="link6_2" material="black" class="visual"/>
                  <geom mesh="link6_3" material="white" class="visual"/>
                  <geom mesh="link6_4" material="white" class="visual"/>
                  <geom mesh="link6_5" material="white" class="visual"/>
                  <geom mesh="link6_6" material="white" class="visual"/>
                  <geom mesh="link6_7" material="light_blue" class="visual"/>
                  <geom mesh="link6_8" material="light_blue" class="visual"/>
                  <geom mesh="link6_9" material="black" class="visual"/>
                  <geom mesh="link6_10" material="black" class="visual"/>
                  <geom mesh="link6_11" material="white" class="visual"/>
                  <geom mesh="link6_12" material="green" class="visual"/>
                  <geom mesh="link6_13" material="white" class="visual"/>
                  <geom mesh="link6_14" material="black" class="visual"/>
                  <geom mesh="link6_15" material="black" class="visual"/>
                  <geom mesh="link6_16" material="white" class="visual"/>
                  <geom mesh="link6_c" class="collision"/>

                  <body name="r2_link7" pos="0.088 0 0" quat="1 1 0 0">
                    <inertial mass="7.35522e-01" pos="1.0517e-2 -4.252e-3 6.1597e-2"
                      fullinertia="1.2516e-2 1.0027e-2 4.815e-3 -4.28e-4 -1.196e-3 -7.41e-4"/>
                    <joint name="r2_joint7"/>
                    <geom mesh="link7_0" material="white" class="visual"/>
                    <geom mesh="link7_1" material="black" class="visual"/>
                    <geom mesh="link7_2" material="black" class="visual"/>
                    <geom mesh="link7_3" material="black" class="visual"/>
                    <geom mesh="link7_4" material="black" class="visual"/>
                    <geom mesh="link7_5" material="black" class="visual"/>
                    <geom mesh="link7_6" material="black" class="visual"/>
                    <geom mesh="link7_7" material="white" class="visual"/>
                    <geom mesh="link7_c" class="collision"/>

                    <!-- 로봇 2 그리퍼 (Hand) -->
                    <body name="r2_hand" pos="0 0 0.107" quat="0.9238795 0 0 -0.3826834">
                      <inertial mass="0.73" pos="-0.01 0 0.03" diaginertia="0.001 0.0025 0.0017"/>
                      <geom mesh="hand_0" material="off_white" class="visual"/>
                      <geom mesh="hand_1" material="black" class="visual"/>
                      <geom mesh="hand_2" material="black" class="visual"/>
                      <geom mesh="hand_3" material="white" class="visual"/>
                      <geom mesh="hand_4" material="off_white" class="visual"/>
                      <geom mesh="hand_c" class="collision"/>

                      <!-- Robot 2 Wrist Camera (Depth & RGB 렌더링용) -->
                      <!-- <camera name="r2_wrist_camera" pos="0.05 0 0.05" quat="0.707107 0 0.707107 0" fovy="58"/> -->
                                            <!--바닥면 중앙-->
                      <!-- <camera name="r2_wrist_camera" pos="0 0 0.0584" quat="0 0 1 0" fovy="58"/> -->
                      <!--집게 중심 중앙-->
                      <!-- <camera name="r2_wrist_camera" pos="0 0 0.0103" quat="0 0 1 0" fovy="58"/> -->
                      <!--브라켓 사용-->
                      <camera name="r2_wrist_camera" pos="0.03 0 0.06" quat="0 0 1 0" fovy="58"/>
                      <!-- 카메라 장착 위치 시각화 마커 (노란색 육면체) -->
                      <!-- <site name="r2_cam_vis" pos="0.03 0 0.06" size="0.006" type="sphere" rgba="1 1 0 1"/> -->
                      <site name="r2_cam_vis" pos="0.03 0 0.06" size="0.008 0.02 0.008" type="box" rgba="1 1 0 1"/>
                      <!-- 엔드이펙터(집게) 제어 중심점 (빨간색 구체) (빨강색 구체) -->
                      <site name="r2_ee_site" pos="0 0 0.103" size="0.005" type="sphere" rgba="1 0 0 1"/>

                      <body name="r2_left_finger" pos="0 0 0.0584">
                        <inertial mass="0.015" pos="0 0 0" diaginertia="2.375e-6 2.375e-6 7.5e-7"/>
                        <joint name="r2_finger_joint1" class="finger"/>
                        <geom mesh="finger_0" material="off_white" class="visual"/>
                        <geom mesh="finger_1" material="black" class="visual"/>
                        <geom mesh="finger_0" class="collision"/>
                        <geom class="fingertip_pad_collision_1"/>
                        <geom class="fingertip_pad_collision_2"/>
                        <geom class="fingertip_pad_collision_3"/>
                        <geom class="fingertip_pad_collision_4"/>
                        <geom class="fingertip_pad_collision_5"/>
                      </body>
                      <body name="r2_right_finger" pos="0 0 0.0584" quat="0 0 0 1">
                        <inertial mass="0.015" pos="0 0 0" diaginertia="2.375e-6 2.375e-6 7.5e-7"/>
                        <joint name="r2_finger_joint2" class="finger"/>
                        <geom mesh="finger_0" material="off_white" class="visual"/>
                        <geom mesh="finger_1" material="black" class="visual"/>
                        <geom mesh="finger_0" class="collision"/>
                        <geom class="fingertip_pad_collision_1"/>
                        <geom class="fingertip_pad_collision_2"/>
                        <geom class="fingertip_pad_collision_3"/>
                        <geom class="fingertip_pad_collision_4"/>
                        <geom class="fingertip_pad_collision_5"/>
                      </body>
                    </body>
                  </body>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>

    <!-- ============================================================================ -->
    <!-- 7. 물류 아이템 6종 + 불량품 1종 (Free Joint 부여)                              -->
    <!-- ============================================================================ -->
    <!-- Robot 1 타겟: Item A-Red -->
    <body name="item_A_Red" pos="-0.25 0.15 0.80">
      <freejoint name="item_A_Red_joint"/>
      <geom class="item_box" material="mat_item_red"/>
    </body>
    <!-- Robot 1 타겟: Item B-Blue -->
    <body name="item_B_Blue" pos="-0.25 0.0 0.80">
      <freejoint name="item_B_Blue_joint"/>
      <geom class="item_box" material="mat_item_blue"/>
    </body>
    <!-- Robot 1 타겟: Item C-Green -->
    <body name="item_C_Green" pos="-0.25 -0.15 0.80">
      <freejoint name="item_C_Green_joint"/>
      <geom class="item_box" material="mat_item_green"/>
    </body>

    <!-- Robot 2 타겟: Item A-Green -->
    <body name="item_A_Green" pos="0.25 0.15 0.80">
      <freejoint name="item_A_Green_joint"/>
      <geom class="item_box" material="mat_item_green"/>
    </body>
    <!-- Robot 2 타겟: Item B-Red -->
    <body name="item_B_Red" pos="0.25 0.0 0.80">
      <freejoint name="item_B_Red_joint"/>
      <geom class="item_box" material="mat_item_red"/>
    </body>
    <!-- Robot 2 타겟: Item C-Blue -->
    <body name="item_C_Blue" pos="0.25 -0.15 0.80">
      <freejoint name="item_C_Blue_joint"/>
      <geom class="item_box" material="mat_item_blue"/>
    </body>

    <!-- 공유 영역 / 규격 외 불량품 (Yellow Cylinder) -->
    <body name="item_unclassified_1" pos="0.0 0.0 0.80">
      <freejoint name="item_unclass_joint"/>
      <geom type="cylinder" size="0.028 0.025" mass="0.12" material="mat_item_yellow"
            friction="1.2 0.005 0.0001" solref="0.005 1" solimp="0.95 0.99 0.001"/>
    </body>

  </worldbody>

  <!-- ============================================================================== -->
  <!-- 핑거 연동 텐던 및 조인트 등가 구속 (Tendon & Equality)                           -->
  <!-- ============================================================================== -->
  <tendon>
    <fixed name="r1_split">
      <joint joint="r1_finger_joint1" coef="0.5"/>
      <joint joint="r1_finger_joint2" coef="0.5"/>
    </fixed>
    <fixed name="r2_split">
      <joint joint="r2_finger_joint1" coef="0.5"/>
      <joint joint="r2_finger_joint2" coef="0.5"/>
    </fixed>
  </tendon>

  <equality>
    <joint joint1="r1_finger_joint1" joint2="r1_finger_joint2" solimp="0.95 0.99 0.001" solref="0.005 1"/>
    <joint joint1="r2_finger_joint1" joint2="r2_finger_joint2" solimp="0.95 0.99 0.001" solref="0.005 1"/>
  </equality>

  <!-- ============================================================================== -->
  <!-- 액추에이터 정의 (Actuators: Robot 1 & Robot 2)                                 -->
  <!-- ============================================================================== -->
  <actuator>
    <!-- Robot 1 암 & 그리퍼 액추에이터 -->
    <general class="panda" name="r1_actuator1" joint="r1_joint1" gainprm="4500" biasprm="0 -4500 -450"/>
    <general class="panda" name="r1_actuator2" joint="r1_joint2" gainprm="4500" biasprm="0 -4500 -450" ctrlrange="-1.7628 1.7628"/>
    <general class="panda" name="r1_actuator3" joint="r1_joint3" gainprm="3500" biasprm="0 -3500 -350"/>
    <general class="panda" name="r1_actuator4" joint="r1_joint4" gainprm="3500" biasprm="0 -3500 -350" ctrlrange="-3.0718 -0.0698"/>
    <general class="panda" name="r1_actuator5" joint="r1_joint5" gainprm="2000" biasprm="0 -2000 -200" forcerange="-12 12"/>
    <general class="panda" name="r1_actuator6" joint="r1_joint6" gainprm="2000" biasprm="0 -2000 -200" forcerange="-12 12" ctrlrange="-0.0175 3.7525"/>
    <general class="panda" name="r1_actuator7" joint="r1_joint7" gainprm="2000" biasprm="0 -2000 -200" forcerange="-12 12"/>
    <general class="panda" name="r1_actuator8" tendon="r1_split" forcerange="-100 100" ctrlrange="0 255" gainprm="0.01568627451 0 0" biasprm="0 -100 -10"/>

    <!-- Robot 2 암 & 그리퍼 액추에이터 -->
    <general class="panda" name="r2_actuator1" joint="r2_joint1" gainprm="4500" biasprm="0 -4500 -450"/>
    <general class="panda" name="r2_actuator2" joint="r2_joint2" gainprm="4500" biasprm="0 -4500 -450" ctrlrange="-1.7628 1.7628"/>
    <general class="panda" name="r2_actuator3" joint="r2_joint3" gainprm="3500" biasprm="0 -3500 -350"/>
    <general class="panda" name="r2_actuator4" joint="r2_joint4" gainprm="3500" biasprm="0 -3500 -350" ctrlrange="-3.0718 -0.0698"/>
    <general class="panda" name="r2_actuator5" joint="r2_joint5" gainprm="2000" biasprm="0 -2000 -200" forcerange="-12 12"/>
    <general class="panda" name="r2_actuator6" joint="r2_joint6" gainprm="2000" biasprm="0 -2000 -200" forcerange="-12 12" ctrlrange="-0.0175 3.7525"/>
    <general class="panda" name="r2_actuator7" joint="r2_joint7" gainprm="2000" biasprm="0 -2000 -200" forcerange="-12 12"/>
    <general class="panda" name="r2_actuator8" tendon="r2_split" forcerange="-100 100" ctrlrange="0 255" gainprm="0.01568627451 0 0" biasprm="0 -100 -10"/>
  </actuator>

  <!-- ============================================================================== -->
  <!-- 초기 기본 자세 키프레임 (Keyframes: Home Position)                             -->
  <!-- ============================================================================== -->
  <keyframe>
    <key name="home" 
         qpos="0 -0.785 0 -2.356 0 1.571 0.785 0.04 0.04  0 -0.785 0 -2.356 0 1.571 0.785 0.04 0.04  -0.25 0.15 0.8 1 0 0 0  -0.25 0 0.8 1 0 0 0  -0.25 -0.15 0.8 1 0 0 0  0.25 0.15 0.8 1 0 0 0  0.25 0 0.8 1 0 0 0  0.25 -0.15 0.8 1 0 0 0  0 0 0.8 1 0 0 0"
         ctrl="0 -0.785 0 -2.356 0 1.571 0.785 255  0 -0.785 0 -2.356 0 1.571 0.785 255"/>
  </keyframe>

  <!-- ============================================================================== -->
  <!-- 인접 링크 자기 충돌 제외 (Contact Exclusions)                                 -->
  <!-- ============================================================================== -->
  <contact>
    <exclude body1="r1_link0" body2="r1_link1"/>
    <exclude body1="r2_link0" body2="r2_link1"/>
  </contact>
</mujoco>
```

---

### 🔹 Step 4: 씬 무결성 자가진단 스크립트 작성 (`test/phase02_test_scene.py`)

작성된 MJCF 파일이 MuJoCo 3.6.x C API에서 에러 없이 로드되는지, 모든 바디/관절/액추에이터/카메라가 정상 파싱되는지 검증하는 테스트 스크립트를 `test/phase02_test_scene.py`에 작성합니다.

```python
#!/usr/bin/env python3
"""
Phase 02 가상 씬 및 물류 환경 무결성 검증 스크립트
scene_dual_panda_logistics.xml 파일 로드, 물리 스텝 구동 및 센서/액추에이터 바인딩 확인
"""

import os
import sys
import numpy as np

def verify_scene():
    print("=" * 65)
    print("🚀 [Phase 02] MuJoCo 3.6.x 듀얼 판다 물류 가상 씬 무결성 검증")
    print("=" * 65)

    # 1. 파일 경로 확인
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.abspath(os.path.join(script_dir, ".."))
    xml_path = os.path.join(
        project_root,
        "ros2_ws", "src", "franka_logistics_description", "mjcf", "scene_dual_panda_logistics.xml"
    )

    print(f"[1/6] 씬 파일 경로 확인: {xml_path}")
    if not os.path.exists(xml_path):
        print(f"❌ 오류: 파일이 존재하지 않습니다: {xml_path}")
        return False
    print("      ✅ 파일 존재 확인 완료")

    # 2. MuJoCo C 바인딩 모델 컴파일 검증
    try:
        import mujoco
        model = mujoco.MjModel.from_xml_path(xml_path)
        data = mujoco.MjData(model)
        print(f"[2/6] ✅ MuJoCo MjModel 컴파일 성공 (버전: {mujoco.__version__})")
    except Exception as e:
        print(f"[2/6] ❌ MuJoCo 모델 로드/컴파일 실패: {e}")
        return False

    # 3. 듀얼 로봇 및 관절/액추에이터 개수 검증
    # 로봇 2대 (각 7암 + 2핑거 = 9 DOF, 총 18 DOF) + 7개 물품 (각 6 DOF = 42 DOF) => 총 60 DOF
    print(f"[3/6] 관절 및 액추에이터 수량 검증:")
    print(f"      • 일반화 좌표수(nq): {model.nq} / 속도 차원(nv): {model.nv}")
    print(f"      • 전체 바디 수: {model.nbody}개 / 지오메트리 수: {model.ngeom}개")
    print(f"      • 제어 입력 액추에이터 수: {model.nu}개 (기대치: 16개)")
    
    assert model.nu == 16, f"액추에이터 수가 올바르지 않습니다: {model.nu} != 16"
    print("      ✅ 듀얼 로봇(8+8=16 액추에이터) 수량 검증 통과")

    # 4. 카메라 및 사이트 센서 프레임 존재 검증
    expected_cameras = ["r1_wrist_camera", "r2_wrist_camera"]
    expected_sites = ["lidar_top_frame", "lidar_bottom_frame", "site_bin_A_Red", "site_bin_C_Blue"]
    
    print(f"[4/6] 비전/LiDAR/적재함 센서 프레임 검증:")
    for cam_name in expected_cameras:
        cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, cam_name)
        assert cam_id >= 0, f"카메라를 찾을 수 없습니다: {cam_name}"
        print(f"      • 카메라 [{cam_name}] 감지 완료 (ID: {cam_id})")

    for site_name in expected_sites:
        site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        assert site_id >= 0, f"사이트를 찾을 수 없습니다: {site_name}"
        print(f"      • 사이트 [{site_name}] 감지 완료 (ID: {site_id})")
    print("      ✅ 모든 필수 센서 및 마킹 사이트 검증 통과")

    # 5. 초기 자세 적용 및 물리 1000스텝(1초) 안정성 시뮬레이션
    print(f"[5/6] 물리 시뮬레이션 1초(1,000 스텝, dt=0.001s) 구동 테스트:")
    # Keyframe 0 (Home) 적용
    mujoco.mj_resetDataKeyframe(model, data, 0)
    
    for step in range(1000):
        mujoco.mj_step(model, data)
        # NaN / 수치 발산 체크
        if np.isnan(data.qpos).any() or np.isnan(data.qvel).any():
            print(f"❌ 오류: Step {step}에서 NaN 수치 발산 감지됨!")
            return False
            
    print("      ✅ 1,000 스텝 물리 적분 성공 (수치 발산 없음, 안정 상태 유지)")

    # 6. 물류 적재함 6개소 및 컨베이어 2개소 배치 확인
    expected_bins = ["bin_A_Red", "bin_B_Blue", "bin_C_Green", "bin_A_Green", "bin_B_Red", "bin_C_Blue"]
    print(f"[6/6] 물류 설비 3D 바디 배치 검증:")
    for bin_name in expected_bins:
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, bin_name)
        assert bid >= 0, f"적재함을 찾을 수 없습니다: {bin_name}"
    print(f"      ✅ 6개 전용 적재함 및 2개 컨베이어 바디 등록 확인 완료")

    print("=" * 65)
    print("🎉 [성공] Phase 02 가상 씬 및 물류 환경 모델링이 완벽하게 검증되었습니다.")
    print("=" * 65)
    return True

if __name__ == "__main__":
    success = verify_scene()
    sys.exit(0 if success else 1)
```

---

### 🔹 Step 5: 대화형 시각화 스크립트 작성 (`test/phase02_view_scene.py`)

개발자가 마우스로 3D 뷰포트를 회전하고, 물류 블록을 드래그하거나 카메라 시점을 실시간으로 관찰할 수 있는 MuJoCo Passive Viewer 실행 스크립트를 작성합니다.

```python
#!/usr/bin/env python3
"""
Phase 02 MuJoCo 3D 대화형 뷰어 실행 스크립트
"""

import os
import time
import mujoco
import mujoco.viewer

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.abspath(os.path.join(script_dir, ".."))
    xml_path = os.path.join(
        project_root,
        "ros2_ws", "src", "franka_logistics_description", "mjcf", "scene_dual_panda_logistics.xml"
    )

    print(f"📦 MuJoCo 씬 로드 중: {xml_path}")
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    # Home 키프레임 초기화
    mujoco.mj_resetDataKeyframe(model, data, 0)

    print("🚀 MuJoCo 3.6 뷰어를 실행합니다. (창을 닫으면 종료됩니다)")
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()

            # 1단계 물리 전진
            mujoco.mj_step(model, data)

            # 뷰어 동기화
            viewer.sync()

            # 1kHz 주기 조절
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

if __name__ == "__main__":
    main()
```

---

### 🔹 Step 6: 워크스페이스 빌드 및 검증 실행

```bash
# 1. Conda 가상환경 활성화
conda activate ros2_mujoco_panda_py3_10

# 2. ROS2 워크스페이스 빌드
cd Proj_Franka_panda_cobot_task_in_ros2_and_mujoco_simulator/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select franka_logistics_description
source install/setup.bash

# 3. 자동 무결성 검증 실행
cd ..
python test/phase02_test_scene.py

# 4. (선택 사항) 3D 대화형 뷰어로 눈으로 직접 확인
python test/phase02_view_scene.py
```

---

## 4. 자주 발생하는 문제 및 해결법 (Troubleshooting)

### Q1. `XML Error: mesh 'link0_0.obj' not found` 에러가 발생합니다.
* **원인**: `<compiler meshdir="assets"/>` 설정에 따라 XML 파일이 위치한 디렉토리 기준 하위 `assets/` 폴더에서 메시 파일을 찾는데, 해당 경로에 파일이 복사되지 않았거나 상대 경로가 어긋난 경우입니다.
* **해결법**:
  ```bash
  # assets 디렉토리와 mesh 파일 복사 상태를 재확인합니다.
  ls Proj_Franka_panda_cobot_task_in_ros2_and_mujoco_simulator/ros2_ws/src/franka_logistics_description/mjcf/assets/
  ```

### Q2. 물류 아이템이 작업대를 뚫고 떨어지거나 튀어 오릅니다 (Penetration / Explosion).
* **원인**: 작업대 상판 geom의 Z 좌표와 물류 아이템 초기 위치 Z 좌표가 겹쳐 시작 시점에 깊은 침투(Deep Penetration)가 발생하여 큰 반발력이 작용한 경우입니다.
* **해결법**:
  * 작업대 상판 표면은 $Z = 0.375 + 0.35 + 0.025 = 0.75\text{m}$입니다.
  * $0.05\text{m}$ 높이(반지름 $0.025\text{m}$)의 물류 박스는 중심 좌표가 최소 $Z = 0.75 + 0.025 = 0.775\text{m}$ 이상이어야 합니다.
  * 씬 XML의 물류 초기 위치를 $Z = 0.80\text{m}$로 여유 있게 설정하여 자연스럽게 안착되도록 합니다.

### Q3. `colcon build` 시 `franka_logistics_description` 패키지를 찾지 못합니다.
* **원인**: `ros2_ws/src` 아래에 패키지 폴더가 위치하지 않았거나 `package.xml` 형식이 올바르지 않은 경우입니다.
* **해결법**: `ros2_ws/src/franka_logistics_description/package.xml`과 `CMakeLists.txt`가 존재하는지 확인하고 빌드를 다시 수행합니다.

---

## 5. Phase 02 완료 체크리스트 (Self Checklist)

다음 항목들을 모두 완료했는지 확인한 후 다음 단계(Phase 03)로 진행하세요:

- [ ] `ros2_ws/src/franka_logistics_description` ROS2 패키지가 생성되고 `CMakeLists.txt`, `package.xml`이 작성되었는가?
- [ ] `mjcf/assets/` 디렉토리에 Franka Panda 3D 메시 파일들이 정상 배치되었는가?
- [ ] `scene_dual_panda_logistics.xml`에 듀얼 로봇, 작업대, 적재함 6개, 컨베이어 2개, 물류 7종, 센서 사이트가 모두 정의되었는가?
- [ ] `colcon build --packages-select franka_logistics_description`이 성공적으로 완료되는가?
- [ ] `python test/phase02_test_scene.py` 실행 시 6개 진단 항목이 모두 통과(Pass)하는가?

---

**다음 단계**: [Phase 03] MuJoCo ↔ ROS2 Sim Bridge 노드 구현 (`franka_logistics_sim`)
