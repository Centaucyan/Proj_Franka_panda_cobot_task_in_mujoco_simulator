# [로드맵] ROS2 Humble & MuJoCo 기반 Franka Panda 듀얼 협동로봇 물류 분류 및 안전 시뮬레이션 시스템 개발 계획

* **문서 번호**: ROADMAP-LOGISTICS-20260902-01-ROS2
* **작성일**: 2026-09-02
* **버전**: v3.0.0
* **개발 및 운용 환경**: 
  * **OS**: **Ubuntu 22.04 LTS (x86_64)**
  * **ROS 2**: **ROS2 Humble Hawksbill**
  * **Python 가상환경**: **Miniconda (`ros2_mujoco_panda_py3_10`, Python 3.10)** (`conda activate ros2_mujoco_panda_py3_10`)
  * **물리 시뮬레이터**: **MuJoCo 3.6.x**
  * **강화학습 프레임워크**: **Gymnasium**, **Stable-Baselines3 (PPO)**, **PyTorch**
* **빌드 시스템**: `colcon build` (C++17 / Python 3.10)

---

## 1. 프로젝트 개요 및 아키텍처 전략

### 1.1 프로젝트 목표
* **Ubuntu 22.04 + ROS2 Humble + Miniconda (`ros2_mujoco_panda_py3_10`, Python 3.10)** 환경에서 2대의 Franka Emika Panda 협동로봇과 작업자가 협업하는 스마트 물류 분류(Sorting) 시뮬레이션 시스템 구축
* **MuJoCo 3.6.x 물리 엔진**의 1kHz 고속 정밀 동역학과 ROS2의 분산 비동기 노드 프레임워크(DDS) 결합
* **Sim Bridge 노드**를 통한 MuJoCo-ROS2 간 완벽한 하드웨어 추상화 및 가상 센서/액추에이터 파이프라인 수립
* Wrist Depth 카메라 기반 다품종 물류(A/B/C 타입, 색상별) 인식 및 Place 직후 **적재함 3D Depth 스캔 점유율(%) 산출**
* **공유 작업 영역(Shared Workspace) 선점 우선순위(FCFS) 및 듀얼 로봇 충돌 방지/회피**
* 2D-LiDAR 2기 기반 작업자 접근 감속/정지 (ISO/TS 15066 SSM)
* 작업대 물류 소진 감지 및 **10분 미공급 시 자동 셧다운(Auto-Shutdown)** 메커니즘 구축
* **Gymnasium 표준 인터페이스 및 PPO 강화학습(RL)** 기반 듀얼 로봇 동적 충돌 회피 및 지능형 궤적 최적화

---

### 1.2 2단계 점진적 지능화 전략 (Baseline & RL Evolution)

```
 ┌───────────────────────────────────────────────────────────────────────────┐
 │                  DualFrankaLogisticsEnv (Gymnasium 표준)                   │
 │                                                                           │
 │   • reset()        : 작업대 위 무작위 물품 배치, 적재함 초기화                  │
 │   • step(action)   : 듀얼 로봇 물리 엔진 전진 (mj_step), 센서 갱신             │
 │   • get_obs()      : 조인트 상태, Depth Map, LiDAR 계측치, 적재함 차지율       │
 │   • compute_reward(): 분류 성공(+), 간섭 회피(+), 충돌/안전 위반(-)            │
 └─────────────────────────────────────┬─────────────────────────────────────┘
                                       │
     ┌─────────────────────────────────┴─────────────────────────────────┐
     ▼                                                                   ▼
[ 1단계: Rule-based Baseline (Phase 01~10) ]               [ 2단계: Multi-Agent PPO RL (Phase 11) ]
 • DLS IK + 5차 다항식 궤적 생성기                            • Stable-Baselines3 Multi-Agent PPO Policy
 • FCFS 선점 조율기 + Safety Supervisor                      • 동적 장애물/작업자 적응형 회피 및 속도 최적화
 • 결정론적 안정성 및 정량 KPI 검증                          • 신경망 ONNX/TorchScript 실시간 ROS2 추론
```

---

### 1.3 점진적(Just-In-Time) 패키지 및 인터페이스 구성

각 패키지와 인터페이스는 초기에 한 번에 생성하지 않고, **해당 기능 개발 단계(Phase)에 진입할 때 목적과 당위성을 학습하며 순차적으로 생성/확장**합니다.

```text
Proj_Franka_panda_cobot_task_in_ros2_and_mujoco_simulator/
├── documents/
│   ├── development_roadmap/
│   │   ├── 20260902_01_development_roadmap.md
│   │   ├── rm_phase01_environment_and_workspace_setup.md
│   │   └── rm_phase02_mujoco_scene_modeling.md
│   └── scenario_and_prd/
├── model_ori/
│   └── franka_emika_panda/
├── environment.yaml
├── test/
│   ├── phase01_test_env.py
│   ├── phase02_test_scene.py
│   ├── phase03_explore_data.py         # [Phase 03 추가] MuJoCo 원시 데이터 I/O 탐색
│   └── phase03_single_pnp.py           # [Phase 03 추가] 단일 로봇 스탠드얼론 P&P 테스트
└── ros2_ws/src/
    ├── franka_logistics_msgs/          # [Phase 01 패키지 생성, Phase 05~08 순차 인터페이스 추가]
    │   ├── action/
    │   │   └── SortItem.action         # [Phase 05 추가] 단일 물류 파지 및 비동기 안착 제어
    │   ├── msg/
    │   │   ├── BinStatus.msg           # [Phase 06 추가] 개별 적재함 차지율 및 만재 상태
    │   │   ├── BinStatusArray.msg      # [Phase 06 추가] 6개 적재함 전체 상태 배열
    │   │   └── SafetyStatus.msg        # [Phase 07 추가] LiDAR 경고/위험 구역 침범 및 E-Stop 상태
    │   └── srv/
    │       └── MesNotification.srv     # [Phase 08 추가] 상위 MES 연동 요청-응답 (90% 경고, 만재, 소진)
    │
    ├── franka_logistics_description/   # [Phase 02 생성] 로봇, 작업대, 적재함, 컨베이어 MJCF 및 3D 메시
    │   ├── mjcf/scene_dual_panda_logistics.xml
    │   ├── mjcf/assets/
    │   └── meshes/
    │
    ├── franka_logistics_sim/           # [Phase 04 생성] MuJoCo 3.6.x 물리 시뮬레이션 및 ROS2 Sim Bridge 노드
    │   ├── simulation_bridge_node      # 1kHz 물리 스텝, JointState, Camera, LiDAR 퍼블리셔
    │   └── conveyor_controller_node    # 분류 불가품 배출 컨베이어 물리 구동기
    │
    ├── franka_logistics_control/       # [Phase 05 생성] 로봇 기구학 및 모션 제어 (Rule-based & RL Policy)
    │   ├── ik_solver                   # DLS 기반 수치적 역기구학
    │   ├── trajectory_planner          # 5차 다항식 부드러운 궤적 생성기
    │   ├── rl_policy_controller_node   # 학습된 PPO 신경망 기반 궤적/속도 추론기
    │   └── motion_controller_node      # FollowJointTrajectory 및 로봇별 P&P 제어
    │
    ├── franka_logistics_vision/        # [Phase 06 생성] 비전 인지 및 3D Depth 적재함 스캔
    │   ├── item_detector_node          # Wrist 카메라 기반 물류 색상/타입 분류
    │   └── bin_occupancy_scanner_node  # Place 직후 적재함 3D Depth 스캔 및 차지율 계산
    │
    ├── franka_logistics_safety/        # [Phase 07 생성] 안전 감독관 (Safety Supervisor)
    │   ├── lidar_monitor_node          # 2D LiDAR 2기 Warning/Danger Zone 레이캐스트 판별
    │   ├── ssm_calculator              # ISO/TS 15066 SSM 동적 안전 이격거리 계산
    │   └── safety_supervisor_node      # 작업자 접근 감속/정지 및 E-Stop 인터록
    │
    ├── franka_logistics_fsm/           # [Phase 08 생성] 중앙 공정 조율기 (FSM / BehaviorTree)
    │   ├── dual_robot_coordinator_node # 듀얼 로봇 FCFS 충돌 회피 및 최근접 타겟 할당
    │   └── fsm_manager_node            # 정상/예외 시퀀스, 10분 미공급 셧다운 타이머
    │
    ├── franka_logistics_hmi/           # [Phase 09 생성] HMI 제어반 및 RViz2 3D 대시보드
    │   ├── hmi_panel_node              # On/Off, Start/Stop, E-Stop 제어반
    │   └── rviz_marker_publisher_node  # 적재함 차지율 HUD, LiDAR 안전 구역 3D 오버레이
    │
    ├── franka_logistics_bringup/       # [Phase 10 생성] 통합 Launch 및 설정 파일
    │   ├── config/logistics_config.yaml
    │   ├── launch/system_bringup.launch.py
    │   └── rviz/dual_panda_logistics.rviz
    │
    └── franka_logistics_rl/            # [Phase 11 생성] 강화학습 환경 및 학습 파이프라인
        ├── envs/dual_panda_env.py      # Gymnasium 표준 듀얼 로봇 환경
        ├── train_ppo.py                # Stable-Baselines3 PPO 분산 학습 스크립트
        └── export_policy.py            # 학습 완료 모델 ONNX/PyTorch 변환
```

---

## 2. 단계별 마일스톤 (Milestones Overview)

| 단계 (Phase) | 마일스톤 명칭 | 신규 생성 패키지 & 인터페이스 | 상세 실습 가이드 & 주요 산출물 | 예상 기간 |
| :--- | :--- | :--- | :--- | :---: |
| **Phase 01** | 가상환경 구축 & ROS2 기본 환경 준비 | `franka_logistics_msgs` (패키지 뼈대) | • [rm_phase01_environment_and_workspace_setup.md](./rm_phase01_environment_and_workspace_setup.md)<br>• `environment.yaml` (Python 3.10) | 1~2일 |
| **Phase 02** | MuJoCo 3.6.x 3D 가상 씬 및 물류 환경 모델링 | `franka_logistics_description` | • [rm_phase02_mujoco_scene_modeling.md](./rm_phase02_mujoco_scene_modeling.md)<br>• `scene_dual_panda_logistics.xml` (로봇 2대, 작업대, 적재함 6개, 컨베이어 2개, 물류 7종) | 2~3일 |
| **Phase 03** | **MuJoCo Python API 데이터 탐색 & 단일 로봇 P&P 프로토타이핑** | 스탠드얼론 프로토타입 스크립트 | • `rm_phase03_mujoco_data_and_single_pnp.md`<br>• `test/phase03_explore_data.py`<br>• `test/phase03_single_pnp.py` (단일 P&P 성공) | 2~3일 |
| **Phase 04** | **MuJoCo ↔ ROS2 Sim Bridge 노드 구현** | `franka_logistics_sim` | • `rm_phase04_ros2_sim_bridge_node.md`<br>• `simulation_bridge_node` (1kHz 물리 루프, JointState, Camera, LiDAR 퍼블리셔) | 3~4일 |
| **Phase 05** | **로봇 기구학, 궤적 생성 & P&P Action 서버** | `franka_logistics_control`<br>➕ `SortItem.action` | • `rm_phase05_kinematics_and_sort_item_action.md`<br>• DLS IK 솔버, 5차 다항식 궤적 생성기, `SortItem.action` 서버/클라이언트 | 3~4일 |
| **Phase 06** | **비전 분류, 적재함 3D 차지율 스캔 & 컨베이어 배출** | `franka_logistics_vision`<br>➕ `BinStatus.msg`, `BinStatusArray.msg` | • `rm_phase06_vision_and_bin_occupancy.md`<br>• Wrist Depth 물류 검출 노드, **적재함 3D 차지율 계산 노드**, 컨베이어 구동 노드 | 3~4일 |
| **Phase 07** | **듀얼 로봇 FCFS 충돌 방지 & 안전 감독관 구축** | `franka_logistics_safety`<br>➕ `SafetyStatus.msg` | • `rm_phase07_fcfs_coordinator_and_safety.md`<br>• **공유 작업 영역 선점 회피 조율기**, 2D-LiDAR SSM 감속/정지, E-Stop 인터록 | 4~5일 |
| **Phase 08** | **공정 조율 FSM, MES 인터페이스 & 10분 Auto-Off** | `franka_logistics_fsm`<br>➕ `MesNotification.srv` | • `rm_phase08_fsm_and_mes_notification.md`<br>• 전체 분류 FSM, 만재 시 품목 제외, 10분 미공급 셧다운 타이머, MES 통신 | 3~4일 |
| **Phase 09** | **HMI 제어반 서비스 & RViz2 3D 모니터링 HUD** | `franka_logistics_hmi` | • `rm_phase09_hmi_and_rviz_dashboard.md`<br>• HMI 스위치 3종 서비스, **적재함 차지율 RViz2 HUD**, 3D 안전 구역 렌더러 | 2~3일 |
| **Phase 10** | **종합 통합 검증, 5-KPI 평가 & ros2bag 테스트** | `franka_logistics_bringup` | • `rm_phase10_system_integration_and_kpi.md`<br>• `system_bringup.launch.py`, 100개 물품 스트레스 테스트, 5대 KPI 정량 평가 | 3~4일 |
| **Phase 11** | **Gymnasium 연동 & PPO 강화학습(RL) 고도화** | `franka_logistics_rl` | • `rm_phase11_gymnasium_and_ppo_rl.md`<br>• `DualFrankaLogisticsEnv`, PPO 학습 신경망, ROS2 추론 노드, Rule vs RL 성능 비교 | 4~5일 |

---

## 3. 상세 단계별 개발 계획 (Detailed Action Plan)

### 📌 Phase 01: 가상환경 구축 & ROS2 기본 환경 준비
* **실습 가이드**: [rm_phase01_environment_and_workspace_setup.md](./rm_phase01_environment_and_workspace_setup.md)
* **목표**: Ubuntu 22.04 상에서 Miniconda 가상환경(`ros2_mujoco_panda_py3_10`, Python 3.10)을 구축하고, 커스텀 인터페이스 패키지(`franka_logistics_msgs`) 기본 뼈대를 빌드하여 ROS2 Humble 워크스페이스 환경 검증.
* **세부 작업**:
  1. **Miniconda 환경 파일 (`environment.yaml`) 작성 및 활성화**:
     * `name: ros2_mujoco_panda_py3_10`
     * `python=3.10`, `mujoco>=3.6.0`, `gymnasium`, `stable-baselines3`, `torch`, `numpy`, `scipy`, `opencv-python`, `open3d`, `pyyaml`, `rich`, `colcon-common-extensions`, `transforms3d` 등 정의.
     * `conda env create -f environment.yaml` 및 `conda activate ros2_mujoco_panda_py3_10` 검증.
  2. **인터페이스 패키지 뼈대 생성 (`franka_logistics_msgs`)**:
     * `ros2 pkg create --build-type ament_cmake franka_logistics_msgs` 명령으로 기본 패키지 생성.
     * `package.xml` 및 `CMakeLists.txt`에 `rosidl_default_generators` 의존성 기본 설정.
     * `colcon build --packages-select franka_logistics_msgs`를 통한 기본 빌드 파이프라인 정상 작동 확인.

---

### 📌 Phase 02: MuJoCo 3.6.x 3D 가상 씬 및 물류 환경 모델링
* **실습 가이드**: [rm_phase02_mujoco_scene_modeling.md](./rm_phase02_mujoco_scene_modeling.md)
* **목표**: PRD 작업 공간 레이아웃 명세를 충족하는 설명 패키지(`franka_logistics_description`)를 생성하고, 통합 MJCF 가상 씬(`scene_dual_panda_logistics.xml`) 완성.
* **신규 패키지**: `franka_logistics_description` (`ament_cmake`)
* **세부 작업**:
  1. **`franka_logistics_description` 패키지 생성**: MJCF XML 및 3D 메시 파일 관리를 위한 디렉토리 구조 수립.
  2. **MJCF 가상 씬 모델링 (`scene_dual_panda_logistics.xml`)**:
     * Dual Franka Panda 로봇 (좌: 로봇 1 `r1_`, 우: 로봇 2 `r2_`) 및 그리퍼 배치.
     * 중앙 대형 물류 분류 작업대 테이블 ($2.8\text{m} \times 0.9\text{m} \times 0.75\text{m}$).
     * 전용 적재함 6개 (좌: A-R, B-B, C-G / 우: A-G, B-R, C-B).
     * 배출 컨베이어 벨트 2개소 (좌측 하단, 우측 상단).
     * 물류 아이템 7종(분류 품목 6종 + 규격 외 불량품 1종) 모델링.
  3. **센서 마운팅 지오메트리 정의**:
     * 각 로봇 EE Wrist `<camera>` (Depth/RGB 렌더링용, `fovy="58"`).
     * 상단/하단 가상 2D LiDAR 센서 기준 사이트(`lidar_top_frame`, `lidar_bottom_frame`) 정의.
  4. **자가 진단 스크립트 작성**: `test/phase02_test_scene.py` 및 `test/phase02_view_scene.py` 검증.

---

### 📌 Phase 03: MuJoCo Python API 데이터 탐색 & 단일 로봇 P&P 프로토타이핑
* **실습 가이드**: `rm_phase03_mujoco_data_and_single_pnp.md`
* **목표**: ROS2 패키징 이전에, MuJoCo 물리 엔진이 제공하는 원시(Raw) 데이터 구조를 심층 분석하고, 순수 Python으로 로봇 1대가 테이블 위 물품을 집어 적재함에 넣는 1회 사이클을 먼저 성공시킴.
* **주요 산출물**: `test/phase03_explore_data.py`, `test/phase03_single_pnp.py`
* **세부 작업**:
  1. **MuJoCo 원시 데이터 I/O 분석 (`test/phase03_explore_data.py`)**:
     * 관절 상태 `data.qpos` (18 DOF), `data.qvel`, 액추에이터 제어 `data.ctrl` (16개) 콘솔 출력 및 구조 확인.
     * Wrist 카메라 오프스크린 렌더러 기반 RGB/Depth NumPy 배열(`(480, 640, 3)`) 직접 시각화.
     * `mj_ray()` 레이캐스팅을 통한 거리값 계측 및 충돌 지오메트리 탐색 확인.
  2. **DLS 수치적 역기구학(IK) 기초 구현**:
     * 목표 3D 위치 $(X, Y, Z)$가 주어졌을 때 7개 관절 각도를 도출하는 순수 Python 기구학 기초 함수 작성.
  3. **그리퍼 파지 및 접촉 역학(Contact Dynamics) 튜닝**:
     * 물품 파지 시 미끄러짐/폭발(Explosion) 없는 최적의 그리퍼 제어값(`ctrl=255/0`) 및 마찰력 확인.
  4. **단일 로봇 Pick & Place 1회 사이클 스탠드얼론 시뮬레이션 (`test/phase03_single_pnp.py`)**:
     * 물품(`item_A_Red`) 접근 $\rightarrow$ 하강 $\rightarrow$ 파지 $\rightarrow$ 리프팅 $\rightarrow$ 적재함(`bin_A_Red`) 상단 이동 $\rightarrow$ 하강 및 놓기 $\rightarrow$ 홈 복귀를 3D 뷰어로 확인.

---

### 📌 Phase 04: MuJoCo ↔ ROS2 Sim Bridge 노드 구현 (가상 하드웨어 드라이버)
* **실습 가이드**: `rm_phase04_ros2_sim_bridge_node.md`
* **목표**: Phase 03에서 확인한 MuJoCo 원시 데이터를 ROS2 표준 인터페이스(`sensor_msgs`, `geometry_msgs`)로 변환 및 발행하는 고속 통신 시뮬레이션 패키지(`franka_logistics_sim`) 생성 및 브릿지 노드 구현.
* **신규 패키지**: `franka_logistics_sim`
* **사용 인터페이스**: ROS2 표준 메시지 (`sensor_msgs`, `geometry_msgs`, `std_msgs`)
* **세부 작업**:
  1. **`franka_logistics_sim` 패키지 생성**: 물리 시뮬레이션 구동 및 브릿지 노드 구현용.
  2. **1kHz 물리 시뮬레이션 루프**: MuJoCo C API / Python 바인딩 기반 $1\text{ms}$ 주기 루프(`mj_step`).
  3. **Joint State Publisher**: `/robot1/joint_states`, `/robot2/joint_states` (1000Hz, `sensor_msgs/msg/JointState`).
  4. **Joint & Gripper Command Subscriber**: 목표 위치/속도/토크 지령을 받아 `mjData.ctrl`에 실시간 인가.
  5. **Wrist Depth/RGB Camera Publisher**: OpenGL 오프스크린 렌더러 기반 영상 캡처 및 `sensor_msgs/msg/Image` (30Hz, `cv_bridge`) 발행.
  6. **2D LiDAR 레이캐스터**: 상/하단 기준점 평면 방사형 레이캐스팅 및 `sensor_msgs/msg/LaserScan` (20Hz) 발행.

---

### 📌 Phase 05: 로봇 기구학, 궤적 생성 & P&P Action 서버
* **실습 가이드**: `rm_phase05_kinematics_and_sort_item_action.md`
* **목표**: 제어 패키지(`franka_logistics_control`)를 생성하고, Pick & Place의 비동기 제어/피드백/취소를 위한 `SortItem.action`을 정의하여 Action 기반 물류 이송 파이프라인 구현.
* **신규 패키지**: `franka_logistics_control`
* **신규 인터페이스 추가**: `franka_logistics_msgs/action/SortItem.action`
* **세부 작업**:
  1. **`SortItem.action` 정의 및 인터페이스 빌드**:
     * Goal: 물품 ID, 타입/색상, 목표 적재함 ID / Feedback: 현재 진행 단계(Approach, Grasp 등), 진행률 / Result: 성공 여부, 소요 시간.
     * `colcon build --packages-select franka_logistics_msgs` 재빌드.
  2. **`franka_logistics_control` 패키지 생성**.
  3. **DLS(Damped Least Squares) 수치적 IK 솔버**: 7-DoF 특이점 회피 및 위치 오차 $\le 1\text{mm}$.
  4. **5차 다항식 궤적 생성기 (Quintic Planner)**: 부드러운 S-Curve 가감속 프로파일 산출.
  5. **최단 거리 우선 타겟팅 모듈**: 로봇 Base 기준 최근접 물품 우선 선별.
  6. **`SortItem.action` 서버 구현**: Approach $\rightarrow$ Grasp $\rightarrow$ Lift $\rightarrow$ Transfer $\rightarrow$ Place $\rightarrow$ Home 복귀.

---

### 📌 Phase 06: 비전 분류, 적재함 3D 차지율 스캔 & 컨베이어 배출
* **실습 가이드**: `rm_phase06_vision_and_bin_occupancy.md`
* **목표**: 비전 패키지(`franka_logistics_vision`)를 생성하고, 적재함 상태 브로드캐스팅을 위한 `BinStatus.msg`/`BinStatusArray.msg`를 정의하여 비전 분류 및 3D 차지율 산출 파이프라인 완성.
* **신규 패키지**: `franka_logistics_vision`
* **신규 인터페이스 추가**: `franka_logistics_msgs/msg/BinStatus.msg`, `BinStatusArray.msg`
* **세부 작업**:
  1. **`BinStatus.msg` 및 `BinStatusArray.msg` 정의 및 빌드**:
     * `BinStatus.msg`: `string bin_id`, `float64 occupancy_rate`, `bool is_full`, `int32 item_count`.
     * `BinStatusArray.msg`: `BinStatus[] bins`.
     * `colcon build --packages-select franka_logistics_msgs` 재빌드.
  2. **`franka_logistics_vision` 패키지 생성**.
  3. **`item_detector_node`**: 물품 바운딩박스/형상/색상 추출 $\rightarrow$ A/B/C 타입 분류 및 불가품 판정.
  4. **`bin_occupancy_scanner_node`**: Place 직후 적재함 상단 Depth 스캔 $\rightarrow$ 3D Point Cloud 복원 $\rightarrow$ 적재 부피 적분 $\rightarrow$ 실시간 차지율(%) 산출 (`/system/bin_occupancy`).
  5. **`conveyor_controller_node`**: 분류 불가품 구역 안착 감지 시 설정 선속도($v_c$)로 외부 연속 이송.

---

### 📌 Phase 07: 듀얼 로봇 FCFS 충돌 방지 & 안전 감독관 구축
* **실습 가이드**: `rm_phase07_fcfs_coordinator_and_safety.md`
* **목표**: 안전 감독 패키지(`franka_logistics_safety`)를 생성하고, 안전 및 E-Stop 상태 전파를 위한 `SafetyStatus.msg`를 정의하여 FCFS 충돌 방지 및 ISO/TS 15066 SSM 안전 체계 구축.
* **신규 패키지**: `franka_logistics_safety`
* **신규 인터페이스 추가**: `franka_logistics_msgs/msg/SafetyStatus.msg`
* **세부 작업**:
  1. **`SafetyStatus.msg` 정의 및 빌드**:
     * `bool is_warning`, `bool is_danger`, `bool is_estop`, `float64 speed_scale_factor`.
     * `colcon build --packages-select franka_logistics_msgs` 재빌드.
  2. **`franka_logistics_safety` 패키지 생성**.
  3. **`dual_robot_coordinator_node` (FCFS 충돌 회피)**: 공유 영역 진입 요청 시 **선착 로봇에게 모션 우선권 부여**, 후착 로봇은 안전 대기 웨이포인트 회피 대기.
  4. **`safety_supervisor_node`**: 작업자 접근 시 감속($\le 250\text{mm/s}$) 및 작업대 진입 시 일시정지, LiDAR Warning($50\%$ 감속) / Danger(E-Stop).
  5. 비상 정지(E-Stop) 소프트웨어 브레이크 및 토크 차단 로직.

---

### 📌 Phase 08: 공정 조율 FSM, MES 인터페이스 & 10분 Auto-Off
* **실습 가이드**: `rm_phase08_fsm_and_mes_notification.md`
* **목표**: 상태 관리 패키지(`franka_logistics_fsm`)를 생성하고, 상위 MES 연동을 위한 `MesNotification.srv`를 정의하여 전체 공정 시퀀스 및 10분 미공급 자동 셧다운 완성.
* **신규 패키지**: `franka_logistics_fsm`
* **신규 인터페이스 추가**: `franka_logistics_msgs/srv/MesNotification.srv`
* **세부 작업**:
  1. **`MesNotification.srv` 정의 및 빌드**:
     * Request: `string event_type`, `string target_bin_id`, `string message` / Response: `bool success`, `string response_msg`.
     * `colcon build --packages-select franka_logistics_msgs` 재빌드.
  2. **`franka_logistics_fsm` 패키지 생성**.
  3. **`fsm_manager_node`**:
     * 전체 공정 상태 머신 제어.
     * 적재함 90% 도달 시 MES `[WARN_BIN_NEAR_FULL]` 이벤트 발행.
     * 100% 만재 시 `[ERR_BIN_FULL]` 발행 및 해당 품목 분류 대상에서 일시 제외.
     * 작업대 물류 전량 소진 시 일시정지 $\rightarrow$ **10분 미공급 시 자동 셧다운(`Auto-Shutdown`)**.
  4. 상위 MES 연동 Bridge 서비스 클라이언트 구현.

---

### 📌 Phase 09: HMI 제어반 서비스 & RViz2 3D 모니터링 HUD
* **실습 가이드**: `rm_phase09_hmi_and_rviz_dashboard.md`
* **목표**: HMI 패키지(`franka_logistics_hmi`)를 생성하고, 제어반 서비스 및 RViz2 3D HUD 대시보드 구축.
* **신규 패키지**: `franka_logistics_hmi`
* **사용 인터페이스**: `std_srvs/srv/SetBool`, `std_srvs/srv/Trigger`, `visualization_msgs/msg/MarkerArray`, `franka_logistics_msgs`
* **세부 작업**:
  1. **`franka_logistics_hmi` 패키지 생성**.
  2. **`hmi_panel_node`**: `/hmi/power_switch`, `/hmi/start_stop`, `/hmi/estop` 서비스 제공.
  3. **RViz2 3D 오버레이 (`rviz_marker_publisher_node`)**: LiDAR 감지 영역(노랑/빨강) 3D Marker, **6개 적재함 실시간 차지율(%) 게이지 HUD**, 로봇 충돌 간섭 영역 시각화.

---

### 📌 Phase 10: 종합 통합 검증, 5-KPI 평가 & ros2bag 테스트
* **실습 가이드**: `rm_phase10_system_integration_and_kpi.md`
* **목표**: 전체 런처 및 설정을 총괄하는 `franka_logistics_bringup` 패키지를 생성하고, PRD 5대 KPI 정량 평가 및 ros2bag 데이터 무결성 검증.
* **신규 패키지**: `franka_logistics_bringup`
* **세부 작업**:
  1. **`franka_logistics_bringup` 패키지 생성**: `config/logistics_config.yaml`, `launch/system_bringup.launch.py`, `rviz/dual_panda_logistics.rviz`.
  2. PRD KPI 5대 항목 정량 평가 (분류 정확도 $\ge 99.0\%$, 충돌 방지율 $100\%$, 차지율 오차 $\le \pm 5\%$, 만재 품목 제외 $100\%$, 10분 셧다운 정밀도).
  3. 100개 무작위 물품 스트레스 테스트 및 `ros2bag` 기록/분석.
  4. 1차 Rule-based 개발 완료 보고서 작성.

---

### 📌 Phase 11: Gymnasium 연동 & PPO 강화학습(RL) 고도화 (지능형 적응 제어)
* **실습 가이드**: `rm_phase11_gymnasium_and_ppo_rl.md`
* **목표**: 강화학습 전용 패키지(`franka_logistics_rl`)를 생성하고, Gymnasium 표준 환경 구축 및 PPO 알고리즘을 통한 동적 충돌 회피/지능형 궤적 최적화 정책 학습 및 ROS2 실시간 배포.
* **신규 패키지**: `franka_logistics_rl`
* **세부 작업**:
  1. **`franka_logistics_rl` 패키지 생성**.
  2. **Gymnasium 표준 환경 래퍼 (`DualFrankaLogisticsEnv`) 구현**:
     * **Action Space**: 듀얼 로봇 관절 속도/토크 또는 EE 델타 포즈 ($\mathbb{R}^{14+2}$)
     * **Observation Space**: 관절 각도/속도, Wrist Depth Point Cloud, 물류 3D 좌표, LiDAR 이격 거리
     * **보상 함수 (Reward Function)**:
       $$R = r_{\text{sort\_success}} (+100) - r_{\text{collision}} (-200) - r_{\text{safety\_violation}} (-50) - \alpha \cdot \text{StepTime}$$
  3. **PPO (Proximal Policy Optimization) 정책 학습 파이프라인**:
     * `Stable-Baselines3` 기반 분산 병렬 학습 환경 구성 (SubprocVecEnv).
     * 작업대 중앙 공유 영역에서의 듀얼 로봇 간 실시간 자율 회피 및 동적 장애물(작업자 손/몸통) 적응형 우회 궤적 학습.
  4. **ROS2 실시간 추론 노드 (`rl_policy_controller_node`) 구현**:
     * 학습된 Policy 신경망을 ONNX/TorchScript로 변환하여 ROS2 제어 노드에 탑재, $100\text{Hz}$ 실시간 추론 실행.
  5. **Rule-based vs RL Policy 정량 비교 평가**:
     * 시간당 물류 분류 처리량(Throughput), 충돌 회피 응답 지연 시간, 모션 소비 에너지 효율성 비교 분석 보고서 작성.
