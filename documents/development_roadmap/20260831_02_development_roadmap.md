# [로드맵] ROS2 Humble & MuJoCo 기반 Franka Panda 듀얼 협동로봇 물류 분류 및 안전 시뮬레이션 시스템 개발 계획

* **문서 번호**: ROADMAP-LOGISTICS-20260831-02-ROS2
* **작성일**: 2026-08-31
* **버전**: v2.5.0
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
 │                  DualFrankaLogisticsEnv (Gymnasium 표준)                  │
 │                                                                           │
 │   • reset()        : 작업대 위 무작위 물품 배치, 적재함 초기화            │
 │   • step(action)   : 듀얼 로봇 물리 엔진 전진 (mj_step), 센서 갱신        │
 │   • get_obs()      : 조인트 상태, Depth Map, LiDAR 계측치, 적재함 차지율  │
 │   • compute_reward(): 분류 성공(+), 간섭 회피(+), 충돌/안전 위반(-)       │
 └─────────────────────────────────────┬─────────────────────────────────────┘
                                       │
     ┌─────────────────────────────────┴─────────────────────────────────┐
     ▼                                                                   ▼
[ 1단계: Rule-based Baseline (Phase 01~09) ]               [ 2단계: Multi-Agent PPO RL (Phase 10) ]
 • DLS IK + 5차 다항식 궤적 생성기                            • Stable-Baselines3 Multi-Agent PPO Policy
 • FCFS 선점 조율기 + Safety Supervisor                      • 동적 장애물/작업자 적응형 회피 및 속도 최적화
 • 결정론적 안정성 및 정량 KPI 검증                          • 신경망 ONNX/TorchScript 실시간 ROS2 추론
```

---

### 1.3 ROS2 패키지 구성

```text
ros2_ws/src/
├── franka_logistics_msgs/          # 사용자 정의 ROS2 인터페이스 (msg, srv, action)
│   ├── msg/
│   │   ├── BinStatus.msg           # 개별 적재함 차지율 및 만재 상태
│   │   ├── BinStatusArray.msg      # 6개 적재함 전체 상태 배열
│   │   └── SafetyStatus.msg        # LiDAR 경고/위험 구역 침범 및 E-Stop 상태
│   ├── srv/
│   │   └── MesNotification.srv     # 상위 MES 연동 알림 (90% 경고, 만재, 소진)
│   └── action/
│       └── SortItem.action         # 단일 물류 파지 및 분류 안착 비동기 액션
│
├── franka_logistics_description/   # 로봇, 작업대, 적재함, 컨베이어 MJCF/URDF 및 3D 메시
│   ├── mjcf/scene_dual_panda_logistics.xml
│   └── meshes/
│
├── franka_logistics_sim/           # MuJoCo 3.6.x 물리 시뮬레이션 및 ROS2 Sim Bridge 노드
│   ├── simulation_bridge_node      # 1kHz 물리 스텝, JointState, Camera, LiDAR 퍼블리셔
│   └── conveyor_controller_node    # 분류 불가품 배출 컨베이어 물리 구동기
│
├── franka_logistics_control/       # 로봇 기구학 및 모션 제어 (Rule-based & RL Policy)
│   ├── ik_solver                   # DLS 기반 수치적 역기구학
│   ├── trajectory_planner          # 5차 다항식 부드러운 궤적 생성기
│   ├── rl_policy_controller_node   # 학습된 PPO 신경망 기반 궤적/속도 추론기
│   └── motion_controller_node      # FollowJointTrajectory 및 로봇별 P&P 제어
│
├── franka_logistics_vision/        # 비전 인지 및 3D Depth 적재함 스캔
│   ├── item_detector_node          # Wrist 카메라 기반 물류 색상/타입 분류
│   └── bin_occupancy_scanner_node  # Place 직후 적재함 3D Depth 스캔 및 차지율 계산
│
├── franka_logistics_safety/        # 안전 감독관 (Safety Supervisor)
│   ├── lidar_monitor_node          # 2D LiDAR 2기 Warning/Danger Zone 레이캐스트 판별
│   ├── ssm_calculator              # ISO/TS 15066 SSM 동적 안전 이격거리 계산
│   └── safety_supervisor_node      # 작업자 접근 감속/정지 및 E-Stop 인터록
│
├── franka_logistics_fsm/           # 중앙 공정 조율기 (FSM / BehaviorTree)
│   ├── dual_robot_coordinator_node # 듀얼 로봇 FCFS 충돌 회피 및 최근접 타겟 할당
│   └── fsm_manager_node            # 정상/예외 시퀀스, 10분 미공급 셧다운 타이머
│
├── franka_logistics_hmi/           # HMI 제어반 및 RViz2 3D 대시보드
│   ├── hmi_panel_node              # On/Off, Start/Stop, E-Stop 제어반
│   └── rviz_marker_publisher_node  # 적재함 차지율 HUD, LiDAR 안전 구역 3D 오버레이
│
├── franka_logistics_rl/            # 강화학습 환경 및 학습 파이프라인
│   ├── envs/dual_panda_env.py      # Gymnasium 표준 듀얼 로봇 환경
│   ├── train_ppo.py                # Stable-Baselines3 PPO 분산 학습 스크립트
│   └── export_policy.py            # 학습 완료 모델 ONNX/PyTorch 변환
│
└── franka_logistics_bringup/       # 통합 Launch 및 설정 파일
    ├── config/logistics_config.yaml
    ├── launch/system_bringup.launch.py
    └── rviz/dual_panda_logistics.rviz
```

---

## 2. 단계별 마일스톤 (Milestones Overview)

| 단계 (Phase) | 마일스톤 명칭 | 주요 산출물 | 예상 기간 |
| :--- | :--- | :--- | :---: |
| **Phase 01** | 가상환경 구축 & ROS2 패키지/인터페이스 정의 | `environment.yml` (Python 3.10), `franka_logistics_msgs` (msg/srv/action) | 1~2일 |
| **Phase 02** | MuJoCo 3.6.x 3D 가상 씬 및 물류 환경 모델링 | `scene_dual_panda_logistics.xml` (로봇 2대, 작업대, 적재함 6개, 컨베이어 2개, 물류 6종) | 2~3일 |
| **Phase 03** | MuJoCo ↔ ROS2 Sim Bridge 노드 구현 | `simulation_bridge_node` (1kHz 물리 루프, JointState, Camera, LiDAR 퍼블리셔) | 3~4일 |
| **Phase 04** | 로봇 기구학, 궤적 생성 & P&P Action 서버 | DLS IK 솔버, 5차 다항식 궤적 생성기, `SortItem.action` 서버 | 3~4일 |
| **Phase 05** | 비전 분류, 적재함 3D 차지율 스캔 & 컨베이어 | Wrist Depth 물류 검출 노드, **적재함 3D 차지율 계산 노드**, 컨베이어 구동 노드 | 3~4일 |
| **Phase 06** | 듀얼 로봇 FCFS 충돌 방지 & 안전 감독관 구축 | **공유 작업 영역 선점 회피 조율기**, 2D-LiDAR SSM 감속/정지, E-Stop 인터록 | 4~5일 |
| **Phase 07** | 공정 조율 FSM, MES 인터페이스 & 10분 Auto-Off | 전체 분류 FSM, 만재 시 품목 제외, 10분 미공급 셧다운 타이머, MES 통신 | 3~4일 |
| **Phase 08** | HMI 제어반 서비스 & RViz2 3D 모니터링 HUD | HMI 스위치 3종 서비스, **적재함 차지율 RViz2 HUD**, 3D 안전 구역 렌더러 | 2~3일 |
| **Phase 09** | 종합 통합 검증, 5-KPI 평가 & ros2bag 테스트 | `system_bringup.launch.py`, 100개 물품 스트레스 테스트, 5대 KPI 정량 평가 | 3~4일 |
| **Phase 10** | **Gymnasium 연동 & PPO 강화학습(RL) 고도화** | `DualFrankaLogisticsEnv`, PPO 학습 신경망, ROS2 추론 노드, Rule vs RL 성능 비교 | 4~5일 |

---

## 3. 상세 단계별 개발 계획 (Detailed Action Plan)

### 📌 Phase 01: 가상환경 구축 & ROS2 패키지/인터페이스 정의
* **목표**: Ubuntu 22.04 상에서 Miniconda 가상환경(`ros2_mujoco_panda_py3_10`, Python 3.10)을 구축하고 ROS2 Humble 워크스페이스 및 인터페이스 패키지 빌드.
* **세부 작업**:
  1. **Miniconda 환경 파일 (`environment.yml`) 작성**:
     * `name: ros2_mujoco_panda_py3_10`
     * `python=3.10`, `mujoco>=3.6.0`, `gymnasium`, `stable-baselines3`, `torch`, `numpy`, `scipy`, `opencv-python`, `open3d`, `pyyaml`, `rich`, `colcon-common-extensions`, `transforms3d` 등 정의.
     * `conda env create -f environment.yml` 및 `conda activate ros2_mujoco_panda_py3_10` 검증.
  2. **ROS2 패키지 인터페이스 생성 (`franka_logistics_msgs`)**:
     * `SortItem.action`, `BinStatusArray.msg`, `BinStatus.msg`, `SafetyStatus.msg`, `MesNotification.srv` 정의 및 `colcon build` 검증.
  3. **9개 핵심 ROS2 패키지 디렉토리 및 메타데이터 스켈레톤 생성**.

---

### 📌 Phase 02: MuJoCo 3.6.x 3D 가상 씬 및 물류 환경 모델링
* **목표**: PRD 작업 공간 레이아웃 명세를 충족하는 통합 MJCF 가상 씬(`scene_dual_panda_logistics.xml`) 완성.
* **세부 작업**:
  1. **MJCF 가상 씬 모델링 (`scene_dual_panda_logistics.xml`)**:
     * Dual Franka Panda 로봇 (좌: 로봇 1, 우: 로봇 2) 및 그리퍼 배치.
     * 중앙 대형 물류 분류 작업대 테이블.
     * 전용 적재함 6개 (좌: A-R, B-B, C-G / 우: A-G, B-R, C-B).
     * 배출 컨베이어 벨트 2개소 (좌측 하단, 우측 상단).
     * 물류 아이템 6종(A-R, B-B, C-G, A-G, B-R, C-B) 및 규격 외 분류 불가품 모델링.
  2. **센서 마운팅 지오메트리 정의**:
     * 각 로봇 EE Wrist `<camera>` (Depth/RGB 렌더링용)
     * 상단/하단 가상 2D LiDAR 센서 기준 위치 정의.
  3. **MuJoCo Viewer 상에서 씬 로딩, 간섭, 관절 한계 검증**.

---

### 📌 Phase 03: MuJoCo ↔ ROS2 Sim Bridge 노드 구현 (가상 하드웨어 드라이버)
* **목표**: MuJoCo 물리 엔진과 ROS2 네트워크를 실시간으로 결합하는 고속 통신 브릿지 노드 구현.
* **세부 작업**:
  1. **1kHz 물리 시뮬레이션 루프**: MuJoCo 3.6.x C API / Python 바인딩 기반 $1\text{ms}$ 주기 루프(`mj_step`).
  2. **Joint State Publisher**: `/robot1/joint_states`, `/robot2/joint_states` (1000Hz, `sensor_msgs/msg/JointState`).
  3. **Joint & Gripper Command Subscriber**: 목표 위치/속도/토크 지령을 받아 `mjData.ctrl`에 실시간 인가.
  4. **Wrist Depth/RGB Camera Publisher**: OpenGL 오프스크린 렌더러 기반 영상 캡처 및 `sensor_msgs/msg/Image` (30Hz, `cv_bridge`) 발행.
  5. **2D LiDAR 레이캐스터**: 상/하단 기준점 평면 방사형 레이캐스팅 및 `sensor_msgs/msg/LaserScan` (20Hz) 발행.

---

### 📌 Phase 04: 로봇 기구학, 궤적 생성 & P&P Action 서버
* **목표**: DLS 역기구학 및 5차 다항식 궤적 생성기를 탑재한 ROS2 Action 기반 Pick & Place 서버 구현.
* **세부 작업**:
  1. **DLS(Damped Least Squares) 수치적 IK 솔버**: 7-DoF 특이점 회피 및 위치 오차 $\le 1\text{mm}$.
  2. **5차 다항식 궤적 생성기 (Quintic Planner)**: 부드러운 S-Curve 가감속 프로파일 산출.
  3. **최단 거리 우선 타겟팅 모듈**: 로봇 Base 기준 최근접 물품 우선 선별.
  4. **`SortItem.action` 서버 구현**: Approach $\rightarrow$ Grasp $\rightarrow$ Lift $\rightarrow$ Transfer $\rightarrow$ Place $\rightarrow$ Home 복귀.

---

### 📌 Phase 05: 비전 분류, 적재함 3D 차지율 스캔 & 컨베이어 배출
* **목표**: Wrist Depth 기반 물류 식별, Place 직후 적재함 3D 점유 부피 적분 및 차지율 산출, 컨베이어 배출 로직 구현.
* **세부 작업**:
  1. **`item_detector_node`**: 물품 바운딩박스/형상/색상 추출 $\rightarrow$ A/B/C 타입 분류 및 불가품 판정.
  2. **`bin_occupancy_scanner_node`**: Place 직후 적재함 상단 Depth 스캔 $\rightarrow$ 3D Point Cloud 복원 $\rightarrow$ 적재 부피 적분 $\rightarrow$ 실시간 차지율(%) 산출 (`/system/bin_occupancy`).
  3. **`conveyor_controller_node`**: 분류 불가품 구역 안착 감지 시 설정 선속도($v_c$)로 외부 연속 이송.

---

### 📌 Phase 06: 듀얼 로봇 FCFS 충돌 방지 & 안전 감독관 구축
* **목표**: 공유 작업 영역 선점 우선순위(FCFS) 회피 조율기 및 다계층 안전 인터록 구현.
* **세부 작업**:
  1. **`dual_robot_coordinator_node` (FCFS 충돌 회피)**: 공유 영역 진입 요청 시 **선착 로봇에게 모션 우선권 부여**, 후착 로봇은 안전 대기 웨이포인트 회피 대기.
  2. **`safety_supervisor_node`**: 작업자 접근 시 감속($\le 250\text{mm/s}$) 및 작업대 진입 시 일시정지, LiDAR Warning($50\%$ 감속) / Danger(E-Stop).
  3. 비상 정지(E-Stop) 소프트웨어 브레이크 및 토크 차단 로직.

---

### 📌 Phase 07: 공정 조율 FSM, MES 인터페이스 & 10분 Auto-Off
* **목표**: 전체 물류 분류 사이클 관리 FSM 및 예외 상황 대응, 상위 MES 연동 구현.
* **세부 작업**:
  1. **`fsm_manager_node`**:
     * 전체 공정 상태 머신 제어.
     * 적재함 90% 도달 시 MES `[WARN_BIN_NEAR_FULL]` 이벤트 발행.
     * 100% 만재 시 `[ERR_BIN_FULL]` 발행 및 해당 품목 분류 대상에서 일시 제외.
     * 작업대 물류 전량 소진 시 일시정지 $\rightarrow$ **10분 미공급 시 자동 셧다운(`Auto-Shutdown`)**.
  2. 상위 MES 연동 Bridge 서비스 클라이언트 구현.

---

### 📌 Phase 08: HMI 제어반 서비스 & RViz2 3D 모니터링 HUD
* **목표**: HMI 스위치 제어반 서비스 및 실시간 상태 모니터링 RViz2 대시보드 구축.
* **세부 작업**:
  1. **`hmi_panel_node`**: `/hmi/power_switch`, `/hmi/start_stop`, `/hmi/estop` 서비스 제공.
  2. **RViz2 3D 오버레이 (`rviz_marker_publisher_node`)**: LiDAR 감지 영역(노랑/빨강) 3D Marker, **6개 적재함 실시간 차지율(%) 게이지 HUD**, 로봇 충돌 간섭 영역 시각화.

---

### 📌 Phase 09: 종합 통합 검증, 5-KPI 평가 & ros2bag 테스트
* **목표**: 통합 런치 실행, PRD 5대 KPI 정량 평가 및 ros2bag 데이터 무결성 검증.
* **세부 작업**:
  1. 통합 실행 런치 (`system_bringup.launch.py`).
  2. PRD KPI 5대 항목 정량 평가 (분류 정확도 $\ge 99.0\%$, 충돌 방지율 $100\%$, 차지율 오차 $\le \pm 5\%$, 만재 품목 제외 $100\%$, 10분 셧다운 정밀도).
  3. 100개 무작위 물품 스트레스 테스트 및 `ros2bag` 기록/분석.
  4. 1차 Rule-based 개발 완료 보고서 작성.

---

### 📌 Phase 10: Gymnasium 연동 & PPO 강화학습(RL) 고도화 (지능형 적응 제어)
* **목표**: Gymnasium 표준 멀티에이전트 환경 구축, PPO 강화학습을 통한 동적 충돌 회피 및 지능형 궤적 최적화 정책 학습 및 ROS2 실시간 배포.
* **세부 작업**:
  1. **Gymnasium 표준 환경 래퍼 (`DualFrankaLogisticsEnv`) 구현**:
     * **Action Space**: 듀얼 로봇 관절 속도/토크 또는 EE 델타 포즈 ($\mathbb{R}^{14+2}$)
     * **Observation Space**: 관절 각도/속도, Wrist Depth Point Cloud, 물류 3D 좌표, LiDAR 이격 거리
     * **보상 함수 (Reward Function)**:
       $$R = r_{\text{sort\_success}} (+100) - r_{\text{collision}} (-200) - r_{\text{safety\_violation}} (-50) - \alpha \cdot \text{StepTime}$$
  2. **PPO (Proximal Policy Optimization) 정책 학습 파이프라인**:
     * `Stable-Baselines3` 기반 분산 병렬 학습 환경 구성 (SubprocVecEnv).
     * 작업대 중앙 공유 영역에서의 듀얼 로봇 간 실시간 자율 회피 및 동적 장애물(작업자 손/몸통) 적응형 우회 궤적 학습.
  3. **ROS2 실시간 추론 노드 (`rl_policy_controller_node`) 구현**:
     * 학습된 Policy 신경망을 ONNX/TorchScript로 변환하여 ROS2 제어 노드에 탑재, $100\text{Hz}$ 실시간 추론 실행.
  4. **Rule-based (Phase 04~06) vs RL Policy (Phase 10) 정량 비교 평가**:
     * 시간당 물류 분류 처리량(Throughput), 충돌 회피 응답 지연 시간, 모션 소비 에너지 효율성 비교 분석 보고서 작성.


