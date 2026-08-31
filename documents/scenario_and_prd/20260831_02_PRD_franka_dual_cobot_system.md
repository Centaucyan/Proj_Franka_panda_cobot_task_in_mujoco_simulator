# [PRD] ROS2 Humble & MuJoCo 기반 Franka Panda 듀얼 협동로봇 물류 분류 및 안전 제어 시뮬레이션 시스템

* **문서 번호**: PRD-LOGISTICS-20260831-02-ROS2
* **작성일**: 2026-08-31
* **버전**: v2.3.0
* **개발 및 운용 환경**: 
  * **OS**: **Ubuntu 22.04 LTS (x86_64)**
  * **ROS 2**: **ROS2 Humble Hawksbill**
  * **Python 가상환경**: **Miniconda (`ros2_mujoco_panda_py3_10`, Python 3.10)**
  * **물리 시뮬레이터**: **MuJoCo 3.6.x**

---

## 1. 프로젝트 개요 (Project Overview)

### 1.1 배경 및 목적
본 프로젝트는 **Ubuntu 22.04**, **ROS2 Humble**, **Miniconda(가상환경: `ros2_mujoco_panda_py3_10`, Python 3.10)** 환경에서 물리 엔진 **MuJoCo 3.6.x**를 연동하여, 2대의 Franka Emika Panda 협동로봇과 작업자가 협업하는 스마트 물류 분류(Sorting) 자동화 시스템을 구축하고 검증하는 것을 목표로 합니다.

로봇 그리퍼의 Wrist Depth 카메라를 통한 다품종 물류(A/B/C 타입, 색상별) 인식 및 실시간 적재함 차지율(%) 3D Depth 스캔, 불량/규격 외 물품의 컨베이어 실시간 배출을 구현합니다. 또한 ROS2의 Topic/Service/Action 및 QoS(Quality of Service) 통신을 활용하여 2D-LiDAR 다계층 안전 인터록, 듀얼 로봇 간 공유 작업 영역 선점 우선순위(FCFS) 충돌 방지, 10분 미공급 시 자동 셧다운 메커니즘을 완성합니다.

### 1.2 핵심 가치
* **ROS2 기반 모듈화 및 확장성**: 인지(Vision), 제어(Motion Control), 안전(Safety Supervisor), 공정 관리(FSM/BehaviorTree), HMI가 완전히 독립된 ROS2 Node로 분리되어 높은 유지보수성과 재사용성 보장.
* **가상환경 (`ros2_mujoco_panda_py3_10`) 관리**: ROS2 Humble 시스템 C-Extension ABI와의 완벽한 호환성 및 독립된 패키지 의존성 격리 환경 제공.
* **지능형 다품종 물류 분류 & 버퍼 관리**:
  * 로봇 1(A-Red, B-Blue, C-Green) / 로봇 2(A-Green, B-Red, C-Blue) 분담 분류.
  * Place 직후 3D Depth 스캔 기반 적재함 차지율(%) 실시간 계산 및 상위 시스템(MES) 90%/100% 이벤트 발행.
  * 분류 불가품 전용 컨베이어 벨트 실시간 배출.
* **다계층 안전 제어 (Safety First)**:
  * 2D-LiDAR 2기를 활용한 속도·이격거리 모니터링(SSM) 및 작업자 접근 시 감속/정지.
  * 듀얼 로봇 간 공유 작업 영역(Shared Workspace) 선점 우선순위(First-Come-First-Served) 회피 제어.
* **자율 복원력 및 자동 절전**: 작업대 물류 소진 시 대기 및 10분 이상 미공급 시 홈 포지션 복귀 후 자동 셧다운(Auto-Shutdown).

---

## 2. 시스템 구성 및 아키텍처 (System Architecture)

### 2.1 하드웨어 & 센서 사양
1. **협동로봇 (Dual Robot Setup)**:
   * **로봇 1 (좌측 로봇)**: Franka Emika Panda (7-DoF + 2-Finger Gripper)
   * **로봇 2 (우측 로봇)**: Franka Emika Panda (7-DoF + 2-Finger Gripper)
2. **비전 센서 (Vision Sensors)**:
   * **Wrist Depth Camera (2기)**: 각 로봇 EE 중앙부에 1기씩 장착 (`sensor_msgs/msg/Image`, `CameraInfo`)
3. **안전 센서 (Safety Sensors)**:
   * **2D Safety LiDAR (2기)**: 상단/하단 작업자 이동 측면에 각 1기 설치 (`sensor_msgs/msg/LaserScan`)
4. **HMI 제어반**:
   * `On/Off`, `Start/Stop`, `Emergency-Stop` 스위치
5. **이송 설비**:
   * **배출 컨베이어 벨트 (2개소)**: 좌측 하단(로봇 1용), 우측 상단(로봇 2용)
6. **외부 연동**:
   * 상위 공정 관리 시스템(MES/WMS) 연동 ROS2 Service/Topic 인터페이스

---

### 2.2 ROS2 노드 아키텍처 다이어그램

```
 ┌───────────────────────────────────────────────────────────────────────────┐
 │                      MuJoCo Simulation Bridge Node (v3.6.x)               │
 │  • Physics Step (1kHz)      • Joint State Publisher                       │
 │  • Wrist Cameras Render     • 2D LiDAR Ray-casting                        │
 └──────┬────────────────────┬─────────────────────┬───────────────────┬─────┘
        │ /robot1/joint_states│ /wrist_camera/depth │ /lidar_top/scan   │
        │ /robot2/joint_states│ /wrist_camera/rgb   │ /lidar_bottom/scan│
        ▼                    ▼                     ▼                   ▼
 ┌──────────────┐     ┌──────────────┐      ┌──────────────┐    ┌──────────────┐
 │Motion Cont-  │     │Vision Proce- │      │Safety Super- │    │Conveyor Cont-│
 │roller Node   │     │ssing Node    │      │visor Node    │    │roller Node   │
 │(IK & Traj)   │     │(Detect & Scan│      │(SSM, LiDAR,  │    │(Velocity     │
 │              │     │Bin Occupancy)│      │Dual-FCFS)    │    │Control)      │
 └──────┬───────┘     └──────┬───────┘      └──────┬───────┘    └──────┬───────┘
        │                    │                     │                   │
        └────────────────────┼─────────────────────┼───────────────────┘
                             ▼                     ▼
               ┌─────────────────────────────────────────┐
               │    FSM / BehaviorTree Coordinator       │
               │    (Sorting Logic & MES Interface)      │
               └────────────────────┬────────────────────┘
                                    │
                                    ▼
               ┌─────────────────────────────────────────┐
               │    HMI & RViz2 3D Dashboard Node        │
               │    (Switches, Bin HUD, 3D Zones)        │
               └─────────────────────────────────────────┘
```

---

### 2.3 작업 공간(Workspace) 레이아웃

```
               [ 작업자 (상단 이동/작업 구역) ]    [ 컨베이어 벨트 ↑ ]
                                                   [ 분류 불가품     ]
┌──────────────┐ ┌────────────────────────┐ ┌──────────────┐
│ 물류 A-Red   │ │                        │ │ 물류 A-Green │
│ 적재함       │ │                        │ │ 적재함       │
├──────────────┤ │                        │ ├──────────────┤
│ 물류 B-Blue  │ (로봇 1)  물류 분류      (로봇 2)│ 물류 B-Red   │
│ 적재함       │ │         작업대         │ │ 적재함       │
├──────────────┤ │                        │ ├──────────────┤
│ 물류 C-Green │ │                        │ │ 물류 C-Blue  │
│ 적재함       │ │                        │ │ 적재함       │
└──────────────┘ └────────────────────────┘ └──────────────┘
[ 분류 불가품  ]        [ 작업자 (하단 이동/작업 구역) ]
[ 컨베이어 벨트↓]
```

---

## 3. 주요 ROS2 인터페이스 정의 (Topics, Services, Actions)

| 분류 | 인터페이스 명칭 | 메시지/타입 | 설명 |
| :--- | :--- | :--- | :--- |
| **Topic (Pub)** | `/robot1/joint_states`<br>`/robot2/joint_states` | `sensor_msgs/msg/JointState` | 로봇 조인트 위치/속도/토크 (1000Hz) |
| **Topic (Pub)** | `/robot1/wrist_camera/depth/image_raw`<br>`/robot2/wrist_camera/depth/image_raw` | `sensor_msgs/msg/Image` | 그리퍼 Wrist Depth 영상 (30Hz) |
| **Topic (Pub)** | `/lidar_top/scan`<br>`/lidar_bottom/scan` | `sensor_msgs/msg/LaserScan` | 2D LiDAR 레이캐스팅 데이터 (20Hz) |
| **Topic (Pub)** | `/system/bin_occupancy` | `custom_msgs/msg/BinStatusArray` | 6개 적재함별 실시간 차지율(%) 및 상태 |
| **Topic (Pub)** | `/safety/zone_status` | `custom_msgs/msg/SafetyStatus` | LiDAR 경고/위험 구역 침범 및 E-Stop 상태 |
| **Action** | `/robot1/execute_sort`<br>`/robot2/execute_sort` | `custom_msgs/action/SortItem` | 개별 물류 파지 및 분류 안착 액션 |
| **Service** | `/hmi/power_switch`<br>`/hmi/start_stop`<br>`/hmi/estop` | `std_srvs/srv/SetBool`<br>`std_srvs/srv/Trigger` | HMI 제어반 이벤트 트리거 |
| **Service** | `/mes/notify_event` | `custom_msgs/srv/MesNotification` | MES 알림 (90% 경고, 만재, 물품 소진) |

---

## 4. 기능 요구사항 (Functional Requirements)

### [FR-01] 듀얼 로봇 물류 분류 및 픽앤플레이스 워크플로우 (FSM)
* **FR-01-1 (물류 탐색 및 우선순위)**:
  * 로봇 1, 2는 Wrist Depth 카메라를 통해 작업대 위의 물품을 스캔하고, **자신의 Base 기준 유클리디안 최단 거리에 있는 물품을 우선 목표**로 선정한다.
* **FR-01-2 (로봇 1 분류 로직)**:
  1. `A-Red`, `B-Blue`, `C-Green` 인식 시 해당 전용 적재함으로 Pick & Place.
  2. 비정상, 미식별, 규격 외 물품 판정 시 좌측 하단 `분류 불가품 구역(컨베이어)`으로 Pick & Place.
  3. Place 완료 즉시 그리퍼 Depth 카메라로 적재함을 스캔하여 **실시간 차지율(%)**을 산출 및 토픽 발행.
  4. 최저 부하 초기 자세(Home)로 복귀 후 다음 물품 사이클 수행.
* **FR-01-3 (로봇 2 분류 로직)**:
  1. `A-Green`, `B-Red`, `C-Blue` 인식 시 해당 전용 적재함으로 Pick & Place.
  2. 비정상, 미식별, 규격 외 물품 판정 시 우측 상단 `분류 불가품 구역(컨베이어)`으로 Pick & Place.
  3. Place 완료 즉시 그리퍼 Depth 카메라로 적재함을 스캔하여 **실시간 차지율(%)**을 산출 및 토픽 발행.
  4. 최저 부하 초기 자세(Home)로 복귀 후 다음 물품 사이클 수행.
* **FR-01-4 (컨베이어 벨트 실시간 배출)**:
  * 컨베이어 노드는 분류 불가품 구역에 안착된 물체를 감지하여 설정된 선속도($v_c$)로 외부로 연속 이송/배출한다.

---

### [FR-02] 적재함 용량 관리 및 MES 연동
* **FR-02-1 (90% 사전 경고)**:
  * 특정 적재함의 차지율이 **$90\%$ 이상**에 도달하면 `/mes/notify_event`를 통해 `[WARN_BIN_NEAR_FULL]` 이벤트를 발행한다.
* **FR-02-2 (100% 만재 처리 및 분류 제외)**:
  * 특정 적재함이 만재($100\%$)되면 `[ERR_BIN_FULL]` 이벤트를 발행한다.
  * **해당 적재함이 작업자에 의해 교체/비워질 때까지, FSM은 해당 물류 품목을 타겟팅 대상에서 일시 제외**한다.

---

### [FR-03] HMI 제어 및 시스템 라이프사이클 (Lifecycle)
* **FR-03-1 (On/Off 제어)**:
  * **전원 On**: 노드 Lifecycle `Configure` $\rightarrow$ `Activate` 전환, 센서/로봇 통신 점검 및 6개 적재함 초기 점유율 스캔 후 대기.
  * **전원 Off**: 진행 중인 Pick & Place 동작을 안전 완료 $\rightarrow$ 최저 부하 홈 포지션 복귀 $\rightarrow$ ROS2 노드 `Deactivate` 및 안전 셧다운.
* **FR-03-2 (Start/Stop 운전)**:
  * `Start`: 자동 물류 분류 루프 시작.
  * `Stop`: 현재 모션 궤적을 보존하며 즉각 감속 정지(Pause), `Start` 시 궤적 스무스 재개(Resume).
* **FR-03-3 (Emergency-Stop)**:
  * 비상 정지 서비스 호출 시 하드웨어 토크 차단 명령을 발행하여 최단 시간 내 완전 정지.

---

### [FR-04] 안전 및 예외 상황 인터록 (Safety Supervisor)

* **FR-04-1 (작업자 작업대 접근 감지)**:
  * 작업자가 작업대 주변으로 접근 시 로봇 1, 2는 이송 속도를 안전 속도($\le 250\text{mm/s}$)로 감속하고, 작업대 간섭 반경 진입 시 **일시 정지(Pause)**한다. 이탈 시 자동 재개한다.
* **FR-04-2 (2D-LiDAR 다계층 감시)**:
  * **경고 구역(Warning Zone)** 침범: 속도 $50\%$ 감속 지령 발행.
  * **위험 구역(Danger Zone)** 침범: 즉시 Emergency-Stop 지령 발행.
* **FR-04-3 (듀얼 로봇 공유 작업 영역 FCFS 충돌 방지)**:
  * 작업대 중앙 공유 영역 진입 시 **먼저 진입을 요청한 로봇에게 모션 우선권**을 부여한다.
  * 후착 로봇은 안전 회피 웨이포인트로 이동하여 대기하고, 선착 로봇이 공유 영역을 이탈하면 즉시 작업을 재개한다.
* **FR-04-4 (작업대 물류 전량 소진 및 자동 셧다운)**:
  * 작업대 물품이 0개가 되면 일시정지 후 MES에 `[REQ_REFILL_TABLE]` 알림을 발송한다.
  * **10분(시뮬레이션 시간) 동안 물품이 채워지지 않으면**, 로봇을 홈 포지션으로 이동시킨 후 시스템을 **자동 Off(Auto-Shutdown)**한다.

---

## 5. 비기능 요구사항 (Non-Functional Requirements)

### 5.1 성능 및 실시간성 (Performance)
* **NFR-PER-01 (제어 루프 주기)**: MuJoCo 3.6.x 물리 시뮬레이션 및 로봇 저수준 제어 루프는 **$1\text{kHz}\ (1\text{ms})$**를 유지한다.
* **NFR-PER-02 (비전 처리 및 Depth 스캔)**: Wrist Depth 처리 및 3D 적재율 계산 파이프라인은 **$\le 33\text{ms}\ (30\text{fps})$** 이내에 완료한다.
* **NFR-PER-03 (ROS2 통신 QoS)**: 안전 제어 및 E-Stop 토픽은 `Reliable` & `Transient Local` QoS를 적용하여 패킷 유실을 방지한다.
* **NFR-PER-04 (실시간 비율)**: 3D RViz2 및 MuJoCo 시각화 상태에서 **$\text{Real-Time Factor (RTF)} \ge 1.0$**을 유지한다.

### 5.2 안전성 및 신뢰성 (Safety & Reliability)
* **NFR-SAF-01 (ISO/TS 15066 SSM 준수)**: 실시간 작업자 이격 거리에 따른 가변 감속 프로파일 적용.
* **NFR-SAF-02 (충돌 무결성)**: 로봇-로봇 간섭 시나리오에서 충돌 발생률 **$0.0\%$ (충돌 0건)**를 보장한다.
* **NFR-SAF-03 (비상 정지 제동)**: E-Stop 수신 시 **$150\text{ms}$ 이내**에 로봇을 완전 정지시킨다.

### 5.3 가시화 및 로깅 (HMI & Visualization)
* **NFR-USA-01 (RViz2 3D 모니터링)**: 듀얼 로봇 모델, LiDAR 레이캐스트 데이터, 6개 적재함 실시간 점유율 게이지, 안전 구역 바운더리를 RViz2에 오버레이 렌더링한다.
* **NFR-USA-02 (ROS2 Bag & JSONL 로깅)**: 모든 센서 토픽 및 FSM 상태 전이 이벤트를 `ros2bag` 및 `.jsonl` 포맷으로 기록한다.

---

## 6. 성공 지표 (KPI / Success Criteria)

| 검증 항목 | 목표 기준치 | 검증 방법 |
| :--- | :--- | :--- |
| **물류 분류 정확도** | $\ge 99.0\%$ | 6종 물류 및 불가품 100회 무작위 분류 테스트 |
| **듀얼 로봇 충돌 방지율** | $100\%$ (충돌 0건) | 중앙 동시 진입 시나리오 50회 집중 검증 |
| **적재함 차지율 산출 오차** | $\le \pm 5\%$ | 실 부피 대비 3D Depth 스캔 계산 오차 계측 |
| **만재 시 품목 제외 정상률** | $100\%$ | 적재함 $100\%$ 도달 시 해당 품목 스킵 확인 |
| **10분 미공급 셧다운 정밀도** | 10분 $\pm 1$초 | 물품 소진 타이머 만료 후 순차 전원 Off 검증 |

---

## 7. 부록: 관련 기술 표준 및 레퍼런스
* **ISO 10218-1/2:2011**: Robots and robotic devices — Safety requirements for industrial robots
* **ISO/TS 15066:2016**: Robots and robotic devices — Collaborative robots
* **ISO 13849-1:2023**: Safety of machinery — Safety-related parts of control systems
* **ROS 2 Quality of Service (QoS) Policies**
