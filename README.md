# Proj_Franka_panda_cobot_task_in_mujoco_simulator
* Update: 2026.08.31.

## 1. 개요
* Mujoco 가상환경에서 Franka Emika Panda 로봇팔과 제조 공정 작업자와 협동하여 완성품을 제작
---

## 2. 작업 공간
![workspace](./documents/scenario_and_prd/20260831_01_work_space.png)
---

## 3. 환경
* **OS:** Windows10
* **Python versioin:** 3.10+(conda 가상환경)
---

## 4. Pre-installed
* 가상환경 활성화 후
```bash
python -m pip install mujoco gymnasium numpy scipy opencv-python open3d torch stable-baselines3 pyyaml matplotlib rich
```
```
# --- Simulation & RL Environment ---
mujoco>=3.1.0
gymnasium>=0.29.0
# --- Math & Kinematics ---
numpy>=1.24.0
scipy>=1.10.0
# --- Vision & 3D Processing ---
opencv-python>=4.8.0
open3d>=0.18.0
# --- Reinforcement Learning (PPO) ---
torch>=2.0.0
stable-baselines3>=2.2.0
# --- Utilities & Visualization ---
pyyaml>=6.0
matplotlib>=3.7.0
rich>=13.0.0
```

### 📚 패키지별 주요 역할 및 설명
* **Simulation & RL Environment**
  * `mujoco`: DeepMind의 고성능 물리 시뮬레이션 엔진. Franka Panda 로봇의 다관절 동역학, 접촉 역학, 1kHz 고속 제어 연산 및 내장 3D 렌더링을 지원합니다.
  * `gymnasium`: Farama Foundation의 강화학습 표준 환경 인터페이스. 시뮬레이션 환경의 관측값(Observation), 행동(Action), 보상(Reward)을 표준화하여 상태 머신(FSM) 제어 및 추후 PPO 강화학습 확장을 지원합니다.
* **Math & Kinematics**
  * `numpy`: 다차원 배열 처리, 행렬 연산 및 관절 각도/토크 벡터 연산의 핵심 라이브러리입니다.
  * `scipy`: 3D 공간 회전 변환(`Rotation`), 5차 다항식 궤적 보간(Spline), 최적화 등 수치 기구학 계산에 활용됩니다.
* **Vision & 3D Processing**
  * `opencv-python`: 그리퍼 장착 Depth 카메라의 영상 전처리, ROI 추출 및 2D/3D 부품 불량 검수 알고리즘을 구현합니다.
  * `open3d`: Depth 맵 데이터를 3D 점군(Point Cloud)으로 변환하고, 평면 분할 및 3D 바운딩 박스를 계산하여 작업 영역 간섭을 감지합니다.
* **Reinforcement Learning (PPO)**
  * `torch (PyTorch)`: 딥러닝 텐서 연산 및 정책 신경망 학습 백엔드로 활용됩니다 (GPU 가속 지원).
  * `stable-baselines3`: 검증된 고성능 강화학습(PPO, SAC 등) 알고리즘 구현체로, 조립 및 충돌 회피 모션 학습에 활용됩니다.
* **Utilities & Visualization**
  * `pyyaml`: 작업 공간 영역 좌표, 안전 반경, 각종 임계치 파라미터를 담은 `config.yaml` 설정 파일을 로드합니다.
  * `matplotlib`: 로봇의 속도 프로파일, 비상 제동 정지 거리 계측치, KPI 성능 평가 결과를 2D 그래프로 시각화합니다.
  * `rich`: 콘솔 터미널에 공정 진행 상태, FSM 상태 전이 및 시스템 알림/경보를 직관적인 컬러로 출력합니다.
---

## 5. Reference
* https://github.com/google-deepmind/mujoco_menagerie.git
---