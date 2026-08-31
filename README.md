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