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
