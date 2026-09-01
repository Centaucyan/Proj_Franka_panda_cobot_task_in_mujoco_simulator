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
