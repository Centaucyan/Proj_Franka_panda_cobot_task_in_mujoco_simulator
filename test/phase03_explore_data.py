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
