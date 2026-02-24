import mujoco
import mujoco.viewer
import time
import numpy as np

# ==========================================
# XML 修改：讓電池變成 "mocap" 物體
# 這樣我們才能用程式碼強制改變它的位置 (模擬被抓起來)
# ==========================================
xml = """
<mujoco>
  <option timestep="0.005" gravity="0 0 -9.81"/>
  <visual>
     <global azimuth="120" elevation="-20"/>
  </visual>

  <worldbody>
    <light pos="0 0 2" dir="0 0 -1"/>
    <geom type="plane" size="2 2 0.1" rgba="0.9 0.9 0.9 1"/>
    
    <body pos="0.5 0 0.2">
        <geom type="box" size="0.2 0.3 0.2" rgba="0.5 0.5 0.5 1"/>
    </body>

    <body name="battery" mocap="true" pos="0.5 0 0.42">
        <geom type="box" size="0.05 0.08 0.02" rgba="0 1 0 1"/> 
    </body>

    <body pos="0 0 0.1">
        <geom type="cylinder" size="0.08 0.1" rgba="0.3 0.3 0.3 1"/>
        <body pos="0 0 0.1">
            <joint name="J1" axis="0 0 1" range="-3.14 3.14"/>
            <geom type="cylinder" size="0.07 0.08" fromto="0 0 0  0 0 0.15" rgba="0.2 0.6 0.8 1"/>
            <body pos="0 0.1 0.15">
                <joint name="J2" axis="0 1 0" range="-3.14 3.14"/>
                <geom type="capsule" fromto="0 0 0  0 0 0.4" size="0.06" rgba="0.8 0.2 0.2 1"/>
                <body pos="0 -0.1 0.4">
                    <joint name="J3" axis="0 1 0" range="-3.14 3.14"/>
                    <geom type="capsule" fromto="0 0 0  0 0 0.35" size="0.05" rgba="0.8 0.5 0.2 1"/>
                    <body pos="0 0 0.35">
                        <joint name="J4" axis="0 1 0" range="-3.14 3.14"/>
                        <geom type="cylinder" fromto="0 0 0  0 0.1 0" size="0.04" rgba="0.2 0.8 0.2 1"/>
                        <body pos="0 0.1 0">
                            <joint name="J5" axis="0 0 1" range="-3.14 3.14"/>
                            <geom type="cylinder" fromto="0 0 0  0 0 0.1" size="0.04" rgba="0.2 0.8 0.2 1"/>
                            <body pos="0 0 0.1">
                                <joint name="J6" axis="0 1 0" range="-3.14 3.14"/>
                                <geom type="cylinder" size="0.04 0.02" axisangle="1 0 0 90" rgba="0.9 0.9 0.1 1"/>
                                <site name="end_effector" pos="0 0.05 0" size="0.02" rgba="1 0 0 1"/>
                            </body>
                        </body>
                    </body>
                </body>
            </body>
        </body>
    </body>
  </worldbody>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)
site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "end_effector")
dof = model.nv

# ==========================================
# 路徑規劃：標準的 Pick & Place 流程
# ==========================================
waypoints = [
    np.array([0.5, 0.0, 0.6]),  # 0. 準備：電池上方
    np.array([0.5, 0.0, 0.45]), # 1. 下降：接觸電池 (GRASP!)
    np.array([0.5, 0.0, 0.6]),  # 2. 抬起：回到高空
    np.array([0.0, -0.4, 0.6]), # 3. 移動：到左側卸貨區上方
    np.array([0.0, -0.4, 0.45]),# 4. 下降：卸貨 (RELEASE!)
    np.array([0.0, -0.4, 0.6])  # 5. 離開
]

current_target_index = 0
accuracy_threshold = 0.02 

# 🧲 磁吸狀態變數
is_gripping = False 

with mujoco.viewer.launch_passive(model, data) as viewer:
    start = time.time()
    
    # 初始化一個比較好動的姿勢
    data.qpos[1] = -1.0 
    data.qpos[2] = 1.0
    mujoco.mj_step(model, data)

    print("🚀 取放任務開始！觀察電池怎麼動...")

    while viewer.is_running():
        step_start = time.time()
        
        # --- 1. 狀態機邏輯 ---
        target_pos = waypoints[current_target_index]
        current_pos = data.site_xpos[site_id]
        dist = np.linalg.norm(target_pos - current_pos)
        
        # 判斷是否到達路徑點
        if dist < accuracy_threshold:
            print(f"✅ 到達點 {current_target_index}")
            
            # --- 關鍵邏輯：在特定的點開關磁鐵 ---
            if current_target_index == 1: # 在點 1 (電池位置) -> 吸住
                is_gripping = True
                print("🧲 磁鐵開啟！吸住電池！")
                # 把末端點變成綠色，代表吸住了
                model.site_rgba[site_id] = [0, 1, 0, 1] 

            elif current_target_index == 4: # 在點 4 (卸貨位置) -> 放開
                is_gripping = False
                print("💨 磁鐵關閉！放下電池！")
                # 把末端點變回紅色
                model.site_rgba[site_id] = [1, 0, 0, 1]

            # 切換到下一個目標
            current_target_index += 1
            if current_target_index >= len(waypoints):
                print("🎉 搬運完成！休息一下...")
                current_target_index = 0
                is_gripping = False # 重置
                # 把電池瞬移回原點，準備下一輪表演
                data.mocap_pos[0] = [0.5, 0, 0.42] 
                time.sleep(1)

        # --- 2. 執行磁吸效果 ---
        if is_gripping:
            # 如果磁鐵開著，強制把電池的位置 = 手臂末端的位置
            # 這樣看起來就像是電池黏在手上一樣
            # 我們要把電池放在手下面一點點 (Z軸 - 0.02)
            grip_offset = np.array([0, 0, -0.02])
            data.mocap_pos[0] = current_pos + grip_offset

        # --- 3. IK 運動控制 (跟之前一樣) ---
        error = target_pos - current_pos
        jacp = np.zeros((3, dof))
        mujoco.mj_jacSite(model, data, jacp, None, site_id)
        
        lambda_val = 0.05
        J_T = jacp.T
        delta_q = J_T @ np.linalg.inv(jacp @ J_T + lambda_val * np.eye(3)) @ error
        
        max_speed = 0.8 # 稍微加速一點
        if np.linalg.norm(delta_q) > max_speed:
            delta_q = delta_q / np.linalg.norm(delta_q) * max_speed

        data.qpos[:] += delta_q * 0.1
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.005)