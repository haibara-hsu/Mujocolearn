import mujoco
import mujoco.viewer
import time
import math
import csv  # <--- 1. 引入 CSV 套件，用來存檔

# ==========================================
# XML 定義 (跟之前一樣，但我們確保有 camera_eye 這個追蹤點)
# ==========================================
xml = """
<mujoco>
  <option timestep="0.005" gravity="0 0 -9.81"/>
  <visual>
     <global azimuth="140" elevation="-30"/>
  </visual>
  <worldbody>
    <light pos="0 0 2" dir="0 0 -1"/>
    <geom type="plane" size="2 2 0.1" rgba="0.9 0.9 0.9 1"/>
    
    <body pos="0 0 0.1">
        <geom type="cylinder" size="0.08 0.1" rgba="0.3 0.3 0.3 1"/>
        <body pos="0 0 0.1">
            <joint name="J1_Waist" type="hinge" axis="0 0 1" range="-3.14 3.14"/>
            <geom type="cylinder" size="0.07 0.08" fromto="0 0 0  0 0 0.15" rgba="0.2 0.6 0.8 1"/>
            <body pos="0 0.1 0.15">
                <joint name="J2_Shoulder" type="hinge" axis="0 1 0" range="-3.14 3.14"/>
                <geom type="capsule" fromto="0 0 0  0 0 0.4" size="0.06" rgba="0.8 0.2 0.2 1"/>
                <body pos="0 -0.1 0.4">
                    <joint name="J3_Elbow" type="hinge" axis="0 1 0" range="-3.14 3.14"/>
                    <geom type="capsule" fromto="0 0 0  0 0 0.35" size="0.05" rgba="0.8 0.5 0.2 1"/>
                    <body pos="0 0 0.35">
                        <joint name="J4_Wrist1" type="hinge" axis="0 1 0" range="-3.14 3.14"/>
                        <geom type="cylinder" fromto="0 0 0  0 0.1 0" size="0.04" rgba="0.2 0.8 0.2 1"/>
                        <body pos="0 0.1 0">
                            <joint name="J5_Wrist2" type="hinge" axis="0 0 1" range="-3.14 3.14"/>
                            <geom type="cylinder" fromto="0 0 0  0 0 0.1" size="0.04" rgba="0.2 0.8 0.2 1"/>
                            <body pos="0 0 0.1">
                                <joint name="J6_Wrist3" type="hinge" axis="0 1 0" range="-3.14 3.14"/>
                                <geom type="cylinder" size="0.04 0.02" axisangle="1 0 0 90" rgba="0.9 0.9 0.1 1"/>
                                <site name="camera_eye" pos="0 0.05 0" size="0.02" rgba="1 0 0 1"/>
                            </body>
                        </body>
                    </body>
                </body>
            </body>
        </body>
    </body>
  </worldbody>
  
  <actuator>
    <position joint="J1_Waist"    kp="500"  kv="50"/>
    <position joint="J2_Shoulder" kp="1000" kv="100"/>
    <position joint="J3_Elbow"    kp="800"  kv="80"/>
    <position joint="J4_Wrist1"   kp="300"  kv="30"/>
    <position joint="J5_Wrist2"   kp="300"  kv="30"/>
    <position joint="J6_Wrist3"   kp="300"  kv="30"/>
  </actuator>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

# 取得末端追蹤點的 ID
tip_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "camera_eye")

# ==========================================
# 2. 開啟 CSV 檔案準備寫入
# ==========================================
filename = "robot_data.csv"
print(f"🎥 模擬開始！數據將寫入 {filename}")

with open(filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    # 寫入標題列 (Header)
    writer.writerow(['Time', 'Tip_X', 'Tip_Y', 'Tip_Z', 'J1_Angle', 'J2_Angle', 'J3_Angle'])

    with mujoco.viewer.launch_passive(model, data) as viewer:
        start = time.time()
        
        while viewer.is_running():
            now = time.time() - start
            slow_t = now * 0.2 # 慢動作，避免抽動
            
            # --- 運動控制 (數值已修正為弧度制) ---
            # J1 (旋轉): 左右來回 
            data.ctrl[0] = 1.5 * math.sin(slow_t)  # 1.5 rad 約 85度
            
            # J2 (大臂): 上下擺動
            # 注意：這裡不能用 20，那會轉 1000度。改成 0.5 rad (約30度)
            data.ctrl[1] = -1.57 + 0.5 * math.sin(slow_t * 0.5) 
            
            # J3 (小臂): 稍微動一下
            data.ctrl[2] = math.sin(slow_t * 0.8)
            
            # J4~J6: 手腕動作
            data.ctrl[3] = math.sin(slow_t)
            data.ctrl[4] = math.cos(slow_t)
            
            # --- 物理計算 ---
            mujoco.mj_step(model, data)
            
            # --- 關鍵：紀錄數據 ---
            # 取得末端座標
            pos = data.site_xpos[tip_id]
            # 取得關節角度 (qpos)
            joints = data.qpos
            
            # 每 0.1 秒存一次檔 (避免檔案太大，也不需要存太快)
            # 我們利用 frame 計數或者簡單的時間判斷
            if int(now * 100) % 10 == 0: 
                # 寫入一行數據
                writer.writerow([
                    round(now, 2),          # 時間
                    round(pos[0], 4),       # X
                    round(pos[1], 4),       # Y
                    round(pos[2], 4),       # Z
                    round(joints[0], 2),    # J1 角度
                    round(joints[1], 2),    # J2 角度
                    round(joints[2], 2)     # J3 角度
                ])
                
                # 同步印在終端機讓你看
                print(f"📍 [{now:.1f}s] Tip: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")

            viewer.sync()
            time.sleep(0.005)

print("💾 模擬結束，數據已存檔！")