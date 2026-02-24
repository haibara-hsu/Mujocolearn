import mujoco
import mujoco.viewer
import time
import math

# --- 定義工業手臂模型 ---
# 注意：我在最末端加了一個 <site name="end_effector"/>
# 這就像是在機器人手上貼一個「追蹤貼紙」，讓我們方便讀取座標
xml = """
<mujoco>
  <option timestep="0.005" gravity="0 0 -9.81"/>
  <visual>
     <global azimuth="120" elevation="-20"/>
  </visual>
  <worldbody>
    <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
    <geom type="plane" size="1 1 0.1" rgba=".9 .9 .9 1"/>
    
    <body pos="0 0 0.1">
        <geom type="cylinder" size="0.1 0.1" rgba="0.5 0.5 0.5 1"/>
        
        <body pos="0 0 0.1">
            <joint name="joint1" type="hinge" axis="0 0 1" range="-3.14 3.14"/>
            <geom type="capsule" fromto="0 0 0 0 0 0.4" size="0.05" rgba="0.8 0.2 0.2 1"/>
            
            <body pos="0 0 0.4">
                <joint name="joint2" type="hinge" axis="0 1 0" range="-2.0 2.0"/>
                <geom type="capsule" fromto="0 0 0 0.4 0 0" size="0.04" rgba="0.2 0.8 0.2 1"/>
                
                <body pos="0.4 0 0">
                    <joint name="joint3" type="hinge" axis="0 1 0" range="-2.0 2.0"/>
                    <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.03" rgba="0.2 0.2 0.8 1"/>
                    
                    <body pos="0.3 0 0">
                         <geom type="sphere" size="0.05" rgba="1 1 0 1"/>
                         <site name="tip" pos="0 0 0" size="0.01" rgba="1 0 0 1"/>
                    </body>
                </body>
            </body>
        </body>
    </body>
  </worldbody>
  
  <actuator>
    <motor joint="joint1" gear="50"/> <motor joint="joint2" gear="50"/>
    <motor joint="joint3" gear="50"/>
  </actuator>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

# 取得 "tip" 追蹤點的 ID，這樣之後查詢比較快
tip_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "tip")

with mujoco.viewer.launch_passive(model, data) as viewer:
    print("🎥 數據監控模式啟動！看終端機的座標變化")
    start_time = time.time()
    
    while viewer.is_running():
        now = time.time() - start_time
        
        # --- 1. 讓機器人做一點規律運動 ---
        # J1 (旋轉): 左右來回掃描
        data.ctrl[0] = 10 * math.sin(now) 
        # J2 (大臂): 上下擺動
        data.ctrl[1] = 20 * math.sin(now * 0.5) + 10 # +10 是為了抵抗重力
        # J3 (小臂): 固定一點
        data.ctrl[2] = 5
        
        # --- 2. 物理計算 ---
        mujoco.mj_step(model, data)
        
        # --- 3. [關鍵技術] 獲取正運動學 (FK) 數據 ---
        # data.site_xpos 存放了所有 site 的世界座標 (x, y, z)
        current_pos = data.site_xpos[tip_id]
        
        # 每 0.5 秒印出一次座標，才不會洗版
        if int(now * 10) % 5 == 0:
            print(f"📍 手臂末端座標: X={current_pos[0]:.2f}, Y={current_pos[1]:.2f}, Z={current_pos[2]:.2f}")

        viewer.sync()
        time.sleep(0.005)