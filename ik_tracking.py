import mujoco
import mujoco.viewer
import time
import numpy as np # <--- IK 需要用到矩陣運算

# ==========================================
# XML: 我們多加一個紅色的球 (Target)
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
    
    <body name="target_ball" mocap="true" pos="0.4 0 0.4">
        <geom type="sphere" size="0.03" rgba="1 0 0 0.5"/> </body>

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
                                <site name="end_effector" pos="0 0.05 0" size="0.01" rgba="0 1 1 1"/>
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

# 取得 ID
site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "end_effector")
target_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "target_ball")
dof = model.nv # 自由度 (6軸)

with mujoco.viewer.launch_passive(model, data) as viewer:
    start = time.time()
    
    # 初始化一個站立的姿勢，避免一開始就躺平
    data.qpos[1] = -1.57 
    mujoco.mj_step(model, data)

    print("🖱️ 請在視窗中，雙擊那個紅色球，然後按住右鍵拖曳它！手臂會跟著動！")

    while viewer.is_running():
        step_start = time.time()

        # 1. 取得目標位置 (紅色球)
        target_pos = data.mocap_pos[0] 

        # 2. 取得目前手臂末端位置
        current_pos = data.site_xpos[site_id]

        # 3. 計算誤差 (Error)
        error = target_pos - current_pos

        # 4. 計算 Jacobian (J)
        # J 是一個 3x6 的矩陣，描述「關節速度」跟「末端速度」的關係
        jacp = np.zeros((3, dof)) # 只看位置 (Position) 的 Jacobian
        mujoco.mj_jacSite(model, data, jacp, None, site_id)

        # 5. [核心數學] 求解逆運動學
        # 我們用「偽逆矩陣 (Pseudo-Inverse)」法： dq = J_inv * error
        # 這會告訴我們：為了消除這個誤差，每個關節該轉多少 (dq)
        
        # 為了避免奇異點 (Singularity) 手臂發瘋，加一點點阻尼 (damping)
        lambda_val = 0.01
        J_T = jacp.T
        # Damped Least Squares 公式 (看不懂沒關係，這是標準解法)
        delta_q = J_T @ np.linalg.inv(jacp @ J_T + lambda_val * np.eye(3)) @ error

        # 6. 更新關節角度
        # 我們不直接設 data.ctrl，而是直接把角度加進去 (積分)
        data.qpos[:] += delta_q * 0.1 # 0.5 是一個縮放係數，越小越滑順

        # 物理推進
        mujoco.mj_step(model, data)
        viewer.sync()
        
        # 保持畫面幀率
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)