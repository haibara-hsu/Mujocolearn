import mujoco
import mujoco.viewer
import time

xml = """
<mujoco>
  <option gravity="0 0 -9.81"/>
  
  <worldbody>
    <light pos="0 0 3"/>
    
    <body pos="0 0 2"> 
        <geom type="sphere" size="0.1" rgba="0.8 0.8 0.8 1"/> 
        
        <body pos="0 0 0">
            <joint type="hinge" axis="0 1 0"/>
            <geom type="capsule" fromto="0 0 0  0 0 -1" size="0.05" rgba="1 0 0 1"/> 
            
            <body pos="0 0 -1">
                <joint type="hinge" axis="0 1 0" damping="0.1"/>
                <geom type="capsule" fromto="0 0 0  0 0 -1" size="0.05" rgba="0 1 0 1"/>
            </body>
            
        </body>
    </body>
  </worldbody>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    print("🎥 雙節棍模擬開始！")
    # 如果畫面還是黑的，按一下鍵盤上的 Backspace 鍵 (重置攝影機)
    
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.005)