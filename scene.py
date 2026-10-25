import mujoco
import mujoco.viewer
import math
import time

# Load the model from your XML file
model = mujoco.MjModel.from_xml_path('5RPlanar.xml')
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.time()

        mujoco.mj_step(model, data)

        # 5. Sync the viewer with the updated simulation data
        viewer.sync()

        
