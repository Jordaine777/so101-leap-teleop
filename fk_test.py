import time
import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path("SO-ARM100/Simulation/SO101/scene.xml")
data = mujoco.MjData(model)

print("Joints:", [model.joint(i).name for i in range(model.njnt)])
print("Bodies:", [model.body(i).name for i in range(model.nbody)])

tip = model.body("gripper").id

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        for _ in range(10):
            mujoco.mj_step(model, data)
        print("Tip xyz:", data.xpos[tip].round(3), end="\r")
        viewer.sync()
        time.sleep(0.02)