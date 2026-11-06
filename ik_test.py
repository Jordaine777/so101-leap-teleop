import time
import numpy as np
import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path("SO-ARM100/Simulation/SO101/scene.xml")
data = mujoco.MjData(model)

arm = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
qadr = [model.joint(n).qposadr[0] for n in arm]
dadr = [model.joint(n).dofadr[0] for n in arm]
ranges = [model.joint(n).range for n in arm]
tip = model.body("gripper").id

target = np.array([0.20, 0.0, 0.15])  # meters, x y z in the world frame
jacp = np.zeros((3, model.nv))
step, damping = 0.5, 0.01

def draw_target(viewer):
    viewer.user_scn.ngeom = 0
    mujoco.mjv_initGeom(
        viewer.user_scn.geoms[0], mujoco.mjtGeom.mjGEOM_SPHERE,
        np.array([0.015, 0, 0]), target, np.eye(3).flatten(),
        np.array([1, 0, 0, 1], dtype=np.float32))
    viewer.user_scn.ngeom = 1

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_forward(model, data)
        err = target - data.xpos[tip]
        mujoco.mj_jacBody(model, data, jacp, None, tip)
        J = jacp[:, dadr]
        dq = J.T @ np.linalg.solve(J @ J.T + damping * np.eye(3), err)
        for i, a in enumerate(qadr):
            lo, hi = ranges[i]
            data.qpos[a] = np.clip(data.qpos[a] + step * dq[i], lo, hi)
        print(f"Error: {np.linalg.norm(err) * 1000:.1f} mm   ", end="\r")
        draw_target(viewer)
        viewer.sync()
        time.sleep(0.02)