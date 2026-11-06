import os
import time
import numpy as np
import mujoco
import mujoco.viewer
from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig

HERE = os.path.dirname(os.path.abspath(__file__))
SCENE = os.path.join(HERE, "..", "SO-ARM100", "Simulation", "SO101", "scene.xml")

# Load the scene and add a draggable target ball
spec = mujoco.MjSpec.from_file(SCENE)
ball = spec.worldbody.add_body(name="target", mocap=True)
ball.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.02, 0, 0],
              rgba=[1, 0, 0, 0.6], contype=0, conaffinity=0)
model = spec.compile()
data = mujoco.MjData(model)

ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
qadr = [model.joint(j).qposadr[0] for j in ARM]
dadr = [model.joint(j).dofadr[0] for j in ARM]
ranges = [model.joint(j).range for j in ARM]
tip = model.body("gripper").id

SIGN   = {j: 1.0 for j in ARM}
OFFSET = {j: 0.0 for j in ARM}
OFFSET["wrist_roll"] = 90.0        # same as mirror.py

def real_to_sim(deg, j): return np.radians(SIGN[j] * deg + OFFSET[j])
def sim_to_real(rad, j): return (np.degrees(rad) - OFFSET[j]) / SIGN[j]

MAX_STEP = 3.0     # max degrees per update sent to the real arm
jacp = np.zeros((3, model.nv))

def ik_step(target, iters=10):
    for _ in range(iters):
        mujoco.mj_forward(model, data)
        err = target - data.xpos[tip]
        if np.linalg.norm(err) < 0.002:
            break
        mujoco.mj_jacBody(model, data, jacp, None, tip)
        J = jacp[:, dadr]
        dq = J.T @ np.linalg.solve(J @ J.T + 0.01 * np.eye(3), err)
        for i, a in enumerate(qadr):
            data.qpos[a] = np.clip(data.qpos[a] + 0.5 * dq[i], *ranges[i])
    mujoco.mj_forward(model, data)
    return np.linalg.norm(target - data.xpos[tip])

config = SOFollowerRobotConfig(port=PORT,
                               id="my_follower_arm", use_degrees=True)
robot = SOFollower(config)
robot.connect()
with robot.bus.torque_disabled():
    for j in ARM + ["gripper"]:
        robot.bus.write("P_Coefficient", j, 32)
    for j in ["shoulder_lift", "elbow_flex"]:
        robot.bus.write("P_Coefficient", j, 48)
        robot.bus.write("Torque_Limit", j, 1000)
for j in ARM:
    print(j, "P =", robot.bus.read("P_Coefficient", j, normalize=False))

# Start the sim at the real arm's pose, ball at the gripper
obs = robot.get_observation()
for j, a in zip(ARM, qadr):
    data.qpos[a] = real_to_sim(obs[f"{j}.pos"], j)
mujoco.mj_forward(model, data)
data.mocap_pos[0] = data.xpos[tip]
sent = {j: obs[f"{j}.pos"] for j in ARM}

STEP_M = 0.01   # 1 cm per key press
nudge = np.zeros(3)

def key_callback(key):
    print("key:", key)
    if key == 265:   nudge[0] += STEP_M   # Up arrow: forward (x+)
    elif key == 264: nudge[0] -= STEP_M   # Down arrow: back (x-)
    elif key == 263: nudge[1] += STEP_M   # Left arrow: left (y+)
    elif key == 262: nudge[1] -= STEP_M   # Right arrow: right (y-)
    elif key == 266: nudge[2] += STEP_M   # fn + Up (Page Up): raise
    elif key == 267: nudge[2] -= STEP_M   # fn + Down (Page Down): lower

try:
    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        while viewer.is_running():
            data.mocap_pos[0] += nudge
            nudge[:] = 0
            err = ik_step(data.mocap_pos[0].copy())
            for j, a in zip(ARM, qadr):
                goal = sim_to_real(data.qpos[a], j)
                sent[j] += np.clip(goal - sent[j], -MAX_STEP, MAX_STEP)
            robot.send_action({f"{j}.pos": v for j, v in sent.items()})
            g = robot.bus.read("Goal_Position", "shoulder_lift", normalize=False)
            p = robot.bus.read("Present_Position", "shoulder_lift", normalize=False)
            print(f"lift goal {g} pos {p} gap {g-p}", end="  ")
            print(f"IK error: {err*1000:5.1f} mm", end="\r")
            viewer.sync()
            time.sleep(0.03)
finally:
    robot.disconnect()