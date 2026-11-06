import os
import time
import numpy as np
import mujoco
import mujoco.viewer
from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig

MODE = "fk"    # "fk" or "nudge"

HERE = os.path.dirname(os.path.abspath(__file__))
SCENE = os.path.join(HERE, "..", "SO-ARM100", "Simulation", "SO101", "scene.xml")
spec = mujoco.MjSpec.from_file(SCENE)
spec.joint("wrist_roll").range = np.radians([-164, 164])
model = spec.compile()
data = mujoco.MjData(model)

ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
qadr = {j: model.joint(j).qposadr[0] for j in ARM}
tip = model.body("gripper").id

TIP_OFFSET = np.array([0.0, 0.0, -0.10])   # meters, in the gripper's own frame

def tip_point():
    R = data.xmat[tip].reshape(3, 3)
    return data.xpos[tip] + R @ TIP_OFFSET

COLORS = {
    "shoulder_pan":  [1.0, 0.2, 0.2, 1],   # red
    "shoulder_lift": [1.0, 0.6, 0.1, 1],   # orange
    "elbow_flex":    [0.2, 0.8, 0.2, 1],   # green
    "wrist_flex":    [0.2, 0.5, 1.0, 1],   # blue
    "wrist_roll":    [0.8, 0.3, 1.0, 1],   # purple
}

from config import PORT
config = SOFollowerRobotConfig(port=PORT,
                               id="my_follower_arm", use_degrees=True)
robot = SOFollower(config)
robot.connect()
with robot.bus.torque_disabled():
    for j in ARM + ["gripper"]:
        robot.bus.write("P_Coefficient", j, 32)
    for j in ["shoulder_lift", "elbow_flex"]:
        robot.bus.write("P_Coefficient", j, 48)
        robot.bus.write("Max_Torque_Limit", j, 1000)
        robot.bus.write("Torque_Limit", j, 1000)
    robot.bus.write("P_Coefficient", "wrist_roll", 48)

trail = []

def sync(viewer, color=None):
    obs = robot.get_observation()
    for j in ARM:
        data.qpos[qadr[j]] = np.radians(obs[f"{j}.pos"])
    mujoco.mj_forward(model, data)
    p_tip = tip_point()
    scn = viewer.user_scn
    if color is not None:
        trail.append((p_tip.copy(), color))
    pts = trail[-(scn.maxgeom - 1):]
    for i, (p, c) in enumerate(pts):
        mujoco.mjv_initGeom(scn.geoms[i], mujoco.mjtGeom.mjGEOM_SPHERE,
                            np.array([0.004, 0, 0]), p, np.eye(3).flatten(),
                            np.array(c, dtype=np.float32))
    mujoco.mjv_initGeom(scn.geoms[len(pts)], mujoco.mjtGeom.mjGEOM_SPHERE,
                        np.array([0.008, 0, 0]), p_tip, np.eye(3).flatten(),
                        np.array([1, 1, 0, 1], dtype=np.float32))
    scn.ngeom = len(pts) + 1
    viewer.sync()
    return obs

def glide(viewer, goal, seconds=2.0, color=None):
    start = robot.get_observation()
    steps = int(seconds / 0.02)
    for k in range(1, steps + 1):
        s = k / steps
        s = s * s * (3 - 2 * s)
        action = {}
        for j in ARM:
            a = start[f"{j}.pos"]
            b = goal.get(j, a)
            action[f"{j}.pos"] = a + s * (b - a)
        robot.send_action(action)
        sync(viewer, color)
        time.sleep(0.02)

def tip_cm():
    return np.round(data.xpos[tip] * 100, 1)

# FK poses, degrees from your calibrated middle pose. Keep them modest.
POSES = [
    {"shoulder_pan": 0,   "shoulder_lift": 0,   "elbow_flex": 0,   "wrist_flex": 0},
    {"shoulder_pan": 30,  "shoulder_lift": -15, "elbow_flex": 20,  "wrist_flex": 10},
    {"shoulder_pan": -30, "shoulder_lift": 10,  "elbow_flex": -20, "wrist_flex": -15},
    {"shoulder_pan": 0,   "shoulder_lift": -25, "elbow_flex": 35,  "wrist_flex": 20},
]

# Exaggerated nudge sizes per joint (degrees)
AMP = {"shoulder_pan": 20, "shoulder_lift": 12, "elbow_flex": 15,
       "wrist_flex": 20, "wrist_roll": 40}

try:
    with mujoco.viewer.launch_passive(model, data) as viewer:
        obs = sync(viewer)
        home = {j: obs[f"{j}.pos"] for j in ARM}
        time.sleep(2)   # time to start recording

        if MODE == "fk":
            for pose in POSES:
                print("\nAngles in:", pose)
                glide(viewer, pose, seconds=2.5, color=[1, 1, 1, 1])
                time.sleep(1.0)
                print("Hand ends up at (cm):", tip_cm())
                time.sleep(1.5)

        elif MODE == "nudge":
            for j in ARM:
                print(f"\nNudging {j}")
                for _ in range(2):
                    glide(viewer, {j: home[j] + AMP[j]}, 0.8, COLORS[j])
                    glide(viewer, {j: home[j] - AMP[j]}, 1.2, COLORS[j])
                glide(viewer, home, 0.8, COLORS[j])
                time.sleep(1.0)

        glide(viewer, home, 2.0)
        print("\nDone. Close the viewer when you're finished recording.")
        while viewer.is_running():
            sync(viewer)
            time.sleep(0.03)
finally:
    robot.disconnect()