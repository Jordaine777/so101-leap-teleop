import time
import numpy as np
import mujoco
import mujoco.viewer
from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig

import os
HERE = os.path.dirname(os.path.abspath(__file__))
model = mujoco.MjModel.from_xml_path(
    os.path.join(HERE, "..", "SO-ARM100", "Simulation", "SO101", "scene.xml"))

data = mujoco.MjData(model)

ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
qadr = {j: model.joint(j).qposadr[0] for j in ARM}

# Fix these after watching: flip a sign if a joint turns the wrong way,
# add an offset (degrees) if it's consistently shifted
SIGN   = {j: 1.0 for j in ARM}
OFFSET = {j: 0.0 for j in ARM}
OFFSET["wrist_roll"] = 0.0

config = SOFollowerRobotConfig(port=PORT,
                               id="my_follower_arm", use_degrees=True)
robot = SOFollower(config)
robot.connect()
robot.bus.disable_torque()   # so you can move it by hand

try:
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            obs = robot.get_observation()
            line = []
            for j in ARM:
                deg = SIGN[j] * obs[f"{j}.pos"] + OFFSET[j]
                data.qpos[qadr[j]] = np.radians(deg)
                line.append(f"{j}:{obs[f'{j}.pos']:6.1f}")
            mujoco.mj_forward(model, data)
            print("  ".join(line), end="\r")
            viewer.sync()
            time.sleep(0.03)
finally:
    robot.disconnect()