import math
import time
from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig

from config import PORT
config = SOFollowerRobotConfig(
    port=PORT,
    id="my_follower_arm",
    use_degrees=True,
)
robot = SOFollower(config)
robot.connect()

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
for j in JOINTS:
    robot.bus.write("P_Coefficient", j, 32)

obs = robot.get_observation()
home = {j: obs[f"{j}.pos"] for j in JOINTS}

# amplitude (degrees, gripper is 0-100), speed, phase offset
MOTION = {
    "shoulder_pan":  (30, 0.5, 0.0),
    "shoulder_lift": (10, 0.5, 0.5),
    "elbow_flex":    (20, 0.5, 1.0),
    "wrist_flex":    (25, 0.5, 1.5),
    "wrist_roll":    (40, 0.5, 2.0),
    "gripper":       (15, 1.0, 0.0),
}

DURATION = 20
t0 = time.time()
try:
    while (t := time.time() - t0) < DURATION:
        ramp = min(1.0, t / 3, (DURATION - t) / 3)   # ease in and out
        action = {}
        for j, (amp, speed, phase) in MOTION.items():
            offset = ramp * amp * math.sin(2 * math.pi * speed * t - phase)
            action[f"{j}.pos"] = home[j] + offset
        robot.send_action(action)
        time.sleep(0.02)
    robot.send_action({f"{j}.pos": home[j] for j in JOINTS})
    time.sleep(1)
finally:
    robot.disconnect()