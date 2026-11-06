import os
import time
import threading
import numpy as np
import mujoco
import mujoco.viewer
import leap
from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig

HERE = os.path.dirname(os.path.abspath(__file__))
SCENE = os.path.join(HERE, "..", "SO-ARM100", "Simulation", "SO101", "scene.xml")

spec = mujoco.MjSpec.from_file(SCENE)
ball = spec.worldbody.add_body(name="target", mocap=True)
ball.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.02, 0, 0],
              rgba=[1, 0, 0, 0.6], contype=0, conaffinity=0)

#OFFSET["wrist_roll"] = 0.0
ROLL_LIM = (-164, 164)
spec.joint("wrist_roll").range = np.radians([-164, 164])

model = spec.compile()
data = mujoco.MjData(model)


#ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
#qadr = [model.joint(j).qposadr[0] for j in ARM]
#dadr = [model.joint(j).dofadr[0] for j in ARM]
#ranges = [model.joint(j).range for j in ARM]
#tip = model.body("gripper").id

ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
IK_JOINTS = ARM[:3]
qadr = [model.joint(j).qposadr[0] for j in ARM]
ik_qadr = qadr[:3]
dadr = [model.joint(j).dofadr[0] for j in IK_JOINTS]
ranges = [model.joint(j).range for j in IK_JOINTS]
tip = model.body("wrist").id
grip_adr = model.joint("gripper").qposadr[0]
grip_lo, grip_hi = model.joint("gripper").range

SIGN   = {j: 1.0 for j in ARM}
OFFSET = {j: 0.0 for j in ARM}
OFFSET["wrist_roll"] = 0.0

def real_to_sim(deg, j): return np.radians(SIGN[j] * deg + OFFSET[j])
def sim_to_real(rad, j): return (np.degrees(rad) - OFFSET[j]) / SIGN[j]

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
        #for i, a in enumerate(qadr):
        #    data.qpos[a] = np.clip(data.qpos[a] + 0.5 * dq[i], *ranges[i])

        for i, a in enumerate(ik_qadr):
            data.qpos[a] = np.clip(data.qpos[a] + 0.5 * dq[i], *ranges[i])
    mujoco.mj_forward(model, data)
    return np.linalg.norm(target - data.xpos[tip])

# ---------- Leap hand reading (runs in background) ----------

hand_state = {"palm": None, "pinch": 0.0, "pitch": 0.0, "roll": 0.0, "grab": 0.0}
lock = threading.Lock()

# class HandListener(leap.Listener):
#     def on_tracking_event(self, event):
#         with lock:
#             if event.hands:
#                 h = event.hands[0]
#                 p = h.palm.position
#                 hand_state["palm"] = np.array([p.x, p.y, p.z]) / 1000.0  # mm -> m
#                 hand_state["pinch"] = h.pinch_strength
#             else:
#                 hand_state["palm"] = None

class HandListener(leap.Listener):
    def on_tracking_event(self, event):
        with lock:
            if event.hands:
                h = event.hands[0]
                p = h.palm.position
                d = h.palm.direction      # points toward your fingers
                n = h.palm.normal         # points out of your palm
                hand_state["palm"] = np.array([p.x, p.y, p.z]) / 1000.0
                hand_state["pinch"] = h.pinch_strength
                hand_state["pitch"] = np.degrees(np.arctan2(d.y, -d.z))
                hand_state["roll"] = np.degrees(np.arctan2(n.x, -n.y))
                hand_state["grab"] = h.grab_strength
            else:
                hand_state["palm"] = None

def leap_to_robot(v):
    # Leap: x right, y up, z toward you  ->  Robot: x forward, y left, z up
    return np.array([-v[2], -v[0], v[1]])

SCALE = 0.5          # 10 cm of hand motion = 5 cm of arm motion
SMOOTH = 0.15        # 0..1, lower = smoother but laggier
MAX_STEP = 3.0       # max degrees per update to the real arm
GRIP_OPEN, GRIP_CLOSED = 60.0, 0.0   # gripper range, adjust to your arm
WRIST_GAIN = -1.0     # 1.0 = wrist copies your hand angle exactly
ROLL_GAIN = 1.0

# ---------- Robot ----------
config = SOFollowerRobotConfig(port=PORT,
                               id="my_follower_arm", use_degrees=True)
robot = SOFollower(config)
robot.connect()

cal = robot.bus.calibration
def deg_limits(j):
    c = cal[j]
    half = (c.range_max - c.range_min) / 2 * 360 / 4096
    return -half + 2, half - 2     # 2-degree safety margin
#ROLL_LIM = deg_limits("wrist_roll")
FLEX_LIM = deg_limits("wrist_flex")
print("roll limits", ROLL_LIM, "flex limits", FLEX_LIM)

lo, hi = model.joint("wrist_roll").range
print("sim roll range in real degrees:",
      round(sim_to_real(lo, "wrist_roll")), round(sim_to_real(hi, "wrist_roll")))


with robot.bus.torque_disabled():
    for j in ARM + ["gripper"]:
        robot.bus.write("P_Coefficient", j, 32)
    for j in ["shoulder_lift", "elbow_flex"]:
        robot.bus.write("P_Coefficient", j, 48)
        robot.bus.write("Max_Torque_Limit", j, 1000)
        robot.bus.write("Torque_Limit", j, 1000)
    
    robot.bus.write("P_Coefficient", "wrist_roll", 48)
    robot.bus.write("Torque_Limit", "gripper", 700)   # gentle grip

obs = robot.get_observation()
for j, a in zip(ARM, qadr):
    data.qpos[a] = real_to_sim(obs[f"{j}.pos"], j)
mujoco.mj_forward(model, data)
data.mocap_pos[0] = data.xpos[tip]
center = data.xpos[model.body("upper_arm").id].copy()   # shoulder pivot
L1 = np.linalg.norm(data.xpos[model.body("lower_arm").id] - center)
L2 = np.linalg.norm(data.xpos[model.body("wrist").id] - data.xpos[model.body("lower_arm").id])
R_MAX = 0.95 * (L1 + L2)   # just short of full stretch
R_MIN = 0.35 * (L1 + L2)   # don't fold into itself
Z_MIN = 0.05               # stay above the table

def clamp_reach(p):
    v = p - center
    r = np.linalg.norm(v)
    if r > R_MAX:
        v *= R_MAX / r
    elif 1e-6 < r < R_MIN:
        v *= R_MIN / r
    q = center + v
    q[2] = max(q[2], Z_MIN)
    return q

sent = {j: obs[f"{j}.pos"] for j in ARM}
grip = obs["gripper.pos"]

anchor_hand = None      # hand position when tracking started
anchor_ball = None      # ball position at that moment

listener = HandListener()
connection = leap.Connection()
connection.add_listener(listener)

paused = False
fist_down = False
grip_locked = False

try:
    with connection.open():
        connection.set_tracking_mode(leap.TrackingMode.Desktop)
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running():
                with lock:
                    palm = hand_state["palm"]
                    pinch = hand_state["pinch"]
                    pinch = min(1.0, pinch / 0.8)
                    pitch = hand_state["pitch"]
                    roll = hand_state["roll"]
                    grab = hand_state["grab"]

                if palm is not None:
                    if not fist_down and grab > 0.9:
                        fist_down = True
                        paused = not paused
                        anchor_hand = None      # re-anchor when you resume
                    elif fist_down and grab < 0.5:
                        fist_down = False

                if palm is None or paused or grab > 0.6:
                    anchor_hand = None
                    grip_locked = True
                    status = "PAUSED (fist to resume)" if paused else "holding"

                else:
                    if anchor_hand is None:
                        anchor_hand = palm.copy()
                        anchor_ball = data.mocap_pos[0].copy()
                        anchor_pitch, anchor_roll = pitch, roll
                        now = robot.get_observation()
                        anchor_flex = now["wrist_flex.pos"]
                        anchor_wroll = now["wrist_roll.pos"]
                    
                    raw = anchor_ball + SCALE * leap_to_robot(palm - anchor_hand)
                    goal = clamp_reach(raw)
                    anchor_ball += goal - raw   # slide anchor so coming back is instant
                    data.mocap_pos[0] += SMOOTH * (goal - data.mocap_pos[0])

                    flex_goal = anchor_flex + WRIST_GAIN * (pitch - anchor_pitch)
                    roll_goal = anchor_wroll + ROLL_GAIN * (roll - anchor_roll)
                    roll_goal = float(np.clip(roll_goal, *ROLL_LIM))
                    flex_goal = float(np.clip(flex_goal, *FLEX_LIM))
                    data.qpos[qadr[3]] += SMOOTH * (real_to_sim(flex_goal, "wrist_flex") - data.qpos[qadr[3]])
                    data.qpos[qadr[4]] += SMOOTH * (real_to_sim(roll_goal, "wrist_roll") - data.qpos[qadr[4]])

                    target_grip = GRIP_OPEN + pinch * (GRIP_CLOSED - GRIP_OPEN)
                    if grip_locked and target_grip <= grip + 2:
                        grip_locked = False          # your pinch caught up, take over
                    if not grip_locked:
                        grip += SMOOTH * (target_grip - grip)

                    status = f"pinch {pinch:.2f}" + (" | GRIP LOCKED, pinch to take over" if grip_locked else "")
                    status = f"pinch {pinch:.2f} pitch {pitch:5.0f} roll {roll:5.0f}"

                data.qpos[grip_adr] = grip_lo + (grip / 100.0) * (grip_hi - grip_lo)
                err = ik_step(data.mocap_pos[0].copy())
                for j, a in zip(ARM, qadr):
                    g = sim_to_real(data.qpos[a], j)
                    sent[j] += np.clip(g - sent[j], -MAX_STEP, MAX_STEP)
                action = {f"{j}.pos": v for j, v in sent.items()}
                action["gripper.pos"] = grip
                robot.send_action(action)

                obs = robot.get_observation()
                print(f"{status} | roll cmd {sent['wrist_roll']:6.1f} | roll actual {obs['wrist_roll.pos']:6.1f}      ", end="\r")
                viewer.sync()
                time.sleep(0.02)
finally:
    robot.disconnect()