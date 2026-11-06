import curses
from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig

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

STEP = 3      # degrees per key press
LEAD = 15     # max degrees the target can run ahead of the arm

def main(scr):
    scr.nodelay(True)
    obs = robot.get_observation()
    target = {"shoulder_pan": obs["shoulder_pan.pos"], "wrist_flex": obs["wrist_flex.pos"]}
    while True:
        key = scr.getch()
        if key == curses.KEY_LEFT:
            target["shoulder_pan"] += STEP
        elif key == curses.KEY_RIGHT:
            target["shoulder_pan"] -= STEP
        elif key == curses.KEY_UP:
            target["wrist_flex"] += STEP
        elif key == curses.KEY_DOWN:
            target["wrist_flex"] -= STEP
        elif key == ord("q"):
            break

        obs = robot.get_observation()
        for j in target:
            actual = obs[f"{j}.pos"]
            target[j] = max(actual - LEAD, min(actual + LEAD, target[j]))

        robot.send_action({f"{j}.pos": t for j, t in target.items()})

        scr.erase()
        scr.addstr(0, 0, "Arrows: Left/Right = base, Up/Down = wrist, q = quit")
        row = 2
        for j, t in target.items():
            scr.addstr(row, 0, f"{j:14s} target {t:7.1f}   actual {obs[f'{j}.pos']:7.1f}")
            row += 1
        curses.napms(30)

try:
    curses.wrapper(main)
finally:
    robot.disconnect()