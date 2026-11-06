import time
from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig

from config import PORT
config = SOFollowerRobotConfig(port=PORT, id="my_follower_arm", use_degrees=True)
robot = SOFollower(config)
robot.connect()

for j in ["shoulder_pan", "wrist_flex"]:
    robot.bus.write("P_Coefficient", j, 32)
    robot.bus.write("Torque_Limit", j, 1000)

for joint in ["shoulder_pan", "wrist_flex"]:
    start = robot.get_observation()[f"{joint}.pos"]
    
    print("model", robot.bus.read("Model_Number", joint, normalize=False))
    print("limits", robot.bus.read("Min_Position_Limit", joint, normalize=False),
      robot.bus.read("Max_Position_Limit", joint, normalize=False))
    
    robot.send_action({f"{joint}.pos": start + 90})
    time.sleep(1.5)
    now = robot.get_observation()[f"{joint}.pos"]
    goal = robot.bus.read("Goal_Position", joint, normalize=False)
    pos = robot.bus.read("Present_Position", joint, normalize=False)
    print(f"{joint}: start {start:.1f}, now {now:.1f}, raw goal {goal}, raw pos {pos}")
    robot.send_action({f"{joint}.pos": start})
    time.sleep(1.5)

robot.disconnect()