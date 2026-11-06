from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig

from config import PORT
config = SOFollowerRobotConfig(port=PORT, id="my_follower_arm")
robot = SOFollower(config)
robot.connect()

for reg in ["Present_Voltage", "Present_Load", "Present_Temperature",
            "P_Coefficient", "Max_Torque_Limit", "Torque_Limit"]:
    try:
        print(reg, robot.bus.read(reg, "shoulder_pan", normalize=False))
    except Exception as e:
        print(reg, "error:", e)

robot.disconnect()