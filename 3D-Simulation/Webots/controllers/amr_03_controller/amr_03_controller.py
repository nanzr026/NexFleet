from controller import Robot

ROBOT_ID = "AMR03"

robot = Robot()
timestep = int(robot.getBasicTimeStep())

print("=" * 55)
print(f"NEXFLEET | {ROBOT_ID} EDGE CONTROLLER")
print("Independent decentralized controller started")
print("=" * 55)

while robot.step(timestep) != -1:
    pass
