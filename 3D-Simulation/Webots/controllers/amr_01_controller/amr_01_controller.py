from controller import Robot
import math

robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())

left_motor = robot.getDevice("left_wheel_motor")
right_motor = robot.getDevice("right_wheel_motor")
gps = robot.getDevice("gps")
imu = robot.getDevice("imu")

left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))

left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

gps.enable(TIME_STEP)
imu.enable(TIME_STEP)

# ---------------------------------------------------------
# NEXFLEET DEMO ROUTE
# ---------------------------------------------------------
#
# AMR01 start:
#     (15.8, -10)
#
# PRODUCT_01:
#     (-12, -11)
#
# The final waypoint stops short of the pallet.
# ---------------------------------------------------------

WAYPOINTS = [
    (12.0, -11.0),     # leave charging area
    (7.0,  -11.0),     # lower corridor
    (2.0,  -11.0),
    (-3.0, -11.0),
    (-7.0, -11.0),
    (-10.6, -11.0),    # pickup approach
]

waypoint_index = 0

MAX_SPEED = 8.0
TURN_SPEED = 3.0

DIST_TOLERANCE = 0.35
ANGLE_TOLERANCE = 0.15


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


def stop():
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)


print("=" * 60)
print("NEXFLEET | AMR01 EDGE CONTROLLER")
print("MODE: AUTOMATIC PRODUCT PICKUP APPROACH")
print("MISSION: PRODUCT_01")
print("START: CHARGING AREA")
print("DESTINATION: PICKUP ZONE")
print("=" * 60)


while robot.step(TIME_STEP) != -1:

    if waypoint_index >= len(WAYPOINTS):
        stop()

        print("=" * 60)
        print("NEXFLEET | AMR01")
        print("PRODUCT_01 PICKUP APPROACH REACHED")
        print("AMR01 READY FOR PICKUP")
        print("=" * 60)

        # Prevent repeated printing.
        while robot.step(TIME_STEP) != -1:
            stop()

        break

    gps_values = gps.getValues()

    x = gps_values[0]
    y = gps_values[1]

    roll, pitch, yaw = imu.getRollPitchYaw()

    target_x, target_y = WAYPOINTS[waypoint_index]

    dx = target_x - x
    dy = target_y - y

    distance = math.hypot(dx, dy)

    desired_yaw = math.atan2(dy, dx)

    angle_error = normalize_angle(
        desired_yaw - yaw
    )

    # -----------------------------------------------------
    # Waypoint reached
    # -----------------------------------------------------

    if distance < DIST_TOLERANCE:

        stop()

        print(
            f"NEXFLEET | AMR01 | "
            f"Waypoint {waypoint_index + 1}/{len(WAYPOINTS)} reached "
            f"({x:.2f}, {y:.2f})"
        )

        waypoint_index += 1
        continue

    # -----------------------------------------------------
    # Rotate toward waypoint
    # -----------------------------------------------------

    if abs(angle_error) > ANGLE_TOLERANCE:

        if angle_error > 0:
            left_motor.setVelocity(-TURN_SPEED)
            right_motor.setVelocity(TURN_SPEED)

        else:
            left_motor.setVelocity(TURN_SPEED)
            right_motor.setVelocity(-TURN_SPEED)

    # -----------------------------------------------------
    # Drive toward waypoint
    # -----------------------------------------------------

    else:

        correction = max(
            -1.0,
            min(1.0, angle_error * 2.0)
        )

        left_speed = MAX_SPEED - correction
        right_speed = MAX_SPEED + correction

        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
