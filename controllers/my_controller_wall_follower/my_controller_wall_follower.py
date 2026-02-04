from controller import Robot

# --------- Tunables (change only if needed) ----------
MAX_SPEED = 6.28

# Sensor thresholds (start safe; adjust if needed)
TH_FRONT = 120.0      # wall in front
TH_LEFT = 80.0        # wall on left exists
TH_TOO_CLOSE = 220.0  # too close to left wall (corner)

# Wall distance control (simple proportional)
TARGET_LEFT = 140.0   # desired left reading (depends on your maze scale)
KP = 0.004            # correction gain (small!)

# Stuck recovery
STUCK_STEPS = 150     # ~ few seconds depending on timestep
# ----------------------------------------------------


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    # Motors
    left_motor = robot.getDevice("left wheel motor")
    right_motor = robot.getDevice("right wheel motor")
    left_motor.setPosition(float("inf"))
    right_motor.setPosition(float("inf"))

    # Proximity sensors ps0..ps7
    ps = []
    for i in range(8):
        s = robot.getDevice(f"ps{i}")
        s.enable(timestep)
        ps.append(s)

    # Helpers for loop/stuck detection
    stuck_counter = 0
    last_state = None
    same_state_count = 0

    while robot.step(timestep) != -1:
        v = [s.getValue() for s in ps]

        # Robust front detection: use both front corners
        front_wall = (v[0] > TH_FRONT) or (v[7] > TH_FRONT)

        # Robust left detection: left + left-front
        left_wall = (v[5] > TH_LEFT) or (v[6] > TH_LEFT)

        # Too close on left-front usually means hugging wall after a turn
        too_close_left = v[6] > TH_TOO_CLOSE

        # Decide state (left-hand rule + safety)
        if front_wall:
            state = "TURN_RIGHT"
        elif not left_wall:
            state = "TAKE_LEFT"
        elif too_close_left:
            state = "MOVE_AWAY"
        else:
            state = "FOLLOW"

        # Simple loop/stuck detection (same state too long)
        if state == last_state:
            same_state_count += 1
        else:
            same_state_count = 0
            last_state = state

        # If we keep doing same turning state too long, do a recovery
        if same_state_count > STUCK_STEPS and state in ("TURN_RIGHT", "TAKE_LEFT"):
            # Recovery: go forward a bit then turn right
            left_speed = 0.6 * MAX_SPEED
            right_speed = 0.6 * MAX_SPEED
            left_motor.setVelocity(left_speed)
            right_motor.setVelocity(right_speed)

            # advance for a short time
            for _ in range(15):
                if robot.step(timestep) == -1:
                    return

            left_speed = 0.5 * MAX_SPEED
            right_speed = -0.5 * MAX_SPEED
            left_motor.setVelocity(left_speed)
            right_motor.setVelocity(right_speed)

            for _ in range(10):
                if robot.step(timestep) == -1:
                    return

            same_state_count = 0
            continue

        # Execute state actions
        if state == "TURN_RIGHT":
            # Turn right in place
            left_speed = 0.45 * MAX_SPEED
            right_speed = -0.45 * MAX_SPEED

        elif state == "TAKE_LEFT":
            # Turn left to find/attach to new left wall
            left_speed = 0.15 * MAX_SPEED
            right_speed = 0.55 * MAX_SPEED

        elif state == "MOVE_AWAY":
            # Too close to left wall -> steer right a bit
            left_speed = 0.55 * MAX_SPEED
            right_speed = 0.25 * MAX_SPEED

        else:  # FOLLOW
            # Go forward while maintaining left distance (P control)
            # Use combined left measure for stability
            left_measure = max(v[5], v[6])
            error = TARGET_LEFT - left_measure  # positive => too far => steer left a bit
            correction = clamp(KP * error, -0.2, 0.2)

            base = 0.65 * MAX_SPEED
            left_speed = base - correction * MAX_SPEED
            right_speed = base + correction * MAX_SPEED

        left_motor.setVelocity(clamp(left_speed, -MAX_SPEED, MAX_SPEED))
        right_motor.setVelocity(clamp(right_speed, -MAX_SPEED, MAX_SPEED))


if __name__ == "__main__":
    main()
