# GO FROM START TO END POINT AVOIDING INTERSECTIONS/EDGES WITH DYNAMIC OBSTACLE DETECTION

import time
import RPi.GPIO as GPIO
import sys
import Adafruit_PCA9685
import DC_motors as motor
import RPIservo_vS as servo
from PID_inter import LineFollowingRobotPID
from collections import deque
from task8 import UltrasonicSensor, OLED_Display

DIR_FORWARD = 0
DIR_BACKWARD = 1
DANGER_DISTANCE = 70.0  # in cm

#LED SETUP
def switchSetup():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(5, GPIO.OUT)   # Blue LED
    GPIO.setup(6, GPIO.OUT)   # Right LED
    GPIO.setup(13, GPIO.OUT)  # Left LED

def set_all_leds_off():
    GPIO.output(5, GPIO.LOW)
    GPIO.output(6, GPIO.LOW)
    GPIO.output(13, GPIO.LOW)

def set_led(port, status):
    """port: 0=right(6), 1=left(13), 2=blue(5)"""
    if port == 0:
        GPIO.output(6, GPIO.HIGH if status else GPIO.LOW)
    elif port == 1:
        GPIO.output(13, GPIO.HIGH if status else GPIO.LOW)
    elif port == 2:
        GPIO.output(5, GPIO.HIGH if status else GPIO.LOW)

def indicate_obstacles(dist_front, dist_left, dist_right, danger_distance):
    """Light up LEDs based on obstacle detection"""
    front_blocked = (dist_front is not None) and (dist_front < danger_distance)
    set_led(2, front_blocked)

    left_blocked = (dist_left is not None) and (dist_left < danger_distance)
    set_led(1, left_blocked)

    right_blocked = (dist_right is not None) and (dist_right < danger_distance)
    set_led(0, right_blocked)


# NEIGHBOR / GRAPH LOGIC
def neighbors(pos):
    x, y = pos
    return {
        "N": (x, y + 1),
        "S": (x, y - 1),
        "E": (x + 1, y),
        "W": (x - 1, y),
    }

def in_bounds(pos, min_x=0, min_y=0, max_x=4, max_y=4):
    x, y = pos
    return min_x <= x <= max_x and min_y <= y <= max_y

def get_direction_from_positions(current_pos, next_pos):
    """Returns the direction ('N', 'S', 'E', 'W') from current to next position"""
    dx = next_pos[0] - current_pos[0]
    dy = next_pos[1] - current_pos[1]

    if dx == 1:
        return "E"
    elif dx == -1:
        return "W"
    elif dy == 1:
        return "N"
    elif dy == -1:
        return "S"
    return None


# BFS PATHFINDING WITH ORIENTATION
def bfs_path(start, start_dir, end, blocked_nodes=None, blocked_edges=None):
    if blocked_nodes is None:
        blocked_nodes = set()
    if blocked_edges is None:
        blocked_edges = set()

    if start in blocked_nodes or end in blocked_nodes:
        return None

    directions = ["N", "E", "S", "W"]

    def turn_left(d):
        return directions[(directions.index(d) - 1) % 4]

    def turn_right(d):
        return directions[(directions.index(d) + 1) % 4]

    queue = deque([(start, start_dir, [])])
    visited = {(start, start_dir)}

    while queue:
        current_pos, current_dir, actions = queue.popleft()

        if current_pos == end:
            return actions

        def can_move(from_pos, to_pos):
            if not in_bounds(to_pos):
                return False
            if to_pos in blocked_nodes:
                return False
            if ((from_pos, to_pos) in blocked_edges or
                (to_pos, from_pos) in blocked_edges):
                return False
            return True

        # Forward
        next_pos_forward = neighbors(current_pos).get(current_dir)
        if next_pos_forward and can_move(current_pos, next_pos_forward):
            state = (next_pos_forward, current_dir)
            if state not in visited:
                visited.add(state)
                queue.append((next_pos_forward, current_dir, actions + ["forward"]))

        # Left
        new_dir_left = turn_left(current_dir)
        next_pos_left = neighbors(current_pos).get(new_dir_left)
        if next_pos_left and can_move(current_pos, next_pos_left):
            state = (next_pos_left, new_dir_left)
            if state not in visited:
                visited.add(state)
                queue.append((next_pos_left, new_dir_left, actions + ["left"]))

        # Right
        new_dir_right = turn_right(current_dir)
        next_pos_right = neighbors(current_pos).get(new_dir_right)
        if next_pos_right and can_move(current_pos, next_pos_right):
            state = (next_pos_right, new_dir_right)
            if state not in visited:
                visited.add(state)
                queue.append((next_pos_right, new_dir_right, actions + ["right"]))

    return None

# COMPUTE MOVES
def compute_moves(start, start_dir, end, blocked_nodes=None, blocked_edges=None):
    """Returns actions list with 'stop' at the end"""
    actions = bfs_path(start, start_dir, end, blocked_nodes, blocked_edges)
    if actions is None:
        print("No valid path found.")
        return None

    actions.append("stop")
    return actions


# OBSTACLE DETECTION

def scan_for_obstacles(robot, ultrasonic, oled):

    print("\n--- OBSTACLE SCANNING ---")

    print("Measuring front distance...")
    dist_front = ultrasonic.measure()
    print(f"Front: {dist_front} cm")

    oled.set_all_lines([
        "SCANNING",
        f"Front: {dist_front if dist_front else '---'} cm",
        "Checking sides...",
        "",
        "",
        ""
    ])
    time.sleep(0.3)

    print("Looking left...")
    robot.servo.moveAngle(1, 120)
    time.sleep(0.8)
    dist_left = ultrasonic.measure()
    print(f"Left: {dist_left} cm")

    oled.set_all_lines([
        "SCANNING LEFT",
        f"Dist: {dist_left if dist_left else '---'} cm",
        "",
        "",
        "",
        ""
    ])
    time.sleep(0.3)

    print("Looking right...")
    robot.servo.moveAngle(1, -120)
    time.sleep(0.8)
    dist_right = ultrasonic.measure()
    print(f"Right: {dist_right} cm")

    oled.set_all_lines([
        "SCANNING RIGHT",
        f"Dist: {dist_right if dist_right else '---'} cm",
        "",
        "",
        "",
        ""
    ])
    time.sleep(0.3)

    # Return head to center
    print("Returning head to center...")
    robot.servo.moveAngle(1, 0)
    time.sleep(0.5)

    print(f"Scan complete: F={dist_front}, L={dist_left}, R={dist_right}")
    return dist_front, dist_left, dist_right

def get_blocked_edges_from_scan(current_pos, current_dir, dist_front, dist_left, dist_right):
    blocked_edges = set()
    directions = ["N", "E", "S", "W"]

    def turn_left(d):
        return directions[(directions.index(d) - 1) % 4]

    def turn_right(d):
        return directions[(directions.index(d) + 1) % 4]

    # Check obstacle in front
    if dist_front is not None and dist_front < DANGER_DISTANCE:
        next_pos = neighbors(current_pos).get(current_dir)
        if next_pos:
            blocked_edges.add((current_pos, next_pos))
            print(f"Blocking FRONT edge: {current_pos} -> {next_pos}")

    # Check obstacle to the left
    if dist_left is not None and dist_left < DANGER_DISTANCE:
        left_dir = turn_left(current_dir)
        next_pos = neighbors(current_pos).get(left_dir)
        if next_pos:
            blocked_edges.add((current_pos, next_pos))
            print(f"Blocking LEFT edge: {current_pos} -> {next_pos}")

    # Check obstacle to the right
    if dist_right is not None and dist_right < DANGER_DISTANCE:
        right_dir = turn_right(current_dir)
        next_pos = neighbors(current_pos).get(right_dir)
        if next_pos:
            blocked_edges.add((current_pos, next_pos))
            print(f"Blocking RIGHT edge: {current_pos} -> {next_pos}")

    return blocked_edges

# MOVEMENT FUNCTIONS
def detect_intersection(robot):
    left, middle, right = robot.read_sensors()
    return left == 0 and middle == 0 and right == 0

def detect_line(robot):
    left, middle, right = robot.read_sensors()
    return left == 0 or middle == 0 or right == 0

def turn_right(robot, sc, speed=40):
    print("Executing right turn...")

    # Recenter on intersection
    print("Recentering on intersection...")
    time.sleep(0.5)
    sc.moveAngle(0, 0)
    motor.motor_left(1, DIR_BACKWARD, 22)
    motor.motor_right(1, DIR_BACKWARD, 22)
    while not detect_intersection(robot):
        time.sleep(0.05)
    motor.motorStop()
    time.sleep(0.5)

    # Maneuver 1
    sc.moveAngle(0, -120)
    time.sleep(0.3)
    motor.motor_left(1, DIR_FORWARD, speed)
    motor.motor_right(1, DIR_FORWARD, speed)
    time.sleep(0.5)
    motor.motorStop()
    time.sleep(0.3)

    # Maneuver 2
    sc.moveAngle(0, 120)
    time.sleep(0.3)
    motor.motor_left(1, DIR_BACKWARD, speed)
    motor.motor_right(1, DIR_BACKWARD, speed)
    time.sleep(0.5)
    motor.motorStop()
    time.sleep(0.3)

    # Maneuver 3
    sc.moveAngle(0, -90)
    time.sleep(0.3)
    motor.motor_left(1, DIR_FORWARD, speed)
    motor.motor_right(1, DIR_FORWARD, speed)
    time.sleep(0.6)

    # Final maneuver with line detection
    sc.moveAngle(0, -70)
    time.sleep(0.2)
    while not detect_line(robot):
        motor.motor_left(1, DIR_FORWARD, 30)
        motor.motor_right(1, DIR_FORWARD, 30)
        time.sleep(0.05)
    motor.motorStop()

    sc.moveAngle(0, 0)
    time.sleep(0.2)

def turn_left(robot, sc, speed=40):
    print("Executing left turn...")

    # Recenter on intersection
    print("Recentering on intersection...")
    time.sleep(0.5)
    sc.moveAngle(0, 0)
    motor.motor_left(1, DIR_BACKWARD, 22)
    motor.motor_right(1, DIR_BACKWARD, 22)
    while not detect_intersection(robot):
        time.sleep(0.05)
    motor.motorStop()
    time.sleep(0.5)

    # Maneuver 1
    sc.moveAngle(0, 120)
    time.sleep(0.3)
    motor.motor_left(1, DIR_FORWARD, speed)
    motor.motor_right(1, DIR_FORWARD, speed)
    time.sleep(0.52)
    motor.motorStop()
    time.sleep(0.3)

    # Maneuver 2
    sc.moveAngle(0, -120)
    time.sleep(0.3)
    motor.motor_left(1, DIR_BACKWARD, speed)
    motor.motor_right(1, DIR_BACKWARD, speed)
    time.sleep(0.45)
    motor.motorStop()
    time.sleep(0.3)

    # Maneuver 3
    sc.moveAngle(0, 70)
    time.sleep(0.3)
    motor.motor_left(1, DIR_FORWARD, speed)
    motor.motor_right(1, DIR_FORWARD, speed)
    time.sleep(0.6)

    # Final maneuver with line detection
    sc.moveAngle(0, 80)
    time.sleep(0.2)
    while not detect_line(robot):
        motor.motor_left(1, DIR_FORWARD, 20)
        motor.motor_right(1, DIR_FORWARD, 20)
        time.sleep(0.05)
    motor.motorStop()

# POSITION TRACKING

class PositionTracker:
    def __init__(self, start_pos, start_dir):
        self.current_pos = start_pos
        self.current_dir = start_dir
        self.directions = ["N", "E", "S", "W"]

    def turn_left(self):
        idx = self.directions.index(self.current_dir)
        self.current_dir = self.directions[(idx - 1) % 4]

    def turn_right(self):
        idx = self.directions.index(self.current_dir)
        self.current_dir = self.directions[(idx + 1) % 4]

    def move_forward(self):
        next_pos = neighbors(self.current_pos).get(self.current_dir)
        if next_pos:
            self.current_pos = next_pos

    def update(self, action):
        """Update position based on action taken"""
        if action == "forward":
            self.move_forward()
        elif action == "left":
            self.turn_left()
            self.move_forward()
        elif action == "right":
            self.turn_right()
            self.move_forward()


# MAIN PATH FOLLOWING WITH DYNAMIC RECALCULATION

def follow_path_with_obstacles(robot, sc, ultrasonic, oled, start, start_dir, end,
                                initial_blocked_nodes=None, initial_blocked_edges=None):

    if initial_blocked_nodes is None:
        initial_blocked_nodes = set()
    if initial_blocked_edges is None:
        initial_blocked_edges = set()

    blocked_edges = initial_blocked_edges.copy()
    blocked_nodes = initial_blocked_nodes.copy()

    tracker = PositionTracker(start, start_dir)

    # Initial path calculation
    path = compute_moves(start, start_dir, end, blocked_nodes, blocked_edges)
    if path is None:
        print("ERROR: No valid path exists!")
        return False

    print(f"Initial path: {path}")
    i = 0
    intersection_handled = False

    while i < len(path):
        left, middle, right = robot.read_sensors()
        is_at_intersection = (left == 0 and middle == 0 and right == 0)

        if is_at_intersection and not intersection_handled:
            print(f"\n=== INTERSECTION DETECTED at {tracker.current_pos} facing {tracker.current_dir} ===")
            robot.stop()
            time.sleep(0.5)

            # Recenter: move backward until we detect the intersection again
            print("Recentering on intersection...")
            sc.moveAngle(0, 0)
            time.sleep(0.3)

            # Continue backward until intersection detected again
            while True:
                l, m, r = robot.read_sensors()
                motor.motor_left(1, DIR_BACKWARD, 22)
                motor.motor_right(1, DIR_BACKWARD, 22)
                if l == 0 and m == 0 and r == 0:
                    print("Intersection re-detected - stopping for scan")
                    motor.motorStop()
                    break
                time.sleep(0.05)

            time.sleep(0.5)

            # Scan for obstacles
            dist_front, dist_left, dist_right = scan_for_obstacles(robot, ultrasonic, oled)

            # Update LEDs
            indicate_obstacles(dist_front, dist_left, dist_right, DANGER_DISTANCE)

            new_blocked = get_blocked_edges_from_scan(
                tracker.current_pos, tracker.current_dir,
                dist_front, dist_left, dist_right
            )

            needs_recalc = False
            if new_blocked:
                print(f"\nNew obstacles detected! Adding {len(new_blocked)} blocked edges.")
                blocked_edges.update(new_blocked)
                needs_recalc = True

            oled.set_all_lines([
                f"Pos: {tracker.current_pos}",
                f"F:{dist_front if dist_front else '---'}",
                f"L:{dist_left if dist_left else '---'}",
                f"R:{dist_right if dist_right else '---'}",
                "Recalc..." if needs_recalc else "Following...",
                ""
            ])
            time.sleep(1.0)

            if needs_recalc:
                print(f"\nRecalculating path from {tracker.current_pos} to {end}...")
                new_path = compute_moves(
                    tracker.current_pos, tracker.current_dir, end,
                    blocked_nodes, blocked_edges
                )

                if new_path is None:
                    print("ERROR: No valid path found after recalculation!")
                    oled.set_all_lines([
                        "ERROR",
                        "No path exists!",
                        "All routes blocked",
                        "",
                        "",
                        ""
                    ])
                    time.sleep(3.0)
                    return False

                print(f"New path calculated: {new_path}")
                path = new_path
                i = 0

            if i >= len(path):
                break

            direction = path[i]
            i += 1

            if direction == 'stop':
                print("=== DESTINATION REACHED ===")
                robot.stop()
                oled.set_all_lines([
                    "ARRIVED",
                    f"Destination: {end}",
                    "Mission complete!",
                    "",
                    "",
                    ""
                ])
                return True

            elif direction == 'right':
                print("Action: RIGHT TURN")
                turn_right(robot, sc)
                tracker.update('right')
                intersection_handled = True

            elif direction == 'left':
                print("Action: LEFT TURN")
                turn_left(robot, sc)
                tracker.update('left')
                intersection_handled = True

            elif direction == 'forward':
                print("Action: FORWARD")
                motor.motor_left(1, DIR_FORWARD, 22)
                motor.motor_right(1, DIR_FORWARD, 22)
                time.sleep(0.5)
                motor.motorStop()
                tracker.update('forward')
                intersection_handled = True

            print(f"New position: {tracker.current_pos}, facing: {tracker.current_dir}")

        elif not is_at_intersection:
            intersection_handled = False
            robot.run()

        time.sleep(0.01)

    return True


# MAIN
def main():

    switchSetup()
    set_all_leds_off()

    robot = LineFollowingRobotPID()
    sc = servo.ServoCtrl()
    ultrasonic = UltrasonicSensor()
    oled = OLED_Display()

    START = (0, 0)
    START_DIR = "W"
    GOAL = (1, 0)

    BLOCKED_NODES = set()
    BLOCKED_EDGES = set()

    print("DYNAMIC PATHFINDING WITH OBSTACLE DETECTION")
    print(f"Start: {START}, Direction: {START_DIR}")
    print(f"Goal: {GOAL}")
    print(f"Danger distance: {DANGER_DISTANCE} cm")

    try:
        robot.gradual_start(target_speed=20)
        success = follow_path_with_obstacles(
            robot, sc, ultrasonic, oled,
            START, START_DIR, GOAL,
            BLOCKED_NODES, BLOCKED_EDGES
        )

        if success:
            print("\n=== MISSION COMPLETE ===")
        else:
            print("\n=== MISSION FAILED ===")

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        set_all_leds_off()
        robot.cleanup()
        ultrasonic.cleanup()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
