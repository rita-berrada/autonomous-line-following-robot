# MANHATTAN USING BFS WITH BLOCKED EDGES AND INTERSECTIONS

import time
import RPi.GPIO as GPIO
import sys
import Adafruit_PCA9685
import DC_motors as motor
import RPIservo_vS as servo
from PID_inter import LineFollowingRobotPID
from collections import deque

DIR_FORWARD = 0
DIR_BACKWARD = 1


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

        # Move forward in current direction
        next_pos_forward = neighbors(current_pos).get(current_dir)
        if next_pos_forward and can_move(current_pos, next_pos_forward):
            state = (next_pos_forward, current_dir)
            if state not in visited:
                visited.add(state)
                queue.append((next_pos_forward, current_dir, actions + ["forward"]))

        # Turn left and move forward in new direction
        new_dir_left = turn_left(current_dir)
        next_pos_left = neighbors(current_pos).get(new_dir_left)
        if next_pos_left and can_move(current_pos, next_pos_left):
            state = (next_pos_left, new_dir_left)
            if state not in visited:
                visited.add(state)
                queue.append((next_pos_left, new_dir_left, actions + ["left"]))

        # Turn right and move forward in new direction
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
    actions = bfs_path(start, start_dir, end, blocked_nodes, blocked_edges)
    if actions is None:
        print("No valid path found.")
        return None

    actions.append("stop")
    return actions

def detect_intersection(robot):
    left, middle, right = robot.read_sensors()
    return left == 0 and middle == 0 and right == 0

def detect_line(robot):
    left, middle, right = robot.read_sensors()
    return left == 0 or middle == 0 or right == 0

def turn_right(robot, sc, speed=40):
    print("Exécution virage à droite...")

    # Go backwards until the lign is detected
    print("Recentrage sur l'intersection...")
    time.sleep(0.5)
    sc.moveAngle(0, 0)  # Right wheels
    motor.motor_left(1, DIR_BACKWARD, 20)
    motor.motor_right(1, DIR_BACKWARD, 20)
    while not detect_intersection(robot):
        time.sleep(0.05)
    motor.motorStop()
    time.sleep(0.5)


    # 1st Maneuver: Wheels to the right, go forward
    sc.moveAngle(0, -120)
    time.sleep(0.3)
    motor.motor_left(1, DIR_FORWARD, speed)
    motor.motor_right(1, DIR_FORWARD, speed)
    time.sleep(0.5)
    motor.motorStop()
    time.sleep(0.3)

    # 2nd Maneuver: Wheels to the left, go backwards
    sc.moveAngle(0, 120)
    time.sleep(0.3)
    motor.motor_left(1, DIR_BACKWARD, speed)
    motor.motor_right(1, DIR_BACKWARD, speed)
    time.sleep(0.50)
    motor.motorStop()
    time.sleep(0.3)

    # 3rd Maneuver: wheels to the right, go forward
    sc.moveAngle(0, -80)
    time.sleep(0.3)
    motor.motor_left(1, DIR_FORWARD, speed)
    motor.motor_right(1, DIR_FORWARD, speed)
    time.sleep(0.52)

    # 4th Maneuver: Wheels to the right, go forward until lign detected
    sc.moveAngle(0, -70)
    time.sleep(0.2)
    while not detect_line(robot):
        motor.motor_left(1, DIR_FORWARD, 30)
        motor.motor_right(1, DIR_FORWARD, 30)
        time.sleep(0.05)
    motor.motorStop()

    # Recalibrates the wheels staright
    sc.moveAngle(0, 0)
    time.sleep(0.2)

def turn_left(robot, sc, speed=40):
    """Virage à gauche avec détection de ligne sur la dernière manœuvre"""
    print("Exécution virage à gauche...")

    # Go backwards until intersection detected
    print("Recentrage sur l'intersection...")
    time.sleep(0.5)
    sc.moveAngle(0, 0)  # Right Wheels
    motor.motor_left(1, DIR_BACKWARD, 20)
    motor.motor_right(1, DIR_BACKWARD, 20)
    while not detect_intersection(robot):
        time.sleep(0.05)
    motor.motorStop()
    time.sleep(0.5)


    # 1st Maneuver: Wheels to the left, go forward

    sc.moveAngle(0, 120)
    time.sleep(0.3)
    motor.motor_left(1, DIR_FORWARD, speed)
    motor.motor_right(1, DIR_FORWARD, speed)
    time.sleep(0.52)
    motor.motorStop()
    time.sleep(0.3)

    # 2nd Maneuver: Wheels to the right, go backwards
    sc.moveAngle(0, -110)
    time.sleep(0.3)
    motor.motor_left(1, DIR_BACKWARD, speed)
    motor.motor_right(1, DIR_BACKWARD, speed)
    time.sleep(0.5)
    motor.motorStop()
    time.sleep(0.3)

    # 3rd Maneuver: Wheels to the left, go forward
    sc.moveAngle(0, 80)
    time.sleep(0.3)
    motor.motor_left(1, DIR_FORWARD, speed)
    motor.motor_right(1, DIR_FORWARD, speed)
    time.sleep(0.5)

    # 4th Maneuver: Wheels to the left, go forward until lign detected
    sc.moveAngle(0, 60)
    time.sleep(0.2)
    while not detect_line(robot):
        motor.motor_left(1, DIR_FORWARD, 20)
        motor.motor_right(1, DIR_FORWARD, 20)
        time.sleep(0.05)
    motor.motorStop()

robot = LineFollowingRobotPID()
sc = servo.ServoCtrl()

A = compute_moves(start=(0, 0), start_dir="W", end=(3,1))
print(f"Generated path: {A}")

def follow_path(A):
    i = 0
    while i < len(A):
        if detect_intersection(robot):
            direction = A[i]
            i += 1

            if direction == 'right':
                print("Intersection detected, right, turning!")
                motor.motorStop()
                time.sleep(0.3)
                turn_right(robot, sc)

            elif direction == 'left':
                print("Intersection detected, left, turning!")
                motor.motorStop()
                time.sleep(0.3)
                turn_left(robot, sc)

            elif direction == 'stop':
                print("You have arrived at your destination")
                motor.motorStop()
                break

            elif direction == 'forward':
                print("Intersection detected, direction forward, keep going!")
                motor.motor_left(1, DIR_FORWARD, 18)
                motor.motor_right(1, DIR_FORWARD, 18)
                time.sleep(0.3)
                motor.motorStop()

        else:
            robot.run()

        time.sleep(0.04)

# Main loop
try:
    robot.gradual_start(target_speed=18)
    follow_path(A)

except KeyboardInterrupt:
    print("\nStopping...")
    robot.cleanup()
