# Obstacle detection on grid + light games

import time
from test_PID import LineFollowingRobotPID
from mpu6050 import mpu6050
from task8 import UltrasonicSensor, OLED_Display, HeadScanner
import RPi.GPIO as GPIO
import DC_motors as motor
import threading

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
    if port == 0:
        GPIO.output(6, GPIO.HIGH if status else GPIO.LOW)
    elif port == 1:
        GPIO.output(13, GPIO.HIGH if status else GPIO.LOW)
    elif port == 2:
        GPIO.output(5, GPIO.HIGH if status else GPIO.LOW)

#MAIN FUNCTIONS
def is_intersection(left, middle, right):
    return (left, middle, right) == (0, 0, 0)

def get_available_paths(dist_front, dist_left, dist_right, danger_distance):
    available = []

    front_clear = (dist_front is None) or (dist_front >= danger_distance)
    left_clear = (dist_left is None) or (dist_left >= danger_distance)
    right_clear = (dist_right is None) or (dist_right >= danger_distance)

    if front_clear:
        available.append("FRONT")
    if left_clear:
        available.append("LEFT")
    if right_clear:
        available.append("RIGHT")

    if len(available) == 0:
        status = "All blocked!"
    elif len(available) == 1:
        status = f"Only {available[0]}"
    elif len(available) == 2:
        status = f"{available[0]} & {available[1]}"
    else:
        status = "All clear!"

    return available, status

def indicate_obstacles(dist_front, dist_left, dist_right, danger_distance):
    # Blue LED = front obstacle
    front_blocked = (dist_front is not None) and (dist_front < danger_distance)
    set_led(2, front_blocked)

    # Left LED = left obstacle
    left_blocked = (dist_left is not None) and (dist_left < danger_distance)
    set_led(1, left_blocked)

    # Right LED = right obstacle
    right_blocked = (dist_right is not None) and (dist_right < danger_distance)
    set_led(0, right_blocked)

def main():
    # Setup LEDs first
    switchSetup()
    set_all_leds_off()

    robot = LineFollowingRobotPID()
    ultrasonic = UltrasonicSensor()
    oled = OLED_Display()
    head = HeadScanner(servo_id=1, min_angle=-120, max_angle=120, step=10)

    DANGER_DISTANCE = 70.0  # cm
    DISPLAY_TIME = 5.0      # seconds to show message

    try:
        print("GRID INTERSECTION OBSTACLE CHECK WITH LED INDICATORS")
        print("Intersection rule : (0,0,0)")
        print(f"Danger distance   : {DANGER_DISTANCE} cm")
        print("LED indicators:")
        print("  - Blue LED (center) : Front obstacle")
        print("  - Left LED          : Left obstacle")
        print("  - Right LED         : Right obstacle")

        # Clear OLED at startup
        oled.set_all_lines(["", "", "", "", "", ""])

        robot.gradual_start(target_speed=40)

        while True:
            left, middle, right = robot.read_sensors()

            # INTERSECTION DETECTED
            if is_intersection(left, middle, right):
                print("\nIntersection detected, robot halting\n")
                robot.stop()
                time.sleep(0.2)

                # Blink blue LED to indicate intersection detection
                for _ in range(3):
                    set_led(2, True)
                    time.sleep(0.1)
                    set_led(2, False)
                    time.sleep(0.1)

                # Ultrasonic measurement (front)
                distance = ultrasonic.measure()
                print(f"Distance measured (front): {distance}")

                # Display on OLED
                oled.set_all_lines([
                    "INTERSECTION",
                    f"Front: {distance if distance else '---'} cm",
                    "Backing up...",
                    "",
                    "",
                    ""
                ])

                time.sleep(1.0)

                # REVERSE UNTIL INTERSECTION 
                print("\nBacking up to the intersection...\n")
                while True:
                    left_b, mid_b, right_b = robot.read_sensors()

                    if is_intersection(left_b, mid_b, right_b):
                        print("Intersection re-detected, stopping.")
                        robot.stop()
                        time.sleep(0.3)
                        break

                    robot.servo.moveAngle(robot.servo_id, 0)
                    motor.motor_left(1, motor.Dir_backward, 30)
                    motor.motor_right(1, motor.Dir_backward, 30)
                    time.sleep(0.05)

                # HEAD SCAN: Check left and right
                print("\nHEAD SCANNING")

                # Look LEFT (+90°)
                print("Looking left (+90°)")
                robot.servo.moveAngle(1, 120)
                time.sleep(0.8)
                dist_left = ultrasonic.measure()
                print(f"Left: {dist_left} cm")

                # Indicate left scan
                left_blocked = (dist_left is not None) and (dist_left < DANGER_DISTANCE)
                set_led(1, left_blocked)

                oled.set_all_lines([
                    "SCANNING LEFT",
                    f"Dist: {dist_left if dist_left else '---'} cm",
                    "Obstacle" if left_blocked else "Clear",
                    "",
                    "",
                    ""
                ])
                time.sleep(0.5)

                # Look RIGHT (-90°)
                print("Looking right (-90°)")
                robot.servo.moveAngle(1, -120)
                time.sleep(0.8)
                dist_right = ultrasonic.measure()
                print(f"Right: {dist_right} cm")

                # Indicate right scan
                right_blocked = (dist_right is not None) and (dist_right < DANGER_DISTANCE)
                set_led(0, right_blocked)

                oled.set_all_lines([
                    "SCANNING RIGHT",
                    f"Dist: {dist_right if dist_right else '---'} cm",
                    "Obstacle" if right_blocked else "Clear",
                    "",
                    "",
                    ""
                ])
                time.sleep(0.5)

                # Return head to center
                print("Returning head to center")
                robot.servo.moveAngle(1, 0)
                time.sleep(0.5)

                # FINAL LED INDICATION 
                indicate_obstacles(distance, dist_left, dist_right, DANGER_DISTANCE)

                # GET AVAILABLE PATHS 
                available_paths, status = get_available_paths(distance, dist_left, dist_right, DANGER_DISTANCE)

                # Final summary
                print("SCAN SUMMARY")
                print(f"Front: {distance if distance else '---'} cm")
                print(f"Left : {dist_left if dist_left else '---'} cm")
                print(f"Right: {dist_right if dist_right else '---'} cm")
                print(f"AVAILABLE PATHS: {', '.join(available_paths) if available_paths else 'NONE'}")
                print(f"STATUS: {status}")
                print("LED Status:")
                print(f"  - Blue (Front): {'ON' if (distance and distance < DANGER_DISTANCE) else 'OFF'}")
                print(f"  - Left        : {'ON' if (dist_left and dist_left < DANGER_DISTANCE) else 'OFF'}")
                print(f"  - Right       : {'ON' if (dist_right and dist_right < DANGER_DISTANCE) else 'OFF'}")

                # Format paths for OLED
                paths_line = ", ".join(available_paths) if available_paths else "NONE"

                oled.set_all_lines([
                    "PATHS AVAILABLE",
                    paths_line,
                    f"F:{distance if distance else '---'}",
                    f"L:{dist_left if dist_left else '---'}",
                    f"R:{dist_right if dist_right else '---'}",
                    status[:20]
                ])

                time.sleep(DISPLAY_TIME)

                print("Robot stopped at intersection.")
                break

            # NORMAL PID FOLLOWING 
            else:
                robot.run()
                time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nStopping program")

    finally:
        GPIO.setmode(GPIO.BCM)

        # Turn off all LEDs
        set_all_leds_off()

        try:
            head.cleanup()
        except Exception as e:
            print(f"Head cleanup error: {e}")

        try:
            ultrasonic.cleanup()
        except Exception as e:
            print(f"Ultrasonic cleanup error: {e}")

        try:
            robot.cleanup()
        except Exception as e:
            print(f"Robot cleanup error: {e}")

        GPIO.cleanup()

if __name__ == "__main__":
    main()