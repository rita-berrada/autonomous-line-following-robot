# FINAL PID ( THE 8)
import time
import RPi.GPIO as GPIO
import RPIservo_vS as servo
import DC_motors as motor


class LineFollowingRobotPID:

    def __init__(self, left_pin=19, middle_pin=16, right_pin=20, servo_id=0):

        self.left_pin = left_pin
        self.middle_pin = middle_pin
        self.right_pin = right_pin
        self.servo_id = servo_id


        self.base_angles = {
            'straight': 0,
            'left_slight': 50,
            'left_sharp': 105,
            'right_slight': -50,
            'right_sharp': -110
        }

        self.error_history = []
        self.max_error_history = 10

        self.speed_normal = 25
        self.speed_turn = 25

        self.Kp = 7.5
        self.Ki = 2.5
        self.Kd = 2.5

        self.max_angle_correction = 25
        self.integral_limit = 40

        self.error = 0
        self.previous_error = 0
        self.integral = 0
        self.derivative = 0
        self.last_time = time.time()

        self.last_action = 'STRAIGHT'
        self.last_base_angle = self.base_angles['straight']

        self.servo = servo.ServoCtrl()
        self.setup()

    def setup(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.left_pin, GPIO.IN)
        GPIO.setup(self.middle_pin, GPIO.IN)
        GPIO.setup(self.right_pin, GPIO.IN)
        motor.setup()
        self.servo.moveInit()

    def read_sensors(self):
        return (
            GPIO.input(self.left_pin),
            GPIO.input(self.middle_pin),
            GPIO.input(self.right_pin)
        )

    def compute_error(self, left, middle, right):
        sensor_pattern = (left, middle, right)

        if sensor_pattern == (1, 0, 1):
            error = 0.0
        elif sensor_pattern == (1, 0, 0):
            error = 1.0  # Slight right deviation
        elif sensor_pattern == (1, 1, 0):
            error = 2.0  # Strong right deviation
        elif sensor_pattern == (0, 1, 0):
            if self.previous_error > 0:
                error = 0.5
            elif self.previous_error < 0:
                error = -0.5
            else:
                error = 0.0
        elif sensor_pattern == (0, 1, 1):
            error = -2.0  # Strong left deviation
        elif sensor_pattern == (0, 0, 1):
            error = -1.0  # Slight left deviation
        elif sensor_pattern == (1, 1, 1):
            error = self.previous_error
        elif sensor_pattern == (0, 0, 0):
            error = self.previous_error
        else:
            error = self.previous_error

        return error

    def compute_pid(self, error):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        if dt < 0.001:
            dt = 0.001

        P = self.Kp * error

        self.error_history.append(error)
        if len(self.error_history) > self.max_error_history:
            self.error_history.pop(0)

        self.integral = sum(self.error_history)
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        I = self.Ki * self.integral

        self.derivative = (error - self.previous_error)
        D = self.Kd * self.derivative

        correction = P + I + D
        correction = max(-self.max_angle_correction, min(self.max_angle_correction, correction))

        return correction, P, I, D

    def determine_base_action(self, left, middle, right):
        if left == 1 and middle == 0 and right == 1:
            return 'STRAIGHT', self.base_angles['straight'], self.speed_normal
        elif left == 1 and middle == 0 and right == 0:
            return 'RIGHT_SLIGHT', self.base_angles['right_slight'], self.speed_turn
        elif left == 1 and middle == 1 and right == 0:
            return 'RIGHT_SHARP', self.base_angles['right_sharp'], self.speed_turn
        elif left == 0 and middle == 0 and right == 1:
            return 'LEFT_SLIGHT', self.base_angles['left_slight'], self.speed_turn
        elif left == 0 and middle == 1 and right == 1:
            return 'LEFT_SHARP', self.base_angles['left_sharp'], self.speed_turn
        else:
            return 'SEARCHING', self.last_base_angle, self.speed_turn

    def calculate_steering_angle(self, base_angle, correction):
        # Le PID ajoute sa correction à l'angle de base
        return base_angle + correction

    def steer(self, angle):
        self.servo.moveAngle(self.servo_id, angle)

    def move_forward(self, speed):
        motor.motor_left(1, motor.Dir_forward, speed)
        motor.motor_right(1, motor.Dir_forward, speed)

    def stop(self):
        motor.motorStop()

    def gradual_start(self, target_speed, step=10, delay=0.03):
        current_speed = 0
        while current_speed < target_speed:
            current_speed = min(current_speed + step, target_speed)
            self.move_forward(current_speed)
            time.sleep(delay)

    def gradual_stop(self, current_speed, step=10, delay=0.03):
        while current_speed > 0:
            current_speed = max(current_speed - step, 0)
            if current_speed > 0:
                self.move_forward(current_speed)
            else:
                self.stop()
            time.sleep(delay)

    def run(self):
        left, middle, right = self.read_sensors()

        self.error = self.compute_error(left, middle, right)
        correction, P, I, D = self.compute_pid(self.error)

        action, base_angle, base_speed = self.determine_base_action(left, middle, right)
        final_angle = self.calculate_steering_angle(base_angle, correction)

        self.steer(final_angle)
        self.move_forward(base_speed)

        self.previous_error = self.error
        self.last_action = action
        self.last_base_angle = base_angle


    def cleanup(self):
        self.stop()
        self.steer(self.base_angles['straight'])
        self.integral = 0
        GPIO.cleanup()


def main():
    robot = LineFollowingRobotPID()

    try:
        print("\n" + "="*100)
        print("PID LINE FOLLOWING ROBOT - ADAPTED TO NEW CIRCUIT")
        print("="*100)
        print(f"PID: Kp={robot.Kp}, Ki={robot.Ki}, Kd={robot.Kd}")
        print(f"Error range: -2 to +2")
        print(f"Speed: Normal={robot.speed_normal}, Turn={robot.speed_turn}")
        print(f"Max correction: ±{robot.max_angle_correction}°")
        print("="*100 + "\n")

        robot.gradual_start(target_speed=40)

        while True:
            robot.run()

    except KeyboardInterrupt:
        print("\n\nStopping...")
        robot.gradual_stop(current_speed=40)
        robot.cleanup()


if __name__ == '__main__':
    main()
