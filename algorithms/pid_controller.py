"""
PID Controller — reusable algorithm extracted from the line-following robot.

Sensor error convention:
  -2.0  strong left deviation   (0,1,1)
  -1.0  slight left deviation   (0,0,1)
   0.0  centred                 (1,0,1)
  +1.0  slight right deviation  (1,0,0)
  +2.0  strong right deviation  (1,1,0)
"""

import time


class PIDController:
    """Generic discrete PID controller with integral clamping and history-based I term."""

    def __init__(self, kp=7.5, ki=2.5, kd=2.5,
                 max_output=25, integral_limit=40, history_size=10):
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd

        self.max_output = max_output
        self.integral_limit = integral_limit
        self.history_size = history_size

        self.error_history = []
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()

    def compute(self, error):
        """Compute PID correction for the given error value.

        Returns:
            correction (float): clamped output in [-max_output, +max_output]
            P, I, D (float): individual term contributions (for logging/tuning)
        """
        current_time = time.time()
        dt = max(current_time - self.last_time, 0.001)
        self.last_time = current_time

        # Proportional
        P = self.Kp * error

        # Integral (history-based sum, clamped)
        self.error_history.append(error)
        if len(self.error_history) > self.history_size:
            self.error_history.pop(0)
        self.integral = sum(self.error_history)
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        I = self.Ki * self.integral

        # Derivative
        D = self.Kd * (error - self.previous_error)
        self.previous_error = error

        correction = P + I + D
        correction = max(-self.max_output, min(self.max_output, correction))
        return correction, P, I, D

    def reset(self):
        self.error_history.clear()
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
