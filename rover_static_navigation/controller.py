import numpy as np

class PID:
    def __init__(self, Kp, Ki, Kd, limit=None):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.limit = limit

        self.integral = 0
        self.prev_error = 0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        output = (
            self.Kp * error +
            self.Ki * self.integral +
            self.Kd * derivative
        )

        if self.limit is not None:
            output = np.clip(output, -self.limit, self.limit)

        return output
