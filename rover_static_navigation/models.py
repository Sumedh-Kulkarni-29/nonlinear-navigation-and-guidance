import numpy as np

def update_state(x, y, theta, v, a, omega, dt, v_max):
    """
    Unicycle model with acceleration dynamics.
    """

    v = v + a * dt
    v = np.clip(v, 0, v_max)

    theta = theta + omega * dt

    x = x + v * np.cos(theta) * dt
    y = y + v * np.sin(theta) * dt

    return x, y, theta, v
