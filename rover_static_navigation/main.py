import numpy as np
import matplotlib.pyplot as plt

from models import update_state
from controller import PID
from planner import compute_guidance
from analysis import compute_path_length

# Time
t = np.linspace(0, 200, 10000)
dt = t[1] - t[0]

# States
x = np.zeros_like(t)
y = np.zeros_like(t)
theta = np.zeros_like(t)
v = np.zeros_like(t)
clearance = np.zeros_like(t)

# Goal
x_goal = 800
y_goal = 800

# Obstacles
obstacles = [(400,400), (600,300), (300,650)]

rho0 = 120
eta = 8000
lam = 1.0
beta = 8000
epsilon = 1e-2

# Limits
v_max = 40
a_max = 15
omega_max = 3

# Controllers
heading_pid = PID(5, 0, 1, omega_max)
velocity_pid = PID(2, 0, 0.5, a_max)

for i in range(1, len(t)):

    Fx, Fy, rho_min = compute_guidance(
        x[i-1], y[i-1],
        x_goal, y_goal,
        obstacles,
        rho0, eta, lam,
        beta, epsilon
    )

    clearance[i] = rho_min

    theta_desired = np.arctan2(Fy, Fx)

    error_theta = np.arctan2(
        np.sin(theta_desired - theta[i-1]),
        np.cos(theta_desired - theta[i-1])
    )

    omega = heading_pid.compute(error_theta, dt)

    v_desired = min(np.sqrt(Fx**2 + Fy**2), v_max)
    error_v = v_desired - v[i-1]
    a = velocity_pid.compute(error_v, dt)

    x[i], y[i], theta[i], v[i] = update_state(
        x[i-1], y[i-1], theta[i-1], v[i-1],
        a, omega, dt, v_max
    )

# Plots
plt.figure(figsize=(8,8))
plt.plot(x, y)
plt.scatter(x_goal, y_goal, marker='x')
plt.axis("equal")
plt.grid(True)
plt.title("Rover Trajectory")
plt.show()

plt.figure()
plt.plot(t, clearance)
plt.title("Minimum Clearance vs Time")
plt.grid(True)
plt.show()

print("Path Length:", compute_path_length(x, y))
