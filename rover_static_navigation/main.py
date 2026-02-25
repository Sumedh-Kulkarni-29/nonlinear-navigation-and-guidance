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

# Logs
omega_log = np.zeros_like(t)
a_log = np.zeros_like(t)
distance_log = np.zeros_like(t)
clearance = np.zeros_like(t)

# Goal
x_goal = 800
y_goal = 800

# Obstacles
obstacles = [(400, 400), (600, 300), (300, 650)]

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

# Simulation Loop
for i in range(1, len(t)):

    Fx, Fy, rho_min = compute_guidance(
        x[i-1], y[i-1],
        x_goal, y_goal,
        obstacles,
        rho0, eta, lam,
        beta, epsilon
    )

    clearance[i] = rho_min

    distance_log[i] = np.sqrt(
        (x_goal - x[i-1])**2 +
        (y_goal - y[i-1])**2
    )

    if distance_log[i] < 2:
        end_index = i
        break

    theta_desired = np.arctan2(Fy, Fx)

    error_theta = np.arctan2(
        np.sin(theta_desired - theta[i-1]),
        np.cos(theta_desired - theta[i-1])
    )

    omega = heading_pid.compute(error_theta, dt)
    omega_log[i] = omega

    v_desired = min(np.sqrt(Fx**2 + Fy**2), v_max)
    error_v = v_desired - v[i-1]

    a = velocity_pid.compute(error_v, dt)
    a_log[i] = a

    x[i], y[i], theta[i], v[i] = update_state(
        x[i-1], y[i-1], theta[i-1], v[i-1],
        a, omega, dt, v_max
    )

else:
    end_index = len(t)

# Slice arrays
t = t[:end_index]
x = x[:end_index]
y = y[:end_index]
theta = theta[:end_index]
v = v[:end_index]
omega_log = omega_log[:end_index]
a_log = a_log[:end_index]
distance_log = distance_log[:end_index]
clearance = clearance[:end_index]

# Trajectory
plt.figure(figsize=(6,6))
plt.plot(x, y, label="Rover Path")
plt.scatter(x_goal, y_goal, marker='x', color='red', label="Goal")

for (xo, yo) in obstacles:
    circle = plt.Circle((xo, yo), rho0, fill=False, linestyle='--')
    plt.gca().add_patch(circle)

plt.title("Rover Trajectory")
plt.xlabel("X")
plt.ylabel("Y")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.show()

# Distance to goal
plt.figure()
plt.plot(t, distance_log)
plt.title("Distance to Goal vs Time")
plt.xlabel("Time")
plt.ylabel("Distance")
plt.grid(True)
plt.show()

# Velocity
plt.figure()
plt.plot(t, v)
plt.title("Velocity vs Time")
plt.xlabel("Time")
plt.ylabel("Velocity")
plt.grid(True)
plt.show()

# Heading
plt.figure()
plt.plot(t, theta)
plt.title("Heading vs Time")
plt.xlabel("Time")
plt.ylabel("Theta (rad)")
plt.grid(True)
plt.show()

# Angular velocity
plt.figure()
plt.plot(t, omega_log)
plt.title("Angular Velocity vs Time")
plt.xlabel("Time")
plt.ylabel("Omega")
plt.grid(True)
plt.show()

# Acceleration
plt.figure()
plt.plot(t, a_log)
plt.title("Acceleration vs Time")
plt.xlabel("Time")
plt.ylabel("Acceleration")
plt.grid(True)
plt.show()

# Clearance
plt.figure()
plt.plot(t, clearance)
plt.title("Minimum Clearance vs Time")
plt.xlabel("Time")
plt.ylabel("Clearance")
plt.grid(True)
plt.show()

print("Path Length:", compute_path_length(x, y))
# ============================================================
# Trajectory Variation with Lambda
# ============================================================

lambda_values = [0, 2000, 8000, 15000]

plt.figure(figsize=(7,7))

for lam_test in lambda_values:

    x_test = np.zeros_like(t)
    y_test = np.zeros_like(t)
    theta_test = np.zeros_like(t)
    v_test = np.zeros_like(t)

    for i in range(1, len(t)):

        Fx, Fy, rho_min = compute_guidance(
            x_test[i-1], y_test[i-1],
            x_goal, y_goal,
            obstacles,
            rho0, eta, lam_test,
            beta, epsilon
        )

        distance = np.sqrt(
            (x_goal - x_test[i-1])**2 +
            (y_goal - y_test[i-1])**2
        )

        if distance < 2:
            break

        theta_desired = np.arctan2(Fy, Fx)

        error_theta = np.arctan2(
            np.sin(theta_desired - theta_test[i-1]),
            np.cos(theta_desired - theta_test[i-1])
        )

        omega = heading_pid.compute(error_theta, dt)

        v_desired = min(np.sqrt(Fx**2 + Fy**2), v_max)
        error_v = v_desired - v_test[i-1]
        a = velocity_pid.compute(error_v, dt)

        x_test[i], y_test[i], theta_test[i], v_test[i] = update_state(
            x_test[i-1], y_test[i-1], theta_test[i-1], v_test[i-1],
            a, omega, dt, v_max
        )

    plt.plot(x_test[:i], y_test[:i], label=f"lambda = {lam_test}")

# Draw obstacles
for (xo, yo) in obstacles:
    circle = plt.Circle((xo, yo), rho0, fill=False)
    plt.gca().add_patch(circle)

plt.scatter(x_goal, y_goal, marker='x')
plt.title("Trajectory Variation with Lambda")
plt.xlabel("X")
plt.ylabel("Y")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.show()
