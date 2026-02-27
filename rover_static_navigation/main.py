import numpy as np
import matplotlib.pyplot as plt

from models import update_state
from controller import PID
from planner import compute_guidance
from safest_path_planner import compute_safest_guidance
from analysis import compute_path_length

# ============================================================
# Time
# ============================================================
t = np.linspace(0, 200, 10000)
dt = t[1] - t[0]

# ============================================================
# Goal
# ============================================================
x_goal = 800
y_goal = 800

# ============================================================
# Obstacles
# ============================================================
obstacles = [(400, 400), (600, 300), (300, 650)]

rho0 = 120
eta = 8000
lam = 1.0
beta = 8000
epsilon = 1e-2

# ============================================================
# Limits
# ============================================================
v_max = 40
a_max = 15
omega_max = 3

# ============================================================
# Controllers
# ============================================================
heading_pid = PID(5, 0, 1, omega_max)
velocity_pid = PID(2, 0, 0.5, a_max)


# ============================================================
# BASELINE SIMULATION
# ============================================================
x = np.zeros_like(t)
y = np.zeros_like(t)
theta = np.zeros_like(t)
v = np.zeros_like(t)

clearance = np.zeros_like(t)

for i in range(1, len(t)):

    Fx, Fy, rho_min = compute_guidance(
        x[i-1], y[i-1],
        x_goal, y_goal,
        obstacles,
        rho0, eta, lam,
        beta, epsilon
    )

    clearance[i] = rho_min

    distance = np.sqrt(
        (x_goal - x[i-1])**2 +
        (y_goal - y[i-1])**2
    )

    if distance < 2:
        break

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

end_index = i

x = x[:end_index]
y = y[:end_index]
clearance = clearance[:end_index]


# ============================================================
# SAFEST PATH SIMULATION
# ============================================================
# Reinitialize PID controllers for fair comparison
heading_pid = PID(5, 0, 1, omega_max)
velocity_pid = PID(2, 0, 0.5, a_max)

x_safe = np.zeros_like(t)
y_safe = np.zeros_like(t)
theta_safe = np.zeros_like(t)
v_safe = np.zeros_like(t)

clearance_safe = np.zeros_like(t)

for j in range(1, len(t)):

    Fx, Fy, rho_min = compute_safest_guidance(
        x_safe[j-1], y_safe[j-1],
        x_goal, y_goal,
        obstacles,
        rho0, eta, lam,
        beta, epsilon
    )

    clearance_safe[j] = rho_min

    distance = np.sqrt(
        (x_goal - x_safe[j-1])**2 +
        (y_goal - y_safe[j-1])**2
    )

    if distance < 2:
        break

    theta_desired = np.arctan2(Fy, Fx)

    error_theta = np.arctan2(
        np.sin(theta_desired - theta_safe[j-1]),
        np.cos(theta_desired - theta_safe[j-1])
    )

    omega = heading_pid.compute(error_theta, dt)

    v_desired = min(np.sqrt(Fx**2 + Fy**2), v_max)
    error_v = v_desired - v_safe[j-1]

    a = velocity_pid.compute(error_v, dt)

    x_safe[j], y_safe[j], theta_safe[j], v_safe[j] = update_state(
        x_safe[j-1], y_safe[j-1],
        theta_safe[j-1], v_safe[j-1],
        a, omega, dt, v_max
    )

end_index_safe = j

x_safe = x_safe[:end_index_safe]
y_safe = y_safe[:end_index_safe]
clearance_safe = clearance_safe[:end_index_safe]


# ============================================================
# PLOTS
# ============================================================

# Trajectory comparison
plt.figure(figsize=(7,7))
plt.plot(x, y, label="Baseline Planner")
plt.plot(x_safe, y_safe, label="Safest Path Planner")

for (xo, yo) in obstacles:
    circle = plt.Circle((xo, yo), rho0, fill=False)
    plt.gca().add_patch(circle)

plt.scatter(x_goal, y_goal, marker='x')
plt.title("Baseline vs Safest Path")
plt.xlabel("X")
plt.ylabel("Y")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.show()


# Clearance comparison
plt.figure()
plt.plot(clearance, label="Baseline")
plt.plot(clearance_safe, label="Safest")
plt.title("Minimum Clearance Comparison")
plt.xlabel("Time Step")
plt.ylabel("Clearance")
plt.grid(True)
plt.legend()
plt.show()


# Path length comparison
length_baseline = compute_path_length(x, y)
length_safest = compute_path_length(x_safe, y_safe)

print("Baseline Path Length:", length_baseline)
print("Safest Path Length:", length_safest)
