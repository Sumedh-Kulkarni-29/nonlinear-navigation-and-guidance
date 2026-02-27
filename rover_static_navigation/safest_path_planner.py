import numpy as np
from planner import compute_guidance

def compute_safest_guidance(
    x, y,
    x_goal, y_goal,
    obstacles,
    rho0, eta, lam,
    beta, epsilon,
    safety_gain=0.2
):
    """
    Magnitude-weighted clearance guidance.
    Uses baseline planner but scales attraction
    based on total force magnitude.
    """

    Fx, Fy, rho_min = compute_guidance(
        x, y,
        x_goal, y_goal,
        obstacles,
        rho0, eta, lam,
        beta, epsilon
    )

    # Magnitude-based safety weight
    force_magnitude = np.sqrt(Fx**2 + Fy**2)
    weight = 1 + safety_gain * force_magnitude

    Fx_safe = Fx / weight
    Fy_safe = Fy / weight

    return Fx_safe, Fy_safe, rho_min
