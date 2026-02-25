import numpy as np

def compute_guidance(x, y,
                     x_goal, y_goal,
                     obstacles,
                     rho0, eta, lam,
                     beta, epsilon):

    # Attractive force
    Fx_goal = x_goal - x
    Fy_goal = y_goal - y

    Fx_rep_total = 0
    Fy_rep_total = 0
    rho_list = []

    for (xo, yo) in obstacles:
        rho = np.sqrt((x - xo)**2 + (y - yo)**2)
        rho_list.append(rho)

        if rho < rho0:
            rho_safe = max(rho, epsilon)
            rep_gain = eta * (1/rho_safe - 1/rho0) * (1/rho_safe**3)

            Fx_rep_total += rep_gain * (x - xo)
            Fy_rep_total += rep_gain * (y - yo)

    rho_min = min(rho_list)

    # Clearance weighting
    w_clear = 1 + beta / (rho_min**2 + epsilon)

    Fx_goal /= w_clear
    Fy_goal /= w_clear

    Fx_total = Fx_goal + lam * Fx_rep_total
    Fy_total = Fy_goal + lam * Fy_rep_total

    return Fx_total, Fy_total, rho_min
