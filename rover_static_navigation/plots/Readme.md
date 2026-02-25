# Simulation Plots and Analysis

This folder contains the output visualizations generated from the rover static navigation simulation.

The plots illustrate the nonlinear guidance behavior, control response, and safety-performance tradeoffs.

---

## 1. Rover Trajectory

Shows the 2D path followed by the rover from the initial position to the goal.

- Circular dashed regions indicate obstacle influence zones.
- The trajectory curvature reflects the artificial potential field behavior.
- Higher repulsive gain (λ) results in wider detours around obstacles.

This plot demonstrates how nonlinear guidance reshapes navigation geometry.

---

## 2. Distance to Goal vs Time

Represents convergence behavior.

- Monotonic decrease indicates stable navigation.
- Oscillations would indicate overly aggressive controller gains.
- The final flattening corresponds to goal tolerance threshold.

This confirms closed-loop stability.

---

## 3. Velocity vs Time

Shows dynamic speed profile.

- Initial acceleration phase.
- Mid-course adaptation based on guidance vector magnitude.
- Deceleration near goal.

Validates cascade control structure.

---

## 4. Heading vs Time

Illustrates steering convergence.

- Smooth transition indicates stable angular PID control.
- Rapid spikes correspond to obstacle avoidance maneuvers.

---

## 5. Angular Velocity vs Time

Represents steering control effort.

- Bounded within actuator limits.
- Saturation indicates aggressive turning near obstacles.

---

## 6. Acceleration vs Time

Represents longitudinal control effort.

- Saturation at ±a_max demonstrates actuator limiting.
- Smooth decay near goal indicates stable velocity regulation.

---

## 7. Minimum Clearance vs Time

Tracks safety margin relative to nearest obstacle.

- Higher λ increases minimum clearance.
- Demonstrates safety-performance tradeoff.

---

## 8. Trajectory Variation with Lambda

Compares rover paths for different repulsive gain values (λ).

Observations:

- λ = 0 → minimal obstacle avoidance.
- Moderate λ → balanced path.
- Large λ → exaggerated avoidance behavior.

This highlights nonlinear sensitivity to repulsive gain.

---

# Key Observations

1. The system exhibits stable convergence under cascade PID control.
2. Artificial potential fields successfully avoid static obstacles.
3. Increasing λ increases safety but also increases path length.
4. The navigation system is locally reactive and does not guarantee global optimality.

---

# Limitations

- No sensor noise or disturbances modeled.
- No global path planner (A*, RRT).
- Potential local minima in complex environments.
- Static obstacle assumption.

---

This analysis provides insight into nonlinear navigation tradeoffs between efficiency and safety.
