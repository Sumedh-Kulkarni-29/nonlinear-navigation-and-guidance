# System Limitations

This document outlines the current limitations of the rover static navigation framework.

The objective is to clearly identify assumptions and boundaries of the present implementation.

---

# 1. Static Environment Assumption

The system assumes:

- All obstacles are static
- Obstacle positions are known a priori
- No moving agents are present

Dynamic obstacles or time-varying environments are not modeled.

---

# 2. No Sensor Modeling

The simulation assumes perfect state knowledge:

- Exact rover position
- Exact obstacle positions
- No measurement noise
- No delay

In real systems, state estimation (e.g., Kalman filtering) would be required.

---

# 3. Local Navigation Method

Artificial potential fields are local methods.

They do not guarantee:

- Global optimality
- Shortest path
- Freedom from local minima in complex geometries

In highly cluttered environments, the rover may get trapped.

---

# 4. Safety–Efficiency Tradeoff

The rover follows the direction of the **minimum potential gradient**, not necessarily the globally safest path.

This distinction is important.

The artificial potential field computes:

    F_total = F_goal + λ F_rep

The rover moves along the steepest descent direction of this combined potential.

However:

- The minimum potential direction is a **local decision**
- It does not evaluate full trajectory safety
- It does not anticipate narrow passages ahead

## Example from Simulation

In the "Trajectory Variation with Lambda" plot:

- For moderate λ values, the rover passes through the narrow gap between two obstacle influence regions.
- This path has lower potential compared to a wide detour.
- Therefore, the rover selects it.

But this path may not be the safest in a real-world scenario.

Why?

Because:

- Clearance margin may be small
- Any disturbance or sensor noise could cause collision
- The method does not evaluate worst-case safety

In contrast, a globally safer path might go around both obstacles entirely, even if it is longer.

The artificial potential method does not reason about that.

It simply follows:

    direction = −∇U(x, y)

Where U is the combined potential.

This means the rover follows the path of **least potential**, not necessarily the path of **maximum safety margin**.

---

# 5. No Global Path Planning

The system does not include:

- A* search
- RRT / RRT*
- Graph-based planners
- Sampling-based planners

It is purely reactive and vector-field driven.

---

# 6. No Terrain or Dynamics Constraints

The rover model assumes:

- Flat terrain
- No slip
- No wheel dynamics
- No friction modeling
- No actuator delay

The dynamics are simplified to a unicycle model.

---

# 7. No Energy or Optimality Criteria

The controller does not optimize:

- Energy consumption
- Time optimality
- Path smoothness cost
- Fuel usage

It is stabilization-focused rather than optimization-driven.

---

# 8. No Robustness Analysis

The framework does not yet include:

- Disturbance rejection testing
- External force modeling
- Monte Carlo simulations
- Gain sensitivity robustness analysis

---

# 9. Single-Agent Scenario

Multi-agent coordination, formation control, and collision avoidance between multiple rovers are not considered.

---

# 10. Goal is Static

The target is assumed fixed in space.

Dynamic goal tracking (e.g., spacecraft rendezvous) is not part of this module.

---

# Summary

The current framework demonstrates nonlinear guidance and cascade control for static obstacle avoidance in an idealized 2D environment.

It highlights:

- Safety–efficiency tradeoffs
- Sensitivity to repulsive gain (λ)
- Local potential-based navigation behavior

However, it does not guarantee globally safest or optimal paths.
