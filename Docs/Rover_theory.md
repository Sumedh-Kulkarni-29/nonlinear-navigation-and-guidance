# Rover Static Navigation – Theoretical Framework

This document describes the mathematical modeling and control architecture used in the rover static navigation simulation.

The objective is to navigate a rover in a 2D plane from an initial position to a fixed goal while avoiding static obstacles using nonlinear feedback control.

---

# 1. Dynamic Model

The rover is modeled as a nonholonomic unicycle with acceleration-based longitudinal dynamics.

## State Variables

- Position: (x, y)
- Heading angle: θ
- Linear velocity: v

## Equations of Motion

dx/dt = v cos(θ)  
dy/dt = v sin(θ)  
dθ/dt = ω  
dv/dt = a  

Where:
- ω is angular velocity (steering input)
- a is longitudinal acceleration (throttle input)

This model captures heading-constrained ground vehicle behavior.

---

# 2. Artificial Potential Field Guidance

Navigation is generated using artificial potential fields.

The total guidance vector consists of:

1. Attractive force toward the goal  
2. Repulsive forces from obstacles  
3. Clearance-based weighting  

## 2.1 Attractive Component

The attractive force pulls the rover toward the goal:

F_goal = [x_goal − x, y_goal − y]

This creates a vector field pointing toward the target.

---

## 2.2 Repulsive Component

For each obstacle:

If the distance ρ between rover and obstacle is less than the influence radius ρ₀, a repulsive force is generated:

F_rep ∝ η (1/ρ − 1/ρ₀) (1/ρ³) (q − q_obs)

Where:
- ρ = distance to obstacle
- ρ₀ = obstacle influence radius
- η = repulsive gain
- q = rover position
- q_obs = obstacle position

The repulsive force increases rapidly as the rover approaches the obstacle boundary.

---

## 2.3 Clearance-Based Weighting

To bias the system toward safer trajectories, the attractive force is scaled using a clearance-dependent factor:

w_clear = 1 + β / (ρ_min² + ε)

Where:
- ρ_min = minimum distance to any obstacle
- β = safety weighting parameter
- ε = small positive constant

The attractive force becomes:

F_goal ← F_goal / w_clear

This mechanism increases avoidance tendency in narrow passages.

---

# 3. Cascade Control Architecture

The system uses a two-layer feedback structure.

## 3.1 Outer Layer – Guidance

The total force vector determines desired direction and speed:

θ_desired = atan2(F_y, F_x)  
v_desired = ||F||

This converts vector-field guidance into reference signals.

---

## 3.2 Inner Layer – Stabilization

Two PID controllers regulate motion:

### Heading Controller

ω = PID(θ_desired − θ)

Ensures angular alignment with the guidance vector.

### Velocity Controller

a = PID(v_desired − v)

Regulates forward speed based on force magnitude.

This separation simplifies nonlinear steering and stabilization.

---

# 4. Parameter Influence

The repulsive gain (λ) influences trajectory geometry:

- Small λ → goal-dominant behavior
- Moderate λ → balanced navigation
- Large λ → strong avoidance and wider detours

The safety weight (β) modifies how aggressively the rover reduces goal attraction near obstacles.

These parameters define a tradeoff between path efficiency and obstacle clearance.

---

# 5. Design Intent

This framework demonstrates:

- Nonlinear vector-field navigation
- Cascade PID stabilization
- Safety-performance tradeoff shaping
- Sensitivity of trajectory geometry to repulsive gain

It serves as a foundation for extending toward dynamic target tracking and higher-order guidance systems.
