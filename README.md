# Nonlinear Navigation and Guidance

This repository contains simulation-based implementations of nonlinear navigation and guidance systems.  
The project currently includes:

- Static obstacle avoidance for a nonholonomic rover using artificial potential fields
- Cascade PID-based dynamic control
- Safety-aware clearance weighting
- Ongoing development of spacecraft relative motion guidance

The focus of this repository is on modeling, control structure, and analysis rather than high-level robotics frameworks.

---

## Repository Structure

nonlinear-navigation-and-guidance/

├── rover_static_navigation/  
├── spacecraft_relative_guidance/ (in progress)  
├── docs/  
├── requirements.txt  
└── README.md  

---

# 1. Rover Static Navigation

## Problem

Navigate a rover in a 2D plane from an initial position to a fixed goal while avoiding multiple static obstacles.

## Modeling Assumptions

- 2D planar motion
- Nonholonomic unicycle model
- Acceleration-based longitudinal dynamics
- Static circular obstacle regions
- Perfect state measurement
- No sensor noise

## Dynamics

dx/dt = v cos(θ)  
dy/dt = v sin(θ)  
dθ/dt = ω  
dv/dt = a  

## Control Architecture

Cascade structure:

- Guidance Layer  
  - Attractive potential toward goal  
  - Repulsive potential from obstacles  
  - Clearance weighting parameter (β)

- Control Layer  
  - PID for heading  
  - PID for velocity  

## Safety Mechanism

A clearance-based weighting factor modifies goal attraction when the rover is near obstacles.  
This allows tuning between shortest-path behavior and higher-clearance trajectories.

## Performance Metrics

- Path length
- Minimum obstacle clearance
- Velocity profile
- Heading response

## Known Limitations

- Artificial potential fields are local methods.
- Narrow symmetric passages may remain preferred paths.
- No global path planner (A*, RRT) implemented.
- No uncertainty or disturbance modeling.

---

# 2. Spacecraft Relative Guidance (In Progress)

This module extends the project toward dynamic target tracking in free space.

Planned features:

- 2D double integrator dynamics
- Moving target trajectory
- Relative motion formulation
- PD-based acceleration control
- Thrust saturation handling

The objective is to transition from nonholonomic rover control to fully actuated translational guidance systems.

---
## Installation

Install dependencies:

pip install -r requirements.txt


# 4. Purpose of This Project

This repository serves as a structured study of nonlinear navigation and feedback-based guidance systems.

It explores:

- Vector field navigation
- Cascade control
- Safety-performance tradeoffs
- Relative motion stabilization

The goal is to progressively move from ground robotics-style navigation to spacecraft-style guidance dynamics.

---

# 5. Future Work

- Orbital relative motion (Clohessy–Wiltshire equations)
- Fuel-aware optimal control
- Global path planning comparison
- Disturbance and noise modeling
