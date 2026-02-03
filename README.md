# LMPC Racing Notebook Project

## Overview

**LMPC_Racing_Notebook_project1.ipynb** is a Jupyter notebook implementing **Learning Model Predictive Control (LMPC)** for autonomous racing applications. This project demonstrates how an autonomous racing vehicle can progressively improve lap performance through iterative learning by building a safe set of trajectories and learning a value function from historical data.


---

## Key Features

### 1. Track Mapping System
- **Curvilinear coordinate framework:** Converts between global \((X, Y)\) coordinates and track-relative \((s, e_y)\) coordinates.
- **Multiple track shapes:**
  - L-shaped track (default)
  - Goggle-shaped track
  - Customizable track geometries
- **Robust coordinate transformation:** Handles position conversion, curvature calculation, and angle computation.

### 2. Vehicle Dynamics Simulator
- **Dynamic bicycle model:** Represents realistic vehicle physics with:
  - Longitudinal and lateral tire forces
  - Steering input (`delta`) and acceleration (`a`)
  - Yaw rate and heading angle dynamics
  - Vehicle parameters (mass, inertia, tire coefficients)
- **Curvilinear frame dynamics:** State variables expressed in the track-relative reference frame.

### 3. LMPC Controller with Local Linear Regression
- **Iterative learning:** Progressively improves performance over multiple laps.
- **Local linearization:** Estimates time-varying linear models \((A_k, B_k, C_k)\) from historical data.
- **Safe set construction:** Builds a set of previously feasible trajectories used as terminal constraints.
- **Value function learning:** Learns a cost-to-go approximation for terminal state constraints.
- **Predictive model:** Uses regression on historical trajectories to predict system behavior.

### 4. Trajectory Visualization
- **Closed-loop trajectory plotting:** Visualizes vehicle path evolution across laps.
- **Predicted trajectory display:** Shows MPC predictions at each control step.
- **Safe set visualization:** Displays learned feasible regions.
- **Animation:** Creates video animations (MP4) of the racing performance evolution.

---

## Project Structure

### Main Components

#### Map Class
```python
class Map():

