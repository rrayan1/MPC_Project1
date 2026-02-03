# MPC_Project1 — LMPC Racing Notebook

This repository contains a Jupyter notebook implementing **Learning Model Predictive Control (LMPC)** for **autonomous racing**. The controller improves lap performance iteratively by learning from previously driven trajectories via a **Safe Set** and an approximation of the **cost-to-go** (value function).

> Notebook: `LMPC_Racing_Notebook_project1.ipynb`

---

## What’s inside

- **Learning MPC (LMPC)** loop over laps (iterative improvement)
- **Dynamic bicycle model** simulation
- **Curvilinear / track-relative coordinates**:
  - \( (X,Y,\psi) \leftrightarrow (s,e_y,e_\psi) \)
- **Local linear regression** to estimate time-varying models along the horizon
- **Safe Set** terminal constraint + learned terminal cost
- **Plots and animations** of closed-loop and predicted trajectories

---

## Repository structure

---

## State and input definitions

### Track-frame state (6D)
\[
x = [v_x,\ v_y,\ w_z,\ e_\psi,\ s,\ e_y]
\]

- `vx`: longitudinal velocity (m/s)  
- `vy`: lateral velocity (m/s)  
- `wz`: yaw rate (rad/s)  
- `epsi`: heading error (rad)  
- `s`: curvilinear abscissa along the track (m)  
- `ey`: lateral deviation from centerline (m)

### Control input (2D)
\[
u = [\delta,\ a]
\]

- `delta`: steering angle (rad)  
- `a`: acceleration (m/s²)

---

## LMPC workflow (high level)

1. **Initialize** controller parameters (e.g., horizon \(N\)) and constraints  
2. **Warm start** with a baseline controller (e.g., PID) to generate an initial feasible lap  
3. For each lap:
   - learn local linear models \(A_k, B_k, C_k\) from stored data (local regression)
   - construct/update the **Safe Set** and terminal cost estimate
   - solve the finite-horizon LMPC optimization
   - apply only the first input, simulate forward, repeat (receding horizon)
4. Store the new trajectory and update the learning data

---

## Installation

Create a virtual environment (recommended), then install dependencies:

```bash
pip install numpy scipy matplotlib osqp cvxopt



