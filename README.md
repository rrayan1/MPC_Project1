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

