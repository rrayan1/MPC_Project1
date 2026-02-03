# LMPC Racing Notebook Project

## Overview

**LMPC_Racing_Notebook_project1.ipynb** is a Jupyter notebook implementing **Learning Model Predictive Control (LMPC)** for autonomous racing applications. This project demonstrates how an autonomous racing vehicle can progressively improve lap performance through iterative learning by building a safe set of trajectories and learning a value function from historical data.

The notebook was developed by **Ugo Rosolia**, **Charlott Vallon**, **Francesco Borrelli** (UC Berkeley), and **Luigi Glielmo** (Università di Napoli Federico II).

---

## Key Features

### 1. **Track Mapping System**
- **Curvilinear Coordinate Framework**: Converts between global (X, Y) coordinates and track-relative (s, e_y) coordinates
- **Multiple Track Shapes**: 
  - L-shaped track (default)
  - Goggle-shaped track
  - Customizable track geometries
- **Robust Coordinate Transformation**: Handles position conversion, curvature calculation, and angle computation

### 2. **Vehicle Dynamics Simulator**
- **Dynamic Bicycle Model**: Represents realistic vehicle physics with:
  - Longitudinal and lateral tire forces
  - Steering input (delta) and acceleration (a)
  - Yaw rate and heading angle dynamics
  - Vehicle parameters (mass, inertia, tire coefficients)
- **Curvilinear Frame Dynamics**: State variables in the track-relative reference frame

### 3. **LMPC Controller with Local Linear Regression**
- **Iterative Learning**: Progressively improves performance over multiple laps
- **Local Linearization**: Estimates time-varying linear models (A_k, B_k, C_k) from historical data
- **Safe Set Construction**: Builds a set of previously feasible trajectories used as terminal constraints
- **Value Function Learning**: Learns a cost-to-go approximation for terminal state constraints
- **Predictive Model**: Uses regression on historical trajectories to predict system behavior

### 4. **Trajectory Visualization**
- **Closed-loop Trajectory Plotting**: Visualizes vehicle path evolution across laps
- **Predicted Trajectory Display**: Shows MPC predictions at each control step
- **Safe Set Visualization**: Displays learned feasible regions
- **Animation**: Creates video animations (MP4) of the racing performance evolution

---

## Project Structure

### Main Components

#### **Map Class**
```python
class Map():
```
- Defines track geometry and provides coordinate transformations
- Methods:
  - `getGlobalPosition(s, ey)`: Converts curvilinear to global coordinates
  - `getLocalPosition(x, y, psi)`: Converts global to curvilinear coordinates
  - `curvature(s)`: Returns track curvature at longitudinal position
  - `getAngle(s, epsi)`: Computes vehicle heading angle

#### **SIMULATOR Class**
```python
class SIMULATOR(object):
```
- Simulates vehicle dynamics using a dynamic bicycle model
- Methods:
  - `sim(x, u)`: Simulates one control step
  - `dyn_bicycle_model(x_states_list, u)`: Core dynamics with Pacejka tire model

#### **LMPC Controller**
- Solves finite-horizon optimal control problems at each time step
- Maintains:
  - Safe Set (SS): Collection of previously feasible trajectories
  - Value Function (V-function): Cost-to-go estimates
  - Historical trajectories for regression

#### **PredictiveModel Class**
- Learns linearized system models from data
- Performs local regression on historical trajectories
- Estimates parameters for predictive MPC formulations

---

## Workflow

### 1. **Initialization**
```
Initialize MPC parameters:
- Horizon length (N = 14)
- State dimension (n = 6)
- Input dimension (d = 2)
```

### 2. **PID Initialization**
- Generate initial sub-optimal trajectory using a PID controller
- This serves as the baseline for LMPC learning

### 3. **LMPC Iterations**
For each lap iteration:
```
For lap = 1 to Laps:
    - At each time step k:
        a) Estimate local linear models using historical data
        b) Construct time-varying Safe Set and V-function
        c) Solve LMPC optimal control problem
        d) Apply input to vehicle simulator
    - Store trajectory and update historical data
    - Append trajectory to Safe Set
    - Compute lap time and improvement metric
```

### 4. **Performance Tracking**
- Monitor lap time reduction across iterations
- Track cost function evolution
- Visualize convergence patterns

---

## State and Input Definitions

### Vehicle State (6D)
```
x = [vx, vy, wz, epsi, s, ey]
```
- **vx**: Longitudinal velocity (m/s)
- **vy**: Lateral velocity (m/s)
- **wz**: Yaw rate (rad/s)
- **epsi**: Heading angle error (rad)
- **s**: Longitudinal track position (m)
- **ey**: Lateral track position (m)

### Control Input (2D)
```
u = [delta, a]
```
- **delta**: Steering angle (rad)
- **a**: Acceleration (m/s²)

### Global State (6D)
```
x_glob = [vx, vy, wz, psi, X, Y]
```
- **psi**: Global heading angle (rad)
- **X, Y**: Global Cartesian coordinates (m)

---

## Vehicle Parameters

| Parameter | Value | Unit | Description |
|-----------|-------|------|-------------|
| m | 1.98 | kg | Vehicle mass |
| lf | 0.125 | m | Distance from CG to front axle |
| lr | 0.125 | m | Distance from CG to rear axle |
| Iz | 0.024 | kg·m² | Yaw inertia |
| Cf | 1.25 | - | Front tire cornering stiffness |
| Cr | 1.25 | - | Rear tire cornering stiffness |
| Df, Dr | 0.8·m·g/2 | N | Tire force coefficients |

---

## Key Algorithms

### 1. **Local Linear Regression**
- Uses N-step historical data windows
- Estimates time-varying linear models at each prediction horizon point
- Captures nonlinear dynamics through local linearization

### 2. **Safe Set Construction**
- Selects subset of historical trajectories
- Forms convex combinations as feasible terminal states
- Ensures recursive feasibility

### 3. **Value Function Approximation**
- Learns quadratic cost-to-go function
- Terminal cost: Q·||x_terminal||²
- Improves convergence and performance bounds

### 4. **Optimization Problem (LMPC)**
```
minimize: sum(||x_k||_Q² + ||u_k||_R²) + ||x_N||_P²
subject to:
  - x_{k+1} = A_k·x_k + B_k·u_k + C_k
  - |delta| <= delta_max
  - |ey| <= track_width
  - x_N in Safe Set
```

---

## Visualization Outputs

### 1. **Lap Trajectory Plot**
- Shows closed-loop vehicle path for each lap
- Displays track boundaries and centerline
- Highlights performance improvement across iterations

### 2. **Predicted Trajectory**
- MPC-predicted vehicle path (red dashed line)
- Current vehicle position with orientation
- Local safe set points (green dots)

### 3. **Animation (MP4)**
- Real-time visualization of LMPC learning progression
- Shows predicted trajectories at each control step
- Displays convergence behavior
- Saved as `lmpc_learning.mp4`

---

## Usage

### Running the Notebook

1. **Install Dependencies**
   ```bash
   pip install numpy matplotlib scipy cvxopt osqp dataclasses
   ```

2. **Execute Cells Sequentially**
   - Cell 1: Import dependencies
   - Cell 2: Define Map class (track geometry)
   - Cell 3: Define SIMULATOR class (vehicle dynamics)
   - Cell 4+: Initialize MPC parameters
   - Cell N: Run LMPC iterations
   - Cell N+1: Plot trajectories and animations

3. **Customize Parameters**
   - Modify `N` (horizon length) for different control horizons
   - Adjust track geometry in Map class
   - Change `Laps` for more learning iterations
   - Tune `numSS_Points` for Safe Set density

---

## Performance Metrics

### Tracked Metrics
- **Lap Time**: Total time to complete one lap (seconds)
- **Convergence**: Lap time reduction per iteration
- **Feasibility**: Constraint satisfaction rate
- **Optimality**: Gap to predicted optimal trajectory

### Expected Behavior
- Lap 1-2: High lap times (PID baseline)
- Lap 3-5: Significant improvements
- Lap 5+: Convergence to near-optimal performance

---

## File Outputs

| File | Description |
|------|-------------|
| `lmpc_learning.mp4` | Animation of LMPC learning progression |
| Trajectory data | Stored in `lmpc.SS_glob[]` and `lmpc.Qfun[]` |
| Terminal costs | Stored in terminal value function estimates |

---

## Advanced Features

### 1. **Time-Varying MPC**
- `lmpcParameters.timeVarying = True`
- Enables local regression for each prediction step
- Improves accuracy of learned models

### 2. **Soft Constraints**
- Terminal slack variables for constraint relaxation
- Configurable via `QterminalSlack`
- Ensures feasibility in early iterations

### 3. **Custom Track Design**
- Modify `spec` array in Map class
- Define segments as `[arc_length, signed_curvature]`
- Support straight lines (curvature = 0) and curves

### 4. **Model Adaptation**
- `PredictiveModel` class learns from closed-loop data
- Updates regression parameters each lap
- Improves predictions as vehicle learns track

---

## Theoretical Background

### LMPC Theory
- Based on Learning Model Predictive Control framework
- Guarantees feasibility and optimality improvement
- Uses finite-time optimal control with terminal constraints
- Convergence proof: Performance improves or stays same each iteration

### Curvilinear Coordinates
- Simplifies track constraints (box constraints instead of circular)
- Reduces computational complexity
- Standard approach in autonomous racing

### Bicycle Model
- Simplified vehicle dynamics with nonlinear tire forces
- Pacejka tire model for realistic lateral forces
- Captures essential racing dynamics

---

## Limitations & Future Work

### Current Limitations
- Simplified vehicle model (single track model)
- No explicit obstacle avoidance
- Assumes constant track properties
- Discretization time: 0.001s (might be too fine)

### Future Enhancements
- Multi-vehicle competition
- Adaptive learning rates
- Robust MPC with uncertainty quantification
- Real-time implementation on embedded systems
- Integration with high-fidelity simulators

---

## Copyright & Attribution

These materials are protected by U.S. copyright law and University policy. They were developed by:
- **Ugo Rosolia** (UC Berkeley)
- **Charlott Vallon** (UC Berkeley)
- **Francesco Borrelli** (UC Berkeley)
- **Luigi Glielmo** (Università di Napoli Federico II)

Unauthorized reproduction or distribution is prohibited without express written consent.

Reference: https://copyright.universityofcalifornia.edu/resources/ownership-course-materials.html

---

## References

1. Rosolia, U., Borrelli, F. (2021). "Learning Model Predictive Control for Iterative Tasks: A Computationally Efficient Approach"
2. Pacejka, H.B. (2006). "Tire and Vehicle Dynamics" - Tire model references
3. Hindiyeh, R.Y. (2011). "Architecture, Perception, and Control for Autonomous Vehicles"

---

## Support & Questions

For technical questions or improvements, refer to the notebook comments and documentation strings in each class.

Last Updated: February 2026
#   M P C _ P r o j e c t 1  
 