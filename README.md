# ✈️ Aircraft Attitude Control System
### (NASA Generic Transport Model Inspired)

A Python-based aircraft attitude control simulator implementing a **Linear Quadratic Regulator (LQR)** autopilot for a transport aircraft inspired by the **NASA Generic Transport Model (GTM)**.

This project was developed as a learning project to understand modern aircraft flight control systems, state-space modeling, and optimal control theory.

---

# Features

- State-Space Aircraft Model (6 States)
- Continuous-Time LQR Controller
- Reference Attitude Tracking
- RK4 Numerical Integration
- First-Order Actuator Dynamics
- Control Surface Saturation
- Wind Disturbance Model
- Sensor Noise Model
- Flight Data Logging
- Automatic Performance Analysis
- CSV Export
- Data Visualization using Matplotlib

---

# Project Structure

```text
Aircraft-Attitude-Control/

│
├── config.py
├── aircraft.py
├── controller.py
├── simulator.py
├── visualization.py
├── main.py
│
├── requirements.txt
├── README.md
│
└── results/
```

---

# Aircraft Model

The aircraft is modeled using a simplified linear state-space representation inspired by the NASA Generic Transport Model (GTM).

State vector

```
x =

[
Roll Angle
Roll Rate

Pitch Angle
Pitch Rate

Yaw Angle
Yaw Rate
]
```

Control input

```
u =

[
Aileron
Elevator
Rudder
]
```

Continuous-time dynamics

```
x_dot = A x + B u
```

---

# Controller

The autopilot uses a Continuous-Time Linear Quadratic Regulator (LQR).

Control law

```
u = -K(x - x_ref)
```

where

- K : Optimal feedback gain
- x : Current aircraft state
- x_ref : Desired attitude

The optimal gain is obtained by solving the Continuous Algebraic Riccati Equation (CARE).

---

# Numerical Integration

The simulator uses a fourth-order Runge-Kutta (RK4) integrator.

Compared to Euler integration,

- Higher numerical accuracy
- Better stability
- Reduced accumulated error

---

# Disturbance Models

The simulator includes simplified environmental disturbances.

### Wind

- Gaussian wind disturbance

### Sensor Noise

- Gaussian measurement noise

These models can later be replaced with a Dryden Turbulence Model.

---

# Control Surface Limits

The following actuator limits are implemented.

| Surface | Limit |
|----------|-------|
| Aileron | ±25° |
| Elevator | ±25° |
| Rudder | ±30° |

---

# Output

The simulator automatically stores

- Time
- Aircraft State
- Control Input
- Reference State

and can export

```
results/simulation.csv
```

---

# Visualization

The following figures are generated.

- Roll Angle
- Pitch Angle
- Yaw Angle

- Roll Rate
- Pitch Rate
- Yaw Rate

- Aileron Deflection
- Elevator Deflection
- Rudder Deflection

- Tracking Error

---

# Performance Metrics

The simulator automatically evaluates

- RMSE
- Peak Time
- Overshoot
- Rise Time
- Settling Time

---

# Installation

Clone the repository

```bash
git clone https://github.com/yourusername/Aircraft-Attitude-Control.git
```

Install dependencies

```bash
pip install -r requirements.txt
```

---

# Run

```bash
python main.py
```

---

# Dependencies

- Python 3.12
- NumPy
- SciPy
- Matplotlib
- Pandas

Install

```bash
pip install numpy scipy matplotlib pandas
```

---

# Future Improvements

- Kalman Filter (LQE)
- Full 12-State Aircraft Model
- Gain Scheduling
- Dryden Turbulence Model
- MPC Controller
- PID Comparison
- Flight Mode Switching
- 3D Aircraft Visualization
- FlightGear Integration

---

# References

1. Stevens, B. L., Lewis, F. L., & Johnson, E. N.
   *Aircraft Control and Simulation*

2. Nelson, R. C.
   *Flight Stability and Automatic Control*

3. NASA Generic Transport Model (GTM)

4. Franklin, Powell & Emami-Naeini
   *Feedback Control of Dynamic Systems*

5. Ogata, K.
   *Modern Control Engineering*

---

## License

This project is licensed under the MIT License.

See the LICENSE file for details.

---

# Author

**Taewoo** + **ChatGPT**

Aircraft Attitude Control System

Python • Control Theory • Flight Dynamics • LQR
