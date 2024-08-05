# EKF-Based Distributed Relative Localization for Swarm Robotics

![Python](https://img.shields.io/badge/Python-3.x-3776AB?logo=python&logoColor=white)
![NumPy](https://img.shields.io/badge/NumPy-013243?logo=numpy&logoColor=white)
![Matplotlib](https://img.shields.io/badge/Matplotlib-11557C)

A simulation of **relative localization for a swarm of micro-drones** using an **Extended Kalman Filter**.
Each drone estimates the relative position and heading of its neighbours by fusing its own odometry
(velocity and yaw rate) with **UWB range measurements**. It does not need GPS or fixed UWB anchors.

---

## Approach

Each pair of robots *(i, j)* tracks the relative state **[x, y, ψ]** of *j* in *i*'s body frame.

| EKF step | Model |
| --- | --- |
| **Prediction** | Relative kinematics driven by the noisy velocity and yaw-rate inputs of both robots |
| **Measurement** | UWB range: `d = √(x² + y²)` |
| **Linearization** | Analytic Jacobians F (state), B (inputs) and H (measurement) |
| **Noise** | Tunable process covariance Q and measurement covariance R |

The simulator converts the relative estimates back to world coordinates so they can be compared with the
ground truth in real time.

## Scenarios

- **Random flight:** every robot changes velocity and yaw rate at random every 100 steps.
- **Formation flight:** after an initial random phase, a PID controller drives robot 0 to hold a fixed
  offset from robot 1, using the EKF estimate as feedback.

To switch scenarios, toggle `random_fly_inputs` / `formation_inputs` in `simulation.py`.

## Getting started

```bash
git clone https://github.com/YogiOnCode/EKF-Based-Distributed-Relative-Localization-for-Swarm-Robotics.git
cd EKF-Based-Distributed-Relative-Localization-for-Swarm-Robotics
pip install numpy pandas matplotlib
python simulation.py
```

The default setup simulates 10 robots in a 10 m × 10 m arena for 50 s, with σ = 0.05 m UWB noise.

## Results

- Estimated positions track the ground truth closely, with errors **consistently within about 2%** in both scenarios.
- A few runs show outliers with larger divergence, which points to room for better covariance tuning.

## Repository structure

```
├── ekf.py              # Extended Kalman Filter (prediction + UWB range update)
├── data_generator.py   # Motion model, noisy inputs/measurements, PID formation control
└── simulation.py       # Animated simulation and error plots
```

## Future work

- Extend the estimator to **3D** (altitude), for aerial and underwater swarms.
- Reduce variance in outlier cases with adaptive noise estimation.

## Tech stack

Python · NumPy · pandas · Matplotlib (animation)

## Author

**Yogeswaran Amsavalli** · [GitHub](https://github.com/YogiOnCode)
