# Nonlinear Model Predictive Control — Pendulum

A from-scratch **nonlinear Model Predictive Control (NMPC)** implementation in MATLAB, demonstrated on a simple pendulum system. Written to expose the full optimization loop — no black-box MPC toolbox.

## Overview

At each time step the controller solves a constrained nonlinear program over a finite horizon, applies the first control move, then re-solves at the next step (receding horizon). This repo implements that loop directly with `fmincon`, making every piece — objective, constraints, and rollout — inspectable.

## What's inside

| File | Purpose |
|------|---------|
| `main.m` | Driver — sets up the simulation, runs the receding-horizon loop, plots results |
| `obj_fun.m` | Objective (cost) function minimized over the control horizon |
| `nlcon.m` | Nonlinear constraints passed to the solver |
| `u0.mat` | Initial control guess (warm start) |

## Key settings

- Control & prediction horizon: `Nc = Np = 8`
- Sampling time: `Ts = 0.05 s`, final time `2 s`
- Solver: `fmincon` (interior-point), with tolerances on the decision vector and objective

## Requirements

MATLAB with the **Optimization Toolbox**.

## Run it

```matlab
main    % runs the NMPC loop and plots state & control trajectories
```

## Background

This implementation supports my broader research on **dynamic programming and model predictive control** — see the companion repo **dp-control-codesign**.

## Author

**Oluwaseun A. Adekoya** — Robotics Engineer & PhD Candidate, University of Cincinnati. License: MIT.
