# Simulation Results

This directory is organized by simulation scenario. Each scenario folder contains a short README describing the experiment and the figures to export from MATLAB.

## Scenario folders

1. `01_nominal_braking_only/` — nominal controller with the braking disturbance only and CBF disabled; baseline for assessing whether the nominal controller maintains safe spacing.
2. `02_cbf_braking_only/` — same braking disturbance and initial conditions, with the Control Barrier Function (CBF) active.
3. `03_cbf_braking_and_leader_sine/` — Scenario 2 plus the sinusoidal disturbance on the leader, with the CBF active.

## Figures for each scenario

- `inter_vehicle_distances.png` — inter-vehicle distances over time.
- `vehicle_speeds.png` — vehicle speeds over time.
- `cbf_safety_function.png` — safety-function values for each vehicle.

For Scenario 1, the CBF safety function can be plotted as a diagnostic even though it is not active in the controller, provided it can be evaluated consistently from the simulated state.

## Fair comparison and validation

Keep initial conditions, simulation horizon, braking disturbance, and all model parameters identical between Scenarios 1 and 2; change only whether the CBF is active. Scenario 3 should differ from Scenario 2 only by the added sinusoidal leader disturbance.

These folders currently contain documentation/placeholders, not validated figure files. Export the plots from MATLAB after running and checking each simulation. State that a collision occurs only if the distance trajectories meet the model's collision criterion. A positive safety-function value indicates safety only if that follows from the exact definition and sign convention used in the model.
