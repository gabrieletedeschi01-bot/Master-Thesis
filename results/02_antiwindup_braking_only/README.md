# Scenario 2 — Anti-windup/safety function with braking disturbance only

## Purpose
Apply the same braking disturbance and initial conditions as Scenario 1, with the anti-windup/safety function enabled, so the two cases can be compared fairly.

## Figures to add
- `inter_vehicle_distances.png` — inter-vehicle distances over time.
- `vehicle_speeds.png` — vehicle speeds over time.
- `cbf_safety_function.png` — safety-function values for each vehicle.

## Comparison
Compare against `../01_nominal_braking_only/`. Keep the disturbance, initial conditions, simulation horizon, and other parameters identical; change only the controller feature being tested where possible.
