# Scenario 1 — Nominal controller with braking disturbance only

## Purpose
Baseline case: apply the braking disturbance to the rear vehicles while the Control Barrier Function (CBF) is disabled, to assess whether the nominal controller maintains safe spacing.

## Figures to add
- `inter_vehicle_distances.png` — inter-vehicle distances over time.
- `vehicle_speeds.png` — vehicle speeds over time.
- `cbf_safety_function.png` — CBF safety-function values for each vehicle, evaluated offline as a diagnostic if available.

## Interpretation
Use the distance plot to check whether a collision occurs according to the model's collision criterion. Do not label this case as a collision unless the simulation output confirms it. The CBF function, if plotted, is diagnostic only and is not active in the controller in this scenario.
