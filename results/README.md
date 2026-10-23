# Simulation Results

This directory organizes exported plots and documentation by experimental scenario. The scenario labels describe the intended comparison. They should not be interpreted as evidence that all three cases can already be selected independently from the current MATLAB script.

## Scenario index

### Scenario 1 — Nominal control with braking only

Path: `01_nominal_braking_only/`

Intended baseline with the nominal controller, braking disturbance, and CBF filter disabled. Use this case to examine the baseline spacing response. The directory currently contains a README only; no exported plots are committed.

### Scenario 2 — CBF enabled with braking only

Path: `02_cbf_braking_only/`

Intended comparison with the same initial conditions and braking disturbance as Scenario 1, but with the CBF safety filter enabled. The directory currently contains a README only; no exported plots are committed.

### Scenario 3 — CBF enabled with braking and sinusoidal leader disturbance

Path: `03_cbf_braking_and_leader_sine/`

This directory contains the three exported PDF figures listed below. The current MATLAB script includes both a sinusoidal term in the leader acceleration and an additional time-limited perturbation on vehicle index 10. Confirm the exact configuration used to produce the PDFs before treating them as a controlled comparison with Scenarios 1–2.

## Available figures

| File | Intended content |
| --- | --- |
| `03_cbf_braking_and_leader_sine/distance_macro_sin.pdf` | Distance-related trajectories for the exported run |
| `03_cbf_braking_and_leader_sine/velocities_macro_sin.pdf` | Leader speed and relative-speed trajectories for the exported run |
| `03_cbf_braking_and_leader_sine/h_macro_sin.pdf` | CBF safety-function trajectories for the exported run |

The names above reflect the files stored in the repository. Inspect their axes, legends, and sign conventions before making claims about a collision or the safety function.

## Recommended figures for each scenario

For a consistent comparison, export and retain the following for each configured scenario:

- Inter-vehicle distances versus time.
- Leader speed and relative-speed trajectories.
- CBF safety-function values, where the function can be evaluated consistently. In the nominal-only baseline, this is a diagnostic and not an active controller constraint.
- Optionally, nominal versus filtered control inputs and the magnitude of CBF intervention for cases with the filter enabled.

Use a consistent naming convention and record the script revision, parameter values, disturbance profile, and scenario flags used for every export.

## Fair comparison protocol

1. Use the same initial conditions and random seed in all scenarios.
2. Keep the simulation horizon, model parameters, solver tolerances, and disturbance profile fixed wherever the scenario definition requires it.
3. Between Scenarios 1 and 2, change only whether the CBF filter is active.
4. Between Scenarios 2 and 3, add only the sinusoidal leader disturbance.
5. Verify the actual disturbance logic and the vehicle index affected before running the comparison.
6. Check the distance sign convention and the exact collision criterion before describing a collision.
7. Check the mathematical definition of (h) before interpreting its sign; a positive or nonnegative value implies safety only if the implemented definition and safe-set convention support that conclusion.

## Current limitation

The current MATLAB script does not yet include top-level switches for independently selecting the nominal-only baseline, CBF activation, braking perturbation, and sinusoidal leader disturbance. Scenario 1 and Scenario 2 therefore remain documented target configurations, not confirmed independent outputs of the current script. Update and validate the simulation configuration before publishing quantitative comparisons.
