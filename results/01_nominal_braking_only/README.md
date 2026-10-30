# Scenario 1 — Nominal Control with Braking Disturbance Only

## Objective

Document the baseline experiment in which the platoon is subjected to the braking perturbation while operating with the nominal vehicle-following controller and without the CBF safety filter. This scenario provides the reference for comparison with Scenario 2, where the CBF filter is enabled.

## Intended configuration

- Nominal vehicle-following controller enabled.
- CBF safety filter disabled.
- Braking perturbation enabled.
- Sinusoidal leader disturbance disabled.
- Initial conditions, model parameters, simulation horizon, and solver settings matched to Scenario 2.

**Implementation status:** the current MATLAB script does not expose independent top-level switches for the CBF filter and the disturbance terms. The CBF-related QP is part of the follower-control calculation, while disturbance conditions are embedded in `platoon_dyn`. Confirm the exact settings used to generate the exported PDFs before describing the run as a controlled nominal-only experiment.

## Exported figures in this folder

### 1. Distance-related trajectories

File: `distance_macro_nominal.pdf`

Use this plot to inspect the evolution of the inter-vehicle spacing over the simulation horizon and the platoon's response to the braking perturbation. Check the plotted sign convention and axes before interpreting the values; in particular, distinguish the plotted distance variable from the physical gap and from the collision criterion used by the model.

### 2. Leader speed and relative-speed trajectories

File: `velocities_macro_nominal.pdf`

This figure reports the velocity response of the platoon during the braking scenario. Inspect the panels, axes, and legend to identify the leader-speed trajectory and the relative-speed signals between consecutive vehicles, and to follow how the braking perturbation propagates through the platoon.

### 3. CBF safety-function trajectories

File: `h_macro_nominal.pdf`

This plot shows the evolution of the (H) safety-function values calculated for the platoon. Since the CBF safety filter is intended to be disabled in this scenario, the figure should be treated as a diagnostic of the safety-function evolution, not as evidence that a CBF constraint is actively enforced. Interpret the sign and values only after checking the implemented function definition and safe-set convention.

## Recommended additional outputs

For a fuller audit of the controller's behavior, also export:

- Nominal control signals and requested acceleration profiles.
- A run-configuration record containing parameters, initial conditions, braking-disturbance timing and affected vehicle, solver tolerances, and the script revision.

These additional outputs are not currently committed in this scenario folder.

## Comparison with Scenario 2

To isolate the effect of enabling the CBF safety filter, Scenario 1 and Scenario 2 should share the same initial conditions, braking perturbation, model parameters, solver settings, and simulation horizon. CBF activation should be the only intended difference. Compare the distance, velocity, and (H) trajectories while accounting for the different role of the safety filter in the two configurations.

## Validation status

The three PDFs are committed exports, but their trajectories and interpretation have not been independently validated by this README update. Re-run the configured MATLAB experiment and compare the resulting figures with these files before using them as quantitative evidence.
