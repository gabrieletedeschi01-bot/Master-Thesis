# Scenario 2 — CBF Enabled with Braking Disturbance Only

## Objective

Document the experiment in which the platoon is subjected to the braking perturbation while the CBF safety filter is enabled. This scenario extends the nominal baseline in Scenario 1 and is intended to show the effect of activating the safety filter under otherwise equivalent conditions.

## Intended configuration

- Nominal vehicle-following controller enabled.
- CBF safety filter enabled.
- Braking perturbation enabled, with the same timing and affected vehicle or vehicles as Scenario 1.
- Sinusoidal leader disturbance disabled.
- Initial conditions, model parameters, simulation horizon, and solver settings matched to Scenario 1.

**Implementation status:** the current MATLAB script does not expose independent top-level switches for the CBF filter and the disturbance terms. The CBF-related QP is part of the follower-control calculation, while disturbance conditions are embedded in `platoon_dyn`. Confirm the exact settings used to generate the exported PDFs before describing the run as a controlled comparison with Scenario 1.

## Exported figures in this folder

### 1. Distance-related trajectories

File: `distance_macro.pdf`

Use this plot to inspect the evolution of the inter-vehicle spacing over the simulation horizon and the platoon's response to the braking perturbation with the CBF filter active. Check the plotted sign convention and axes before interpreting the values; compare the same quantities in Scenario 1 to assess the effect of filter activation.

### 2. Leader speed and relative-speed trajectories

File: `velocities_macro.pdf`

This figure reports the velocity response of the platoon during the braking scenario. Inspect the panels, axes, and legend to identify the leader-speed trajectory and the relative-speed signals between consecutive vehicles, and compare their evolution with the nominal-control case in Scenario 1.

### 3. CBF safety-function trajectories

File: `h_macro.pdf`

This plot shows the evolution of the (H) safety-function values calculated for the platoon while the CBF filter is enabled. Use it to inspect the safety-function trajectories over time. Interpret the sign and values only after checking the implemented function definition and safe-set convention: a plot of (H) alone is not a formal proof of safety or of constraint satisfaction.

## Recommended additional outputs

For a fuller audit of the controller's behavior, also export:

- Nominal, filtered, and requested control signals.
- CBF intervention magnitude, calculated as the difference between nominal and filtered control.
- A run-configuration record containing parameters, initial conditions, braking-disturbance timing and affected vehicle, solver tolerances, and the script revision.

These additional outputs are not currently committed in this scenario folder.

## Comparison with Scenario 1

To attribute differences to CBF activation, Scenario 1 and Scenario 2 must share the same initial conditions, braking perturbation, model parameters, solver settings, and simulation horizon. CBF activation should be the only changed factor. Compare the distance, velocity, and (H) trajectories, and inspect the control signals to identify when and how the filter intervenes.

Do not infer collision avoidance or guaranteed safety from the presence of the CBF filter alone. Those conclusions require checking the trajectories, the implemented safety-function definition, the safe-set convention, and the simulation configuration.

## Validation status

The three PDFs are committed exports, but their trajectories and interpretation have not been independently validated by this README update. Re-run the configured MATLAB experiment and compare the resulting figures with these files before using them as quantitative evidence.
