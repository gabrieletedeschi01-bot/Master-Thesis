# Scenario 3 — CBF Enabled with Braking and Sinusoidal Leader Disturbance

## Objective

Document the intended experiment that extends Scenario 2 by adding a sinusoidal disturbance to the leader while retaining the braking perturbation and CBF safety filter. The comparison with Scenario 2 is intended to isolate the additional effect of the leader disturbance.

## Intended configuration

- Nominal vehicle-following controller enabled.
- CBF safety filter enabled.
- Braking perturbation retained from Scenario 2.
- Sinusoidal leader disturbance enabled.
- Initial conditions, model parameters, simulation horizon, and solver settings matched to Scenario 2 except for the added leader disturbance.

**Implementation status:** the current MATLAB script contains a sinusoidal term in the leader acceleration for (5 < t < 30) s and an additional (-10) acceleration term for vehicle index 10 during (10 < t < 15) s. The CBF-related QP is also used in the follower-control calculation. These conditions are embedded in the code rather than controlled by independent scenario flags. Confirm the exact settings used for the existing PDFs before describing them as a controlled comparison.

## Exported figures in this folder

### 1. Distance-related trajectories

File: `distance_macro_sin.pdf`

Use this plot to inspect the spacing trajectories over the simulation horizon. Check the plotted sign convention: the script's distance plot uses the negative of the computed inter-vehicle distance. Do not infer a collision from the figure title or annotation alone; use the model's explicit collision criterion and the underlying data.

### 2. Leader speed and relative-speed trajectories

File: `velocities_macro_sin.pdf`

The MATLAB plotting section creates a two-panel figure with leader speed in the upper panel and relative speeds between consecutive vehicles in the lower panel. Inspect the axes and legend in the exported PDF to confirm the displayed quantities and indexing.

### 3. CBF safety-function trajectories

File: `h_macro_sin.pdf`

This plot shows the safety-function values calculated in the script. Interpret their sign only after confirming the exact function definition, sign convention, and safe-set convention. A plot of (h) alone is not a formal proof of safety.

## Recommended additional outputs

For a fuller audit of the controller's behavior, also export:

- Nominal, filtered, and requested control signals.
- CBF intervention magnitude, calculated as the difference between nominal and filtered control.
- A run-configuration record containing parameters, initial conditions, disturbance timings, solver tolerances, and the script revision.

These additional outputs are not currently committed in this scenario folder.

## Comparison with Scenario 2

To attribute differences to the sinusoidal leader disturbance, Scenario 2 and Scenario 3 must share the same initial conditions, braking perturbation, CBF configuration, model parameters, solver settings, and simulation horizon. The leader sinusoid should be the only changed factor. The current script's additional perturbation on vehicle index 10 must be accounted for in both cases or explicitly identified as a further difference.

## Validation status

The three PDFs are committed exports, but their trajectories and interpretation have not been independently validated by this README update. Re-run the configured MATLAB experiment and compare the resulting figures with these files before using them as quantitative evidence.
