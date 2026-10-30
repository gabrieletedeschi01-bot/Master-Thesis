# Scenario 2 — CBF Enabled with Braking Disturbance Only

## Objective

This scenario shows the platoon response to the braking disturbance with the CBF safety filter active. It is intended to be compared with Scenario 1, where the nominal controller is used without CBF filtering.

## Figures

The three PDF files in this directory show the time evolution of the main platoon variables for the CBF-enabled run.

### 1. Inter-vehicle distances — `distance_macro.pdf`

Shows the evolution over time of the inter-vehicle distances in the platoon. Use the curves to inspect the spacing response following the braking disturbance and compare it with the corresponding distances in Scenario 1.

### 2. Vehicle velocities — `velocities_macro.pdf`

Shows the velocity trajectories of the platoon vehicles over the simulation horizon. The curves illustrate the vehicles' response to braking and how the disturbance propagates through the platoon when the CBF safety filter is active.

### 3. Safety function (H) — `h_macro.pdf`

Shows the time evolution of the (H) safety-function values associated with the platoon. This plot is useful for checking the evolution of the safety metric while the CBF filter is active. Whether a value represents satisfaction of the safety condition depends on the exact definition and sign convention used in the implementation; the plot should be interpreted against that definition rather than from the label alone.

## Comparison with Scenario 1

Compare the distance, velocity, and (H) plots with those in [Scenario 1 — nominal control with braking disturbance](../01_nominal_braking_only/README.md). For a controlled comparison, use the same initial state, braking profile, affected vehicle, model parameters, simulation horizon, and solver settings; CBF activation should be the intended difference.

Do not infer collision avoidance or guaranteed safety from the presence of the CBF filter alone. Those conclusions require checking the plotted trajectories, the implemented safety-function definition, and the simulation configuration.

## Reproducibility note

The current MATLAB script does not provide a top-level switch for independently enabling/disabling the CBF and each disturbance. Verify the exact configuration used to generate these PDFs before treating the run as directly reproducible. Record the script revision, parameter values, disturbance timing and affected vehicle, random seed, and solver settings.
