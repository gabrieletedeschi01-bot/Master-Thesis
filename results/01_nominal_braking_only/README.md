# Scenario 1 — Nominal Control with Braking Disturbance Only

## Objective

This scenario shows the platoon response to the braking disturbance with the nominal vehicle-following controller and without the CBF safety filter. It provides the baseline for comparison with Scenario 2, where the CBF filter is active.

## Figures

The three PDF files in this directory show the time evolution of the main platoon variables for this nominal-control run.

### 1. Inter-vehicle distances — `distance_macro_nominal.pdf`

Shows the evolution over time of the inter-vehicle distances in the platoon. The curves make it possible to inspect how the spacing between consecutive vehicles changes after the braking disturbance and whether any spacing approaches the safety limit used in the model.

### 2. Vehicle velocities — `velocities_macro_nominal.pdf`

Shows the velocity trajectories of the platoon vehicles over the simulation horizon. This plot illustrates how the braking disturbance propagates through the platoon and how the vehicles respond under nominal control.

### 3. Safety function (H) — `h_macro_nominal.pdf`

Shows the time evolution of the (H) safety-function values associated with the platoon. In this scenario the CBF safety filter is not intended to intervene, so the plot is a diagnostic of the safety-function evolution, not evidence that a CBF constraint is actively enforced. Interpret the values using the definition and sign convention implemented in the model.

## How to use this scenario

Compare these plots with the corresponding figures in [Scenario 2 — CBF enabled with braking disturbance](../02_cbf_braking_only/README.md). The intended difference between the scenarios is whether the CBF safety filter is active; initial conditions, braking profile, model parameters, and solver settings should otherwise match.

## Reproducibility note

The current MATLAB script has disturbance conditions embedded in `platoon_dyn` and does not expose top-level switches for independently selecting each scenario. To reproduce this figure set, verify the exact configuration used for the exported PDFs before treating it as a directly reproducible run. Record the script revision, parameter values, disturbance timing and affected vehicle, random seed, and solver settings.
