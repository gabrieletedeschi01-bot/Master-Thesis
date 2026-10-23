# Scenario 2 — CBF Enabled with Braking Disturbance Only

## Objective

Evaluate the intended effect of the CBF safety filter under the same braking-disturbance conditions used in Scenario 1. The comparison should isolate the effect of enabling the filter rather than changes to initial conditions or other model parameters.

## Intended configuration

- Nominal vehicle-following control enabled.
- The same braking disturbance, timing, and affected vehicle or vehicles as Scenario 1.
- CBF safety filter enabled.
- Sinusoidal leader disturbance disabled.
- Initial conditions, simulation horizon, model parameters, and solver settings matched to Scenario 1.

**Implementation status:** the current MATLAB script does not provide a top-level switch for disabling the CBF or independently enabling/disabling each disturbance. The scenario should be considered a target experiment until the script is configured and the resulting run is checked.

## Figures

No exported figure files are currently stored in this directory.

Recommended exports:

- **Inter-vehicle distances:** compare spacing response against Scenario 1.
- **Speed response:** leader speed and relative speeds across the platoon.
- **CBF safety function:** evaluate each relevant pair using the implemented definition and sign convention.
- **Control inputs (recommended):** show nominal and filtered acceleration commands and actuator bounds, so the filter's interventions can be inspected.

## Comparison with Scenario 1

Use the same initial state, random seed, solver settings, braking profile, and all model parameters. The only intended change is CBF activation. Report the actual behavior observed in the simulation; do not claim that the filter prevents a collision or enforces the safety condition without checking the trajectories, solver outcomes, and safety-function definition.

## Reproducibility record

When figures are added, record the script revision, parameter values, random seed, solver settings, and exact disturbance configuration used to generate them.
