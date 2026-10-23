# Scenario 1 — Nominal Control with Braking Disturbance Only

## Objective

Define the baseline case for assessing the response of the nominal vehicle-following controller to a braking perturbation, with the CBF safety filter disabled. This baseline is intended to be compared with Scenario 2, where the same experiment is run with CBF filtering enabled.

## Intended configuration

- Nominal vehicle-following control enabled.
- Braking disturbance enabled according to the selected experimental definition.
- CBF safety filter disabled.
- Initial conditions, model parameters, simulation horizon, and solver settings matched to Scenario 2.
- Sinusoidal leader disturbance disabled, so it does not confound the braking-only comparison.

**Implementation status:** the current MATLAB script does not expose switches for this configuration. The CBF-related QP calculation is part of the current follower-control path, and disturbance conditions are embedded in `platoon_dyn`. Configure and validate the script before labelling a generated figure as Scenario 1.

## Figures

No exported figure files are currently stored in this directory.

Recommended exports:

- **Inter-vehicle distances:** trajectories for each adjacent vehicle pair, with axes and sign convention clearly stated.
- **Speed response:** leader speed and follower-to-preceding-vehicle speed differences.
- **CBF safety function (diagnostic):** optionally evaluate and plot the same safety function offline. Since the filter is intended to be disabled in this scenario, this plot must not be described as an active constraint enforcing safety.

## Interpretation checklist

- Verify the disturbance start/end times and the affected vehicle or vehicles.
- Identify a collision only if the simulated distance satisfies the explicitly defined collision criterion.
- Do not infer collision avoidance from the nominal controller design alone.
- Keep the initial conditions and every other parameter identical to Scenario 2.

## Reproducibility record

When figures are added, record the script revision, parameter values, random seed, solver settings, and exact disturbance configuration used to generate them.
