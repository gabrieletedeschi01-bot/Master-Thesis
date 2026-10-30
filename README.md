# Safety-Critical Control for Vehicle Platooning

## Project overview

This repository contains the MATLAB implementation and selected simulation figures for an academic research project on safety-critical control of vehicle platoons. The project investigates how a nominal vehicle-following controller can be combined with a Control Barrier Function (CBF)-based safety filter, while the model also includes terms intended to represent platoon-level or mesoscopic spacing and velocity effects.

The work is associated with the paper **“Safety via Control Barrier Functions in a String Stable platoon: a mesoscopic approach”**, by Gabriele Tedeschi, Mattia Mattioni, and Alessio Iovine, presented at IEEE ITSC 2026.

> **Reproducibility note:** the repository contains a MATLAB simulation script and three PDF figures for Scenario 3. The scenario folders document the intended comparisons, but the current MATLAB script does not yet expose switches for selecting each scenario independently. Do not treat all three scenarios as independently reproducible from the current script without first configuring and validating the corresponding disturbance and CBF settings.

## Repository structure

| Path | Contents |
| --- | --- |
| `src/Safety_Critical_Vehicle_Platooning.m` | Main MATLAB script: parameters, initial conditions, controller and safety-filter calculations, platoon dynamics, numerical integration, diagnostics, and plots. |
| `src/README.md` | Detailed guide to the MATLAB script, parameters, outputs, and execution. |
| `results/README.md` | Index of the documented simulation scenarios and exported figures. |
| `results/01_nominal_braking_only/` | Documentation for the nominal-control baseline with braking disturbance and CBF intended to be disabled. No exported figures are currently stored here. |
| `results/02_cbf_braking_only/` | Documentation for the braking-only case with the CBF intended to be enabled. No exported figures are currently stored here. |
| `results/03_cbf_braking_and_leader_sine/` | Three exported PDF figures associated with the case involving braking and a sinusoidal leader disturbance, with CBF filtering. |
| `docs/Platooning_Safety_Critical_Control_Technical_Report.pdf` | Standalone technical project report describing the platoon model, mesoscopic information, nominal controller, CBF-QP safety layer, design rationale, and simulation campaign. |
| `docs/README.md` | Technical documentation outline and validation checklist. |

## Technical report

A standalone companion document is available here:

**[Platooning – Technical Project Report (PDF)](docs/Platooning_Safety_Critical_Control_Technical_Report.pdf)**

The report provides a concise technical overview of the modelling framework and control architecture, including the relative-state formulation, mesoscopic aggregate terms, nominal control law, macro-enhanced CBF, QP safety filter, and the three simulation configurations.

The PDF is an independently typeset project report inspired by the underlying research work. It is **not** the official conference paper or publisher's version, and it does not replace the full paper, MATLAB implementation, or exported simulation data. Its descriptions should be interpreted together with the implementation and the reproducibility limitations stated in this README.

## Main methods

- **Nominal vehicle-following control:** computes a baseline acceleration command for the leader and followers.
- **Control Barrier Function (CBF):** constructs a safety function using relative position, relative velocity, a headway-like term, and mesoscopic terms derived from spacing and velocity variations.
- **Quadratic-programming filter:** uses `quadprog` to find a control input close to the nominal command while enforcing the implemented inequality and actuator bounds.
- **Platoon dynamics:** simulates the positions, velocities, and an additional internal state for each vehicle.
- **Numerical integration:** uses MATLAB's `ode45` solver with configured tolerances and a maximum step.
- **Diagnostics and plots:** generates figures for inter-vehicle distances, leader speed and relative speeds, nominal/filtered/requested controls, CBF intervention magnitude, and the safety function.

The presence of a CBF calculation or a plotted safety function should not, by itself, be interpreted as proof of safety or string stability. Those conclusions depend on the implemented equations, sign conventions, solver behavior, and validated simulation evidence.

## Software requirements

- MATLAB with support for local functions in scripts.
- Optimization Toolbox, required for `quadprog`.

The script uses `rng(1)` to make the randomized initial-position offsets repeatable under the same MATLAB environment and code version.

## Running the code

1. Open `src/Safety_Critical_Vehicle_Platooning.m` in MATLAB.
2. Review the parameter block and the disturbance logic before running.
3. Confirm that Optimization Toolbox is available.
4. Run the script and inspect the command-window timing output and generated figures.
5. Export figures only after confirming that they correspond to the intended scenario and parameter set.

The script currently contains disturbance logic directly in the dynamics function and applies the CBF-filtered controller in the follower-control calculation. It does not yet provide top-level Boolean switches to enable or disable the CBF, braking disturbance, and leader sinusoidal disturbance independently. Therefore, the three scenario folders should be read as the intended experimental structure, not as proof that one run of the current script reproduces all three cases.

## Results currently committed

The folder `results/03_cbf_braking_and_leader_sine/` contains:

- `distance_macro_sin.pdf` — distance-related plot.
- `velocities_macro_sin.pdf` — leader speed and relative-speed plot.
- `h_macro_sin.pdf` — CBF safety-function plot.

The files are preserved as exported figures. Their existence does not independently verify the plotted trajectories, the occurrence of a collision, or formal safety/string-stability properties. The exact plotting conventions and the sign of the plotted distance should be checked against the MATLAB code before drawing conclusions.

## Scope and limitations

This is an academic numerical-simulation project, not a validated production controller. The repository does not claim real-world vehicle validation, formal proof of collision avoidance, or a formal guarantee of string stability. Comparisons between scenarios should use the same initial conditions, simulation horizon, model parameters, and disturbance profiles, changing only the feature under investigation.

## Publication

- **Title:** *Safety via Control Barrier Functions in a String Stable platoon: a mesoscopic approach*
- **Conference:** IEEE International Conference on Intelligent Transportation Systems (ITSC 2026)
- **Authors:** Gabriele Tedeschi, Mattia Mattioni, Alessio Iovine
- **Publication details:** add the final proceedings link and DOI when confirmed.

## License and reuse

No software reuse license is currently specified in this repository. Before redistributing or reusing the code or figures, check the applicable academic, co-author, and publication rights.
