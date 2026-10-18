# Safety-Critical Control for Vehicle Platooning

MATLAB-based academic research project investigating safety-critical control for vehicle platooning by combining Control Barrier Functions (CBFs), nominal vehicle-following control, and string-stability-oriented dynamics.

## Repository Structure

- `src/Safety_Critical_Vehicle_Platooning.m` — MATLAB simulation script.
- `results/` — exported simulation figures and result summaries (to be added after verification).
- `docs/` — technical report and supporting documentation (in preparation).

## Methods and Tools

- MATLAB numerical simulation with `ode45`
- Nominal vehicle-following control
- Control Barrier Function safety constraint
- Quadratic Programming (QP) control filtering with `quadprog`
- Vehicle-platoon dynamics and inter-vehicle spacing analysis
- Simulation-based analysis of control inputs and safety functions

## Requirements

- MATLAB
- Optimization Toolbox, for `quadprog`

## Running the Simulation

Open `src/Safety_Critical_Vehicle_Platooning.m` in MATLAB and run it. The script is designed to generate simulation plots. Check the script's assumptions, numerical behavior, and output figures before interpreting or presenting the results.

## Scope and Limitations

This is an academic numerical-simulation project. The code and results are still subject to review; simulation results alone do not establish real-world vehicle safety or a formal guarantee of string stability.

## Publication and Conference Presentation

**Title:** *Safety via Control Barrier Functions in a String Stable platoon: a mesoscopic approach*

**Conference:** IEEE International Conference on Intelligent Transportation Systems (ITSC 2026)

**Authors:** Gabriele Tedeschi, Mattia Mattioni, Alessio Iovine

**Status:** Presented at IEEE ITSC 2026. Publication link and DOI will be added when confirmed.

## License

No reuse license has been specified. Please check applicable academic and publication rights before redistributing the code.
