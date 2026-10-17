# Safety-Critical Control for Vehicle Platooning

MATLAB-based research project on safety-critical control for vehicle platooning, combining Control Barrier Functions (CBFs) with nominal vehicle-following control and string-stability-oriented dynamics.

## Project Overview

The project investigates how safety constraints can be incorporated into the control of a vehicle platoon while preserving the behavior of a nominal controller.

The repository contains MATLAB simulation scripts exploring different CBF formulations and model variants.

## Methods and Tools

- MATLAB numerical simulation using `ode45`
- Nominal vehicle-following control
- Control Barrier Function constraints
- Quadratic Programming (QP) for control filtering using `quadprog`
- Vehicle platoon dynamics and inter-vehicle spacing analysis
- Simulation-based evaluation of control inputs and safety functions

## Repository Status

The code is being organized and reviewed for reproducibility. Different experimental variants are retained for comparison; their assumptions and results must be checked before they are presented as validated implementations.

## Requirements

- MATLAB
- Optimization Toolbox, for `quadprog`

## Scope and Limitations

This is an academic research project based on numerical simulations. Simulation results alone do not establish real-world vehicle safety or a formal guarantee of string stability.

## Authors and References

Research context: Master's thesis project on safety-critical control for vehicle platooning.

Relevant publications and acknowledgements will be added after confirming the appropriate citation and publication details.

## License

No reuse license has been specified yet. Please check the applicable academic and publication rights before redistributing the code.
