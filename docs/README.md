# Technical Documentation

## Purpose

This directory is reserved for the technical explanation of the MATLAB implementation and the experiments for safety-critical control of vehicle platooning. The documentation should make the model, controller, safety filter, simulation assumptions, and interpretation of the figures traceable to the implementation in `src/Safety_Critical_Vehicle_Platooning.m`.

## Recommended document structure

1. **Problem statement and objectives**
   - Vehicle-platooning setup and safety motivation.
   - Role of nominal following control versus safety filtering.
   - Intended relationship between local safety and platoon-level/string-stability analysis.

2. **State and platoon model**
   - Numbering convention for the leader and followers.
   - State vector and the meaning of position, velocity, and the additional internal state.
   - Relative position and relative velocity conventions.
   - Continuous-time dynamics and numerical integration setup.

3. **Nominal controller**
   - Leader reference-speed tracking.
   - Follower-control law and controller gains.
   - Acceleration saturation and sign conventions.

4. **CBF safety function**
   - Exact definition of the safety function used by the code.
   - Meaning of the headway-like term and braking-related term.
   - Definition of the mesoscopic spacing/velocity terms.
   - Sign convention for the safe set and interpretation of the zero level.

5. **Quadratic-programming filter**
   - Optimization objective relative to the nominal command.
   - Implemented inequality constraint and actuator bounds.
   - Solver settings and handling of solver failures or infeasibility.
   - Recorded computational-time statistics.

6. **Simulation configuration**
   - Parameter table with values taken from the script.
   - Initial-condition generation and random seed.
   - Leader reference-speed profile.
   - Disturbance timing, amplitude, frequency, and affected vehicle.
   - Solver tolerances, time interval, and maximum integration step.

7. **Scenario definitions**
   - Scenario 1: nominal controller, braking disturbance, CBF disabled.
   - Scenario 2: same braking disturbance and initial conditions, CBF enabled.
   - Scenario 3: Scenario 2 plus sinusoidal disturbance on the leader.
   - Explicitly state which switches or code changes are needed to reproduce each scenario.

8. **Results and interpretation**
   - Distance, speed, safety-function, and control-input plots.
   - Scenario-to-scenario comparison using consistent conditions.
   - Evidence supporting any claim about collision, constraint satisfaction, or disturbance propagation.

9. **Limitations and future work**
   - Verification of equations and implementation.
   - Independent scenario selection and repeatable figure export.
   - Numerical robustness and solver-failure handling.
   - Additional validation before any real-world or formal-safety claim.

## Current implementation notes

The current script uses `N = 11`, a 30-second simulation horizon, and `ode45`. The CBF-related follower control uses `quadprog`. The leader reference-speed function changes from 15 to 5 at 5 seconds, while additional disturbance logic is present inside the dynamics function: a sinusoidal term is applied to the leader acceleration for (5 < t < 30), and an additional negative acceleration term is applied to vehicle index 10 for (10 < t < 15). These are separate pieces of logic and should be documented explicitly when describing a run.

The current script does not yet provide independent top-level switches for the nominal-only baseline, CBF activation, braking disturbance, and leader sinusoidal disturbance. The scenario descriptions are therefore experimental intentions; each scenario should be configured and run separately before its figures are described as a validated comparison.

## Validation checklist before publishing results

- [ ] Check the definitions and signs of all relative distances and velocities.
- [ ] Confirm the CBF safe-set convention and what the sign of (h) means.
- [ ] Verify that the intended braking disturbance affects the intended vehicle(s).
- [ ] Confirm the CBF is truly disabled in Scenario 1 and enabled in Scenarios 2–3.
- [ ] Keep initial conditions and all other parameters fixed for controlled comparisons.
- [ ] Check the numerical solver output and the result of every `quadprog` call.
- [ ] Confirm the distance criterion used to identify a collision.
- [ ] Re-export plots from the exact script configuration being documented.
- [ ] Avoid interpreting a nonnegative safety function or a single numerical run as a formal guarantee of safety or string stability.
