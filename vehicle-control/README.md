# Vehicle Control

Three MATLAB/Simulink course projects by Forough Sadat Razavi demonstrating adaptive and nonlinear control for vehicle path tracking and longitudinal motion.

| Project | Focus |
| --- | --- |
| `adaptive-lateral-path-tracking` | Lateral path tracking with online cornering-stiffness estimation |
| `adaptive-cruise-control-mras` | Model-reference adaptive cruise control |
| `sliding-mode-speed-control` | Longitudinal sliding-mode control, diagnostics, and sensitivity analysis |

How to explore the projects

Download the repository and open each project folder in MATLAB. The .slx models require Simulink. The models were saved with MATLAB R2022b.

Lateral dynamics: Run codes/project1.m to initialize the model, open Sim_project1.slx, and run the simulation. codes/plot_results.m uses the simulation output out. Before running any of the additional steering or parameter-study scripts, enter model_name = 'Sim_project1'; in the MATLAB Command Window. This variable is used by those scripts but is not defined inside them.

Quarter-car ride and suspension: Open sim2.slx, add codes to the MATLAB path, and run codes/init_quarter_car.m before the simulation or analysis scripts. The initialization script changes MATLAB's current folder to codes; generated inputs and results may therefore appear inside that folder.

Parameter identification: Run project1.m and the project1sim.slx model before LS.m. The LS script expects a simulation output named out. RLS.m is a separate MATLAB simulation and uses random measurement noise, so repeated runs may produce different numerical results.

Each project folder has its own README and source files. These are simulation-based coursework studies; the numerical results have not been independently reproduced for this repository. They should not be interpreted as hardware or road-test validation.
