%% init_SMC_vehicle_project
% SMC Vehicle Speed Control Project
clc;
close all;

save_results = true;
generate_plots = true;

%% 1) Create Results Folders

mainFolder = 'SMC_Project_Results';
stepFolder = fullfile(mainFolder, 'Vehicle_Setup');

figFolder  = fullfile(stepFolder, 'Figures');
dataFolder = fullfile(stepFolder, 'Data');
logFolder  = fullfile(stepFolder, 'Logs');

folders = {mainFolder, stepFolder, figFolder, dataFolder, logFolder};

for i = 1:length(folders)
    if ~exist(folders{i}, 'dir')
        mkdir(folders{i});
    end
end

%% 2) Clean Old PNG Figures

if save_results
    oldPngFiles = dir(fullfile(figFolder, '*.png'));

    for i = 1:length(oldPngFiles)
        delete(fullfile(figFolder, oldPngFiles(i).name));
    end
end

%% 3) Start Command Window Log

if save_results
    diaryFile = fullfile(logFolder, 'Command_Window_Output.txt');

    if exist(diaryFile, 'file')
        delete(diaryFile);
    end

    diary(diaryFile);
end

%% 4) Constants

g = 9.81;   % gravity acceleration [m/s^2]

%% 5) Vehicle Parameters from Project Statement

p.M      = 1000;        % vehicle mass [kg]
p.Mr5    = 100;         % equivalent rotating mass in 5th gear [kg]
p.Meq    = p.M + p.Mr5; % equivalent longitudinal mass [kg]

p.L      = 2600e-3;     % wheelbase [m]
p.b      = 1000e-3;     % distance parameter [m]
p.Cr     = 0.02;        % rolling resistance coefficient [-]
p.ha     = 500e-3;      % aerodynamic height parameter [m]
p.h      = 700e-3;      % CG height [m]

p.eta_tf = 0.85;        % transmission efficiency [-]
p.Ntf5   = 1.2;         % total gear ratio in 5th gear [-]
p.r_tire = 0.3;         % tire radius [m]

p.theta  = 0;           % road slope angle [rad], flat road

%% 6) Drag Coefficient Uncertainty

p.Cd_min = 0.2;
p.Cd_nom = 0.3;
p.Cd_max = 0.4;

% Default value used by Simulink.
% Later, for robustness analysis, change this to 0.2 or 0.4.
Cd_sim = p.Cd_nom;

%% 7) Simulation Time

dt = 0.01;              % simulation time step [s]
t_end = 60;             % total simulation time [s]

modelName = 'sim4';

if exist([modelName '.slx'], 'file') || bdIsLoaded(modelName)

    if ~bdIsLoaded(modelName)
        load_system(modelName);
    end

    set_param(modelName, ...
        'SolverType', 'Fixed-step', ...
        'Solver', 'ode4', ...
        'FixedStep', num2str(dt), ...
        'StartTime', '0', ...
        'StopTime', num2str(t_end));

    save_system(modelName);
end
t = 0:dt:t_end;

%% 8) Reference Speed Profile
% Leading vehicle speed:
% 60 km/h -> 80 km/h -> 100 km/h over 60 seconds.

t_points = [0 30 60];             % [s]
v_points_kmh = [60 80 100];       % [km/h]

v_ref_kmh = interp1(t_points, v_points_kmh, t, 'linear');
v_ref = v_ref_kmh / 3.6;          % [m/s]

% Reference acceleration
dv_ref = gradient(v_ref, dt);     % [m/s^2]

%% 9) Resistance Forces

W = p.M * g;                      % vehicle weight [N]
Rx = p.Cr * W;                    % rolling resistance [N]

F_grade = W * sin(p.theta);       % zero for flat road

DA_min = p.Cd_min * v_ref.^2;
DA_nom = p.Cd_nom * v_ref.^2;
DA_max = p.Cd_max * v_ref.^2;

%% 10) Longitudinal Model Constants for Simulink

% Drive force:
% F_drive = (Ntf5 * eta_tf / r_tire) * Te
K_Te_to_Fx = p.Ntf5 * p.eta_tf / p.r_tire;  % 3.4 N/Nm
K_Fx_to_Te = 1 / K_Te_to_Fx;                % 0.2941176 Nm/N

% Compatibility aliases
K_torque_to_force        = K_Te_to_Fx;
K_engine_torque_to_force = K_Te_to_Fx;
K_force_to_torque        = K_Fx_to_Te;

% Acceleration:
%a = F_net / Meq;
K_acceleration = 1 / p.Meq;

% Torque-to-acceleration gain:
b_torque = K_torque_to_force / p.Meq;

% Initial vehicle speed
v0 = v_ref(1);

%% 11) Nominal Feedforward Engine Torque
% This is not the SMC controller.
% It is only the nominal torque required to follow the reference trajectory.

F_required_nom = p.Meq * dv_ref + Rx + DA_nom + F_grade;

Te_ff_nom = F_required_nom * p.r_tire / (p.Ntf5 * p.eta_tf);

%% 12) Simulink-Ready Input Signals

% Use these variables in From Workspace blocks.
v_ref_input = [t(:), v_ref(:)];             % reference speed [m/s]
v_ref_kmh_input = [t(:), v_ref_kmh(:)];     % reference speed [km/h], only for display if needed
dv_ref_input = [t(:), dv_ref(:)];           % reference acceleration [m/s^2]
Te_input = [t(:), Te_ff_nom(:)];            % nominal feedforward torque [Nm]

%% 13) Actuator Limits for Simulink

Te_min = 0;        % minimum engine torque [Nm]
Te_max = 300;      % maximum engine torque [Nm]

%% 14) Placeholder SMC Parameters for Later Steps
% These are not used yet in the open-loop model.
% They will be used when the SMC controller is added.

lambda_smc = 2.0;      % [1/s]
k_smc      = 0.5;      % [m/s^2]
eta_smc    = k_smc;    % compatibility with current Simulink block
phi_smc    = 0.1;      % [m/s]
%% 15) Print Summary

fprintf('================ Vehicle Setup Results ================\n');
fprintf('Equivalent mass Meq        = %.2f kg\n', p.Meq);
fprintf('Initial reference speed    = %.2f m/s = %.2f km/h\n', v_ref(1), v_ref_kmh(1));
fprintf('Final reference speed      = %.2f m/s = %.2f km/h\n', v_ref(end), v_ref_kmh(end));
fprintf('Reference acceleration     = %.4f m/s^2\n', dv_ref(10));
fprintf('Rolling resistance Rx      = %.2f N\n', Rx);
fprintf('K torque to force          = %.4f N/Nm\n', K_torque_to_force);
fprintf('K acceleration             = %.8f 1/kg\n', K_acceleration);
fprintf('Torque input gain b        = %.6f (m/s^2)/Nm\n', b_torque);
fprintf('Initial nominal torque     = %.2f Nm\n', Te_ff_nom(1));
fprintf('Final nominal torque       = %.2f Nm\n', Te_ff_nom(end));
fprintf('Cd uncertainty range       = [%.2f, %.2f]\n', p.Cd_min, p.Cd_max);
fprintf('Nominal Cd for simulation  = %.2f\n', Cd_sim);
fprintf('=======================================================\n');

%% 16) Save Numerical Data

if save_results

    resultsTable = table( ...
        t(:), ...
        v_ref(:), ...
        v_ref_kmh(:), ...
        dv_ref(:), ...
        DA_min(:), ...
        DA_nom(:), ...
        DA_max(:), ...
        Te_ff_nom(:), ...
        'VariableNames', { ...
        'Time_s', ...
        'ReferenceSpeed_mps', ...
        'ReferenceSpeed_kmh', ...
        'ReferenceAcceleration_mps2', ...
        'DragForce_Cd_0p2_N', ...
        'DragForce_Cd_0p3_Nominal_N', ...
        'DragForce_Cd_0p4_N', ...
        'NominalFeedforwardTorque_Nm'});

    writetable(resultsTable, fullfile(dataFolder, 'Numerical_Results.csv'));

    save(fullfile(dataFolder, 'Workspace_Data.mat'), ...
        'p', 'g', 'dt', 't_end', 't', ...
        'v_ref', 'v_ref_kmh', 'dv_ref', ...
        'Rx', 'DA_min', 'DA_nom', 'DA_max', ...
        'F_grade', 'b_torque', ...
        'K_torque_to_force', 'K_acceleration', ...
        'Te_ff_nom', 'Te_input', ...
        'v_ref_input', 'v_ref_kmh_input', 'dv_ref_input', ...
        'Cd_sim', 'v0', ...
        'Te_min', 'Te_max', ...
        'lambda_smc', 'eta_smc', 'phi_smc', ...
        'resultsTable');

end

%% 17) Generate and Save Figures

if generate_plots

    fig1 = figure;
    plot(t, v_ref_kmh, 'LineWidth', 2);
    grid on;
    xlabel('Time [s]');
    ylabel('Reference Speed [km/h]');
    title('Reference Speed Profile: 60 to 100 km/h');

    if save_results
        saveFigurePNG(fig1, figFolder, 'Fig01_Reference_Speed_Profile');
    end

    fig2 = figure;
    plot(t, DA_min, 'LineWidth', 2); hold on;
    plot(t, DA_nom, '--', 'LineWidth', 2);
    plot(t, DA_max, 'LineWidth', 2);
    grid on;
    xlabel('Time [s]');
    ylabel('Aerodynamic Drag Force D_A [N]');
    title('Aerodynamic Drag Force under C_d Uncertainty');
    legend('C_d = 0.2', 'C_d = 0.3 nominal', 'C_d = 0.4', 'Location', 'best');

    if save_results
        saveFigurePNG(fig2, figFolder, 'Fig02_Drag_Force_Uncertainty');
    end

    fig3 = figure;
    plot(t, Te_ff_nom, 'LineWidth', 2);
    grid on;
    xlabel('Time [s]');
    ylabel('Feedforward Engine Torque T_e [Nm]');
    title('Nominal Feedforward Engine Torque for Reference Speed Tracking');

    if save_results
        saveFigurePNG(fig3, figFolder, 'Fig03_Nominal_Feedforward_Engine_Torque');
    end

end

%% 18) Save Summary Text File

if save_results

    summaryFile = fullfile(dataFolder, 'Summary.txt');

    fid = fopen(summaryFile, 'w');

    fprintf(fid, 'Vehicle Setup Summary\n');
    fprintf(fid, '====================================\n');
    fprintf(fid, 'Equivalent mass Meq        = %.2f kg\n', p.Meq);
    fprintf(fid, 'Initial reference speed    = %.2f m/s = %.2f km/h\n', v_ref(1), v_ref_kmh(1));
    fprintf(fid, 'Final reference speed      = %.2f m/s = %.2f km/h\n', v_ref(end), v_ref_kmh(end));
    fprintf(fid, 'Reference acceleration     = %.4f m/s^2\n', dv_ref(10));
    fprintf(fid, 'Rolling resistance Rx      = %.2f N\n', Rx);
    fprintf(fid, 'K torque to force          = %.4f N/Nm\n', K_torque_to_force);
    fprintf(fid, 'K acceleration             = %.8f 1/kg\n', K_acceleration);
    fprintf(fid, 'Torque input gain b        = %.6f (m/s^2)/Nm\n', b_torque);
    fprintf(fid, 'Initial nominal torque     = %.2f Nm\n', Te_ff_nom(1));
    fprintf(fid, 'Final nominal torque       = %.2f Nm\n', Te_ff_nom(end));

    fprintf(fid, '\nDrag Coefficient Uncertainty:\n');
    fprintf(fid, 'Cd_min = %.2f\n', p.Cd_min);
    fprintf(fid, 'Cd_nom = %.2f\n', p.Cd_nom);
    fprintf(fid, 'Cd_max = %.2f\n', p.Cd_max);
    fprintf(fid, 'Cd_sim = %.2f\n', Cd_sim);

    fprintf(fid, '\nSimulink Variables:\n');
    fprintf(fid, 'v_ref_input      : [time, reference speed in m/s]\n');
    fprintf(fid, 'v_ref_kmh_input  : [time, reference speed in km/h]\n');
    fprintf(fid, 'Te_input         : [time, nominal feedforward torque in Nm]\n');
    fprintf(fid, 'K_torque_to_force = %.4f\n', K_torque_to_force);
    fprintf(fid, 'K_acceleration    = %.8f\n', K_acceleration);
    fprintf(fid, 'v0                = %.4f m/s\n', v0);
    fprintf(fid, 'Te_min            = %.2f Nm\n', Te_min);
    fprintf(fid, 'Te_max            = %.2f Nm\n', Te_max);

    fclose(fid);

end

%% 19) Finish Logging

if save_results
    fprintf('\nAll vehicle setup figures and results were saved in:\n');
    fprintf('%s\n', stepFolder);
    diary off;
end

%% Local Function: Save Figure as PNG Only

function saveFigurePNG(figHandle, figFolder, fileName)

    set(figHandle, 'Color', 'w');

    pngPath = fullfile(figFolder, [fileName, '.png']);

    print(figHandle, pngPath, '-dpng', '-r300');

end