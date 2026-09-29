%% ACC-MRAS Final Results Script

clc; close all;

%% ================================
%  Output folder
%% ================================

outputFolder = 'ACC_MRAS_Results';

if ~exist(outputFolder, 'dir')
    mkdir(outputFolder);
end

%% ================================
%  Desired distance
%% ================================

if evalin('base', 'exist(''D_ref'', ''var'')')
    D_ref_value = evalin('base', 'D_ref');
else
    D_ref_value = 45;
    warning('D_ref was not found in workspace. D_ref = 45 m was used.');
end

%% ================================
%  Extract signals from Simulink output
%  Works with:
%  1) out.get('signal_name')
%  2) direct workspace variable
%% ================================

vL_kmh_out   = getLoggedSignal('vL_kmh_out');
vE_kmh_out   = getLoggedSignal('vE_kmh_out');
vm_kmh_out   = getLoggedSignal('vm_kmh_out');

d_out        = getLoggedSignal('d_out');
ed_out       = getLoggedSignal('ed_out');

Te_out       = getLoggedSignal('Te_out');

Kx_out       = getLoggedSignal('Kx_out');
Kr_out       = getLoggedSignal('Kr_out');
K0_out       = getLoggedSignal('K0_out');

e_mras_out   = getLoggedSignal('e_mras_out');

aE_out       = getLoggedSignal('aE_out');
Fdrag_out    = getLoggedSignal('Fdrag_out');

% Optional signals
has_vcmd = true;
has_vw   = true;

try
    vcmd_kmh_out = getLoggedSignal('vcmd_kmh_out');
catch
    has_vcmd = false;
    warning('vcmd_kmh_out was not found. Command speed plot will be skipped.');
end

try
    vw_kmh_out = getLoggedSignal('vw_kmh_out');
catch
    has_vw = false;
    warning('vw_kmh_out was not found. Wind speed plot will be skipped.');
end

%% ================================
%  Convert timeseries / structure to numeric vectors
%% ================================

[t, vL_kmh] = getTimeData(vL_kmh_out);
[~, vE_kmh] = getTimeData(vE_kmh_out);
[~, vm_kmh] = getTimeData(vm_kmh_out);

[~, d]      = getTimeData(d_out);
[~, ed]     = getTimeData(ed_out);

[~, Te]     = getTimeData(Te_out);

[~, Kx]     = getTimeData(Kx_out);
[~, Kr]     = getTimeData(Kr_out);
[~, K0]     = getTimeData(K0_out);

[~, e_mras] = getTimeData(e_mras_out);

[~, aE]     = getTimeData(aE_out);
[~, Fdrag]  = getTimeData(Fdrag_out);

if has_vcmd
    [~, vcmd_kmh] = getTimeData(vcmd_kmh_out);
end

if has_vw
    [~, vw_kmh] = getTimeData(vw_kmh_out);
end

%% ================================
%  Figure 1: Speeds
%% ================================

figure;
plot(t, vL_kmh, 'LineWidth', 1.8); hold on;
plot(t, vE_kmh, 'LineWidth', 1.8);
plot(t, vm_kmh, 'LineWidth', 1.8);
grid on;

xlabel('Time [s]');
ylabel('Speed [km/h]');
title('Lead Vehicle, Ego Vehicle, and Reference Model Speeds');

legend('Lead vehicle speed v_L', ...
       'Ego vehicle speed v_E', ...
       'Reference model speed v_m', ...
       'Location', 'best');

exportgraphics(gcf, fullfile(outputFolder, '01_Speeds_kmh.png'), 'Resolution', 300);

%% ================================
%  Figure 2: ACC commanded speed vs Reference Model
%% ================================

if has_vcmd
    figure;
    plot(t, vcmd_kmh, 'LineWidth', 1.8); hold on;
    plot(t, vm_kmh, 'LineWidth', 1.8);
    grid on;

    xlabel('Time [s]');
    ylabel('Speed [km/h]');
    title('ACC Commanded Speed and Reference Model Speed');

    legend('ACC commanded speed v_{cmd}', ...
           'Reference model speed v_m', ...
           'Location', 'best');

    exportgraphics(gcf, fullfile(outputFolder, '02_vcmd_vs_vm.png'), 'Resolution', 300);
end

%% ================================
%  Figure 3: Distance Tracking
%% ================================

figure;
plot(t, d, 'LineWidth', 1.8); hold on;
plot(t, D_ref_value * ones(size(t)), '--', 'LineWidth', 1.8);
grid on;

xlabel('Time [s]');
ylabel('Distance [m]');
title('Inter-Vehicle Distance Tracking');

legend('Actual distance d', ...
       'Desired distance D_{ref}', ...
       'Location', 'best');

exportgraphics(gcf, fullfile(outputFolder, '03_Distance_Tracking.png'), 'Resolution', 300);

%% ================================
%  Figure 4: Spacing Error
%% ================================

figure;
plot(t, ed, 'LineWidth', 1.8);
grid on;

xlabel('Time [s]');
ylabel('Spacing Error [m]');
title('Spacing Error Response');

exportgraphics(gcf, fullfile(outputFolder, '04_Spacing_Error.png'), 'Resolution', 300);

%% ================================
%  Figure 5: Engine Torque
%% ================================

figure;
plot(t, Te, 'LineWidth', 1.8);
grid on;

xlabel('Time [s]');
ylabel('Engine Torque [N.m]');
title('Adaptive Engine Torque Command');

exportgraphics(gcf, fullfile(outputFolder, '05_Engine_Torque.png'), 'Resolution', 300);

%% ================================
%  Figure 6: MRAS Adaptive Parameters
%% ================================

figure;
plot(t, Kx, 'LineWidth', 1.8); hold on;
plot(t, Kr, 'LineWidth', 1.8);
plot(t, K0, 'LineWidth', 1.8);
grid on;

xlabel('Time [s]');
ylabel('Adaptive Parameters');
title('MRAS Adaptive Parameters');

legend('K_x', 'K_r', 'K_0', 'Location', 'best');

exportgraphics(gcf, fullfile(outputFolder, '06_MRAS_Adaptive_Parameters.png'), 'Resolution', 300);

%% ================================
%  Figure 7: MRAS Speed Tracking Error
%% ================================

figure;
plot(t, e_mras, 'LineWidth', 1.8);
grid on;

xlabel('Time [s]');
ylabel('MRAS Speed Error [m/s]');
title('MRAS Speed Tracking Error');

exportgraphics(gcf, fullfile(outputFolder, '07_MRAS_Error.png'), 'Resolution', 300);

%% ================================
%  Figure 8: Ego Vehicle Acceleration
%% ================================

figure;
plot(t, aE, 'LineWidth', 1.8);
grid on;

xlabel('Time [s]');
ylabel('Acceleration [m/s^2]');
title('Ego Vehicle Acceleration');

exportgraphics(gcf, fullfile(outputFolder, '08_Ego_Acceleration.png'), 'Resolution', 300);

%% ================================
%  Figure 9: Aerodynamic Drag Force
%% ================================

figure;
plot(t, Fdrag, 'LineWidth', 1.8);
grid on;

xlabel('Time [s]');
ylabel('Drag Force [N]');
title('Aerodynamic Drag Force under Head Wind Disturbance');

exportgraphics(gcf, fullfile(outputFolder, '09_Aerodynamic_Drag.png'), 'Resolution', 300);

%% ================================
%  Figure 10: Head Wind Speed
%% ================================

if has_vw
    figure;
    plot(t, vw_kmh, 'LineWidth', 1.8);
    grid on;

    xlabel('Time [s]');
    ylabel('Head Wind Speed [km/h]');
    title('Head Wind Speed Profile');

    exportgraphics(gcf, fullfile(outputFolder, '10_Head_Wind_Profile.png'), 'Resolution', 300);


figure;
plot(t, Kx - Kx(1), 'LineWidth', 1.8); hold on;
plot(t, Kr - Kr(1), 'LineWidth', 1.8);
plot(t, K0 - K0(1), 'LineWidth', 1.8);
grid on;
xlabel('Time [s]');
ylabel('Parameter variation');
legend('\DeltaK_x','\DeltaK_r','\DeltaK_0');
title('Variations of MRAS Adaptive Parameters');

end

%% ================================
%  Numerical Performance Metrics
%% ================================

final_spacing_error = ed(end);
min_spacing_error   = min(ed);
max_spacing_error   = max(ed);

initial_distance    = d(1);
min_distance        = min(d);
max_distance        = max(d);
final_distance      = d(end);

max_deceleration    = min(aE);
max_acceleration    = max(aE);

min_torque          = min(Te);
max_torque          = max(Te);

initial_vL_kmh      = vL_kmh(1);
initial_vE_kmh      = vE_kmh(1);
initial_vm_kmh      = vm_kmh(1);

final_vL_kmh        = vL_kmh(end);
final_vE_kmh        = vE_kmh(end);
final_vm_kmh        = vm_kmh(end);

max_abs_mras_error  = max(abs(e_mras));
final_mras_error    = e_mras(end);

fprintf('\n========== ACC-MRAS Performance Metrics ==========\n');

fprintf('Desired distance D_ref           = %.3f m\n', D_ref_value);
fprintf('Initial distance                 = %.3f m\n', initial_distance);
fprintf('Minimum distance                 = %.3f m\n', min_distance);
fprintf('Maximum distance                 = %.3f m\n', max_distance);
fprintf('Final distance                   = %.3f m\n', final_distance);

fprintf('\nMaximum spacing error             = %.3f m\n', max_spacing_error);
fprintf('Minimum spacing error             = %.3f m\n', min_spacing_error);
fprintf('Final spacing error               = %.3f m\n', final_spacing_error);

fprintf('\nInitial lead vehicle speed         = %.3f km/h\n', initial_vL_kmh);
fprintf('Initial ego vehicle speed          = %.3f km/h\n', initial_vE_kmh);
fprintf('Initial reference model speed      = %.3f km/h\n', initial_vm_kmh);

fprintf('\nFinal lead vehicle speed           = %.3f km/h\n', final_vL_kmh);
fprintf('Final ego vehicle speed            = %.3f km/h\n', final_vE_kmh);
fprintf('Final reference model speed        = %.3f km/h\n', final_vm_kmh);

fprintf('\nMaximum deceleration               = %.3f m/s^2\n', max_deceleration);
fprintf('Maximum acceleration               = %.3f m/s^2\n', max_acceleration);

fprintf('\nMinimum torque command             = %.3f N.m\n', min_torque);
fprintf('Maximum torque command             = %.3f N.m\n', max_torque);

fprintf('\nMaximum absolute MRAS error         = %.3f m/s\n', max_abs_mras_error);
fprintf('Final MRAS error                   = %.3f m/s\n', final_mras_error);

fprintf('==================================================\n');

%% ================================
%  Save performance metrics as table
%% ================================

Metric = {
    'Desired distance D_ref';
    'Initial distance';
    'Minimum distance';
    'Maximum distance';
    'Final distance';
    'Maximum spacing error';
    'Minimum spacing error';
    'Final spacing error';
    'Initial lead vehicle speed';
    'Initial ego vehicle speed';
    'Initial reference model speed';
    'Final lead vehicle speed';
    'Final ego vehicle speed';
    'Final reference model speed';
    'Maximum deceleration';
    'Maximum acceleration';
    'Minimum torque command';
    'Maximum torque command';
    'Maximum absolute MRAS error';
    'Final MRAS error'
    };

Value = [
    D_ref_value;
    initial_distance;
    min_distance;
    max_distance;
    final_distance;
    max_spacing_error;
    min_spacing_error;
    final_spacing_error;
    initial_vL_kmh;
    initial_vE_kmh;
    initial_vm_kmh;
    final_vL_kmh;
    final_vE_kmh;
    final_vm_kmh;
    max_deceleration;
    max_acceleration;
    min_torque;
    max_torque;
    max_abs_mras_error;
    final_mras_error
    ];

Unit = {
    'm';
    'm';
    'm';
    'm';
    'm';
    'm';
    'm';
    'm';
    'km/h';
    'km/h';
    'km/h';
    'km/h';
    'km/h';
    'km/h';
    'm/s^2';
    'm/s^2';
    'N.m';
    'N.m';
    'm/s';
    'm/s'
    };

ResultsTable = table(Metric, Value, Unit);

disp(ResultsTable);

writetable(ResultsTable, fullfile(outputFolder, 'ACC_MRAS_Performance_Metrics.xlsx'));
writetable(ResultsTable, fullfile(outputFolder, 'ACC_MRAS_Performance_Metrics.csv'));

fprintf('\nAll figures and performance tables were saved in folder: %s\n', outputFolder);

%% ================================
%  Local helper functions
%% ================================

function sig = getLoggedSignal(signalName)
    % This function tries to get a signal from:
    % 1) Simulink SimulationOutput object named "out"
    % 2) Direct base workspace variable

    if evalin('base', 'exist(''out'', ''var'')')
        simOut = evalin('base', 'out');

        try
            sig = simOut.get(signalName);
            return;
        catch
            % Continue and try direct workspace
        end
    end

    if evalin('base', sprintf('exist(''%s'', ''var'')', signalName))
        sig = evalin('base', signalName);
    else
        error(['Signal "' signalName '" was not found. ', ...
               'Check the To Workspace variable name and run the Simulink model again.']);
    end
end

function [t, y] = getTimeData(sig)
    % This function supports:
    % 1) Timeseries format
    % 2) Structure With Time format

    if isa(sig, 'timeseries')
        t = sig.Time;
        y = squeeze(sig.Data);

    elseif isstruct(sig) && isfield(sig, 'time') && isfield(sig, 'signals')
        t = sig.time;
        y = squeeze(sig.signals.values);

    else
        error('Unsupported signal format. Use Timeseries or Structure With Time in To Workspace.');
    end

    t = t(:);
    y = y(:);
end

