%% ACC with MRAS Controller - Initialization Script

clear; clc; close all;

%% Vehicle Parameters
M       = 1000;        % kg
Mr2     = 200;         % kg, equivalent rotating mass
M_eq    = M + Mr2;     % total equivalent mass

Cd      = 0.4;         % DA = Cd * V^2
Cr      = 0.02;        % rolling resistance coefficient

eta_tf  = 0.85;        % transmission efficiency
N_tf2   = 2.8;         % second gear ratio
r_tire  = 0.3;         % m

g       = 9.81;
theta_deg = 5;
theta_rad = deg2rad(theta_deg);

T_sim   = 30;          % sec

%% Force/Torque Conversion
K_torque = eta_tf * N_tf2 / r_tire;
% F_engine = K_torque * Te

W = M * g;

F_roll_const  = Cr * M * g * cos(theta_rad);
F_grade_const = M * g * sin(theta_rad);

%% Initial Conditions
vE0 = 50/3.6;          % ego vehicle initial speed, m/s
xE0 = 0;               % ego initial position

d0  = 60;              % initial distance to lead car, m
xL0 = d0;              % lead car initial position

%% ACC Parameters
D_ref = 45;            % desired constant distance, m
K_gap = 0.25;          % distance-to-speed gain

v_cmd_min = 0;
v_cmd_max = 90/3.6;    % max commanded speed, m/s

%% Reference Model Parameters
tau_m = 0.8;           % reference model time constant
% v_m_dot = (v_cmd - v_m)/tau_m

%% MRAS Adaptive Gains
gamma_x = 0.001;
gamma_r = 0.001;
gamma_0 = 0.01;

%% Initial Adaptive Parameters
Kx0 = -M_eq / (K_torque * tau_m);
Kr0 =  M_eq / (K_torque * tau_m);
K00 = (F_roll_const + F_grade_const) / K_torque;

%% Engine Torque Limits
Te_min = -500;         % Nm, negative means braking/equivalent braking torque
Te_max = 600;          % Nm

fprintf('Initialization complete.\n');