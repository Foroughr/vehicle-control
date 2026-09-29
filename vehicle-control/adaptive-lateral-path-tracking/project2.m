clc;
clear;
close all;

%% ==========================================================
% Vehicle parameters
% Caf and Car are assumed to be the cornering stiffness of
% one tire at the front and rear axle, respectively.
% Therefore, 2*Caf and 2*Car are used in the bicycle model.
% ==========================================================

m  = 1050;      % Vehicle mass (kg)
Iz = 2000;      % Yaw moment of inertia (kg.m^2)

lf = 1.1;       % Distance from CG to front axle (m)
lr = 1.3;       % Distance from CG to rear axle (m)
L  = lf + lr;   % Wheelbase (m)

Caf = 60000;    % Front tire cornering stiffness per tire (N/rad)
Car = 60000;    % Rear tire cornering stiffness per tire (N/rad)
Cnom = 60000;   % Nominal tire cornering stiffness (N/rad)

%% ==========================================================
% Road parameters
% ==========================================================

Aroad = 5;          % Sine road amplitude (m)
lambdaRoad = 100;   % Sine road wavelength (m)

%% ==========================================================
% Simulation parameters
% ==========================================================

Ts = 0.001;         % Simulation sample time (s)
Tsim = 50;          % Total simulation time (s)

% Select vehicle speed
% Uncomment only one speed at a time.

%Vx = 60/3.6;      % 60 km/h
%Vx = 80/3.6;      % 80 km/h
Vx = 100/3.6;       % 100 km/h

%% ==========================================================
% Steering saturation
% ==========================================================

deltaMax = 0.25;    % Maximum steering angle (rad), about 14.3 degrees

%% ==========================================================
% RLS parameters
% ==========================================================

lambdaRLS = 0.995;  % Forgetting factor
Cinit = 30000;      % Initial estimate of tire cornering stiffness (N/rad)
Pinit = 1e4;        % Initial covariance for scalar RLS estimator