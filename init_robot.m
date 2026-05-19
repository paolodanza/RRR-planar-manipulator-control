%% Physical parameters for the 3R Planar Manipulator
clear all; clc;

% Gravity constant
g = 9.81;

% Link lengths (meters)
a1 = 1.0; a2 = 1.0; a3 = 0.5;

% Center of mass positions (assumed at half length)
l1 = a1/2; l2 = a2/2; l3 = a3/2;

% ACTUAL link masses (kg) - Physical Plant
ml1 = 5.0; ml2 = 3.0; ml3 = 1.0;

% ESTIMATED link masses (kg) - Internal Controller Model
% Used to test robustness against parameter mismatch
% ml1_stimato = 4.5; ml2_stimato = 3.6; ml3_stimato = 0.8; % 10-20% error
ml1_stimato = 1; ml2_stimato = 1; ml3_stimato = 0.3;

% Motor masses (kg)
mm1 = 2.0; mm2 = 1.5; mm3 = 1.0;

% Link moments of inertia (I_zz = 1/12 * m * L^2 for a slender rod)
Il1 = (1/12)*ml1*a1^2; 
Il2 = (1/12)*ml2*a2^2; 
Il3 = (1/12)*ml3*a3^2;

% Motor rotor inertias (kg*m^2)
Im1 = 0.01; Im2 = 0.01; Im3 = 0.01;

% Motor gear reduction ratios (k_r)
kr1 = 50; kr2 = 50; kr3 = 50;

% Friction matrices (Viscous and Static)
Fv = diag([0.1, 0.1, 0.1]); 
Fs = diag([0, 0, 0]);

%% Controller Parameters
% Computed Torque Controller Gains (PD) - Previous attempts and benchmarks
% Kp = diag([100, 100, 100]); % Proportional (Stiffness)
% Kv = diag([20, 20, 20]);    % Derivative (Damping)
% Kp = diag([10, 10, 10]); 
% Kv = diag([5, 5, 5]);    

% Specifications: Settling time 0.5s, zero overshoot
% Kp = diag([64, 64, 64]);
% Kv = diag([16, 16, 16]);

% Specifications: Settling time 1.5s, zero overshoot (Zeta = 1)
% wn = 2.6667 rad/s
% Kp_val = 7.1111;
% Kv_val = 5.3333;

% --- APPROACH 1 ---
% Specifications: Settling time 1s (Using robust Lyapunov formulas)
% Ts = 0.1;
% a = 4/Ts;
a = 20;
Kv_val = 2 * a;
Kp_val = 4 * a;

% --- APPROACH 2 ---
% Specifications: Settling time Ts sec, Overshoot Delta
% Ts = 0.5;
% delta = 1;
% Kv_val = 8/Ts;
% Kp_val = 16/(delta^2 * Ts^2);

% Integral Action gain (Ki)
Ki_val = Kp_val;

% Final Gain Matrices (Isotropic)
Kp = diag([Kp_val, Kp_val, Kp_val]);
Kv = diag([Kv_val, Kv_val, Kv_val]);
Ki = diag([Ki_val, Ki_val, Ki_val]);