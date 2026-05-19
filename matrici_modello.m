clear all; close all; clc;

%% 1. Symbolic variables (3R Planar manipulator)
syms q1 q2 q3 real
q = [q1; q2; q3];

% Geometric parameters
syms a1 a2 a3 real           % Total lengths of the links
syms l1 l2 l3 real           % Distances of the link centers of mass from the joints (l_i)

% Inertial and mass parameters (consistent with lecture notation)
syms ml1 ml2 ml3 real        % Masses of the links (m_l_i)
syms mm1 mm2 mm3 real        % Masses of the motors (m_m_i)
syms Il1 Il2 Il3 real        % Baricentric inertias of the links (I_l_i)
syms Im1 Im2 Im3 real        % Inertias of the motor rotors (I_m_i)
syms kr1 kr2 kr3 real        % Gear reduction ratios (k_r_i)

%% 2. POSITION VECTORS (Kinematics)
% Positions of Link centers of mass (p_l_i)
p_l1 = [l1*cos(q1); l1*sin(q1); 0];
p_l2 = [a1*cos(q1) + l2*cos(q1+q2); a1*sin(q1) + l2*sin(q1+q2); 0];
p_l3 = [a1*cos(q1) + a2*cos(q1+q2) + l3*cos(q1+q2+q3); a1*sin(q1) + a2*sin(q1+q2) + l3*sin(q1+q2+q3); 0];

% Positions of Motor centers of mass (p_m_i) mounted on the joints
p_m1 = [0; 0; 0];
p_m2 = [a1*cos(q1); a1*sin(q1); 0];
p_m3 = [a1*cos(q1) + a2*cos(q1+q2); a1*sin(q1) + a2*sin(q1+q2); 0];

%% 3. JACOBIAN CALCULATION (3x3)
% --- Linear Jacobians (J_P) ---
JP_l1 = jacobian(p_l1, q);
JP_l2 = jacobian(p_l2, q);
JP_l3 = jacobian(p_l3, q);
JP_m1 = jacobian(p_m1, q); % Results in a 3x3 zero matrix
JP_m2 = jacobian(p_m2, q);
JP_m3 = jacobian(p_m3, q);

% --- Angular Jacobians (J_O) ---
z0 = [0; 0; 1];
z_0 = [0; 0; 0];

% J_O for Links (rotating solidly with preceding joints)
JO_l1 = [z0, z_0, z_0];
JO_l2 = [z0, z0, z_0];
JO_l3 = [z0, z0, z0];

% J_O for Motors (including gear ratio k_r for the specific joint)
JO_m1 = [kr1*z0, z_0, z_0];
JO_m2 = [z0, kr2*z0, z_0];
JO_m3 = [z0, z0, kr3*z0];

%% 4. INERTIA MATRIX CALCULATION B(q)
% Inertia tensors (Z-axis only for planar robots)
I_l1_mat = diag([0, 0, Il1]);
I_l2_mat = diag([0, 0, Il2]);
I_l3_mat = diag([0, 0, Il3]);
I_m1_mat = diag([0, 0, Im1]);
I_m2_mat = diag([0, 0, Im2]);
I_m3_mat = diag([0, 0, Im3]);

% Initialize B(q) as a 3x3 zero matrix
B = sym(zeros(3,3));

% Contribution of Joint 1 (Link 1 + Motor 1)
B = B + ml1*(JP_l1.' * JP_l1) + (JO_l1.' * I_l1_mat * JO_l1) ...
      + mm1*(JP_m1.' * JP_m1) + (JO_m1.' * I_m1_mat * JO_m1);

% Contribution of Joint 2 (Link 2 + Motor 2)
B = B + ml2*(JP_l2.' * JP_l2) + (JO_l2.' * I_l2_mat * JO_l2) ...
      + mm2*(JP_m2.' * JP_m2) + (JO_m2.' * I_m2_mat * JO_m2);

% Contribution of Joint 3 (Link 3 + Motor 3)
B = B + ml3*(JP_l3.' * JP_l3) + (JO_l3.' * I_l3_mat * JO_l3) ...
      + mm3*(JP_m3.' * JP_m3) + (JO_m3.' * I_m3_mat * JO_m3);

% --- Final simplification ---
B = simplify(B);
B = combine(B, 'sincos'); % Combines trigonometric terms
disp('--- Inertia Matrix B(q) successfully calculated ---');

%% 5. VELOCITY AND GRAVITY DEFINITION
syms dq1 dq2 dq3 real
dq = [dq1; dq2; dq3]; % Joint velocity vector
syms g real           % Gravity acceleration
g0 = [0; -g; 0];      % Gravity vector (pointing downwards along Y-axis)

%% 6. CORIOLIS AND CENTRIFUGAL MATRIX CALCULATION C(q, dq)
% Initialize C as a 3x3 zero matrix
C = sym(zeros(3,3));

% Apply Christoffel Symbols formula
for i = 1:3
    for j = 1:3
        for k = 1:3
            % Calculate c_ijk
            c_ijk = 0.5 * (diff(B(i,j), q(k)) + diff(B(i,k), q(j)) - diff(B(j,k), q(i)));
            % Assemble c_ij by multiplying by joint velocity dq_k
            C(i,j) = C(i,j) + c_ijk * dq(k);
        end
    end
end

% Simplify Matrix C
C = simplify(C);
C = combine(C, 'sincos');
disp('--- Coriolis Matrix C(q, dq) calculated ---');

%% 7. GRAVITY VECTOR CALCULATION g(q)
% Total Potential Energy U (Links + Motors)
U_links  = -(ml1 * g0.' * p_l1) - (ml2 * g0.' * p_l2) - (ml3 * g0.' * p_l3);
U_motori = -(mm1 * g0.' * p_m1) - (mm2 * g0.' * p_m2) - (mm3 * g0.' * p_m3);
U_tot = U_links + U_motori;

% Gravity vector g(q) is the gradient of potential energy with respect to q
G_vec = jacobian(U_tot, q).'; 

% Simplify Vector G
G_vec = simplify(G_vec);
G_vec = combine(G_vec, 'sincos');
disp('--- Gravity Vector g(q) calculated ---');

%% 8. FRICTION DEFINITION (Fv, Fs)
% Define viscous (v) and static (s) friction coefficients for the 3 joints
syms fv1 fv2 fv3 real
syms fs1 fs2 fs3 real

% Create diagonal matrices Fv and Fs
Fv = diag([fv1, fv2, fv3]);
Fs = diag([fs1, fs2, fs3]);

% Calculate dissipative torques
Tau_viscoso = Fv * dq;             % F_v * q_dot
Tau_statico = Fs * sign(dq);       % F_s * sgn(q_dot)
disp('--- Friction terms defined ---');

%% 9. EXTERNAL FORCES DEFINITION (h and J^T)
% h is the wrench (forces and moments) applied to the end-effector
syms h_x h_y h_z mu_x mu_y mu_z real
h = [h_x; h_y; h_z; mu_x; mu_y; mu_z]; % 6x1 Vector (Linear Forces and Moments)

% End-effector Tip position (final tip a3)
p_ee = [a1*cos(q1) + a2*cos(q1+q2) + a3*cos(q1+q2+q3);
        a1*sin(q1) + a2*sin(q1+q2) + a3*sin(q1+q2+q3);
        0];

% Linear and Angular Jacobian of the End-Effector
JP_ee = jacobian(p_ee, q);
JO_ee = [z0, z0, z0]; % Tip rotation influenced by all 3 joints

% Complete Geometric Jacobian (6x3)
J_ee = [JP_ee; JO_ee];

% Torques induced by external forces: J^T * h
Tau_ext = J_ee.' * h;
disp('--- External force terms calculated ---');

%% 10. COMPLETE DYNAMIC EQUATION
% Define acceleration vector
syms ddq1 ddq2 ddq3 real
ddq = [ddq1; ddq2; ddq3];

% Direct Dynamics Equation: B(q)*q_ddot + C(q,q_dot)*q_dot + Fv*q_dot + Fs*sgn(q_dot) + g(q) = Tau - J^T*h
Tau_totale = B*ddq + C*dq + Tau_viscoso + Tau_statico + G_vec + Tau_ext;
Tau_totale = simplify(Tau_totale);
disp('--- MATHEMATICAL MODEL SUCCESSFULLY COMPLETED ---');

%% 11. EXPORTING MATRICES FOR SIMULINK
disp('--- Exporting functions... Please wait ---');

% Export Inertia Matrix B(q)
matlabFunction(B, 'File', 'calc_B', 'Vars', {q, a1, a2, a3, l1, l2, l3, ml1, ml2, ml3, mm1, mm2, mm3, Il1, Il2, Il3, Im1, Im2, Im3, kr1, kr2, kr3});

% Export Coriolis Matrix C(q, dq)
matlabFunction(C, 'File', 'calc_C', 'Vars', {q, dq, a1, a2, a3, l1, l2, l3, ml1, ml2, ml3, mm1, mm2, mm3, Il1, Il2, Il3, Im1, Im2, Im3, kr1, kr2, kr3});

% Export Gravity Vector G(q)
matlabFunction(G_vec, 'File', 'calc_G', 'Vars', {q, g, a1, a2, a3, l1, l2, l3, ml1, ml2, ml3, mm1, mm2, mm3});

disp('--- Export completed! .m files generated in the current folder ---');