% =========================================================
% File: Assignment2_Section22_AG16.m
% Purpose:
%   Section 2.2 - Gravity torque evaluation across workspace
%   (COM Jacobians + payload + motor feasibility check)
%
% Kinematic convention used:
%   DH columns: [a_{i-1}, alpha_{i-1}, d_i, theta_i]
%   A_i = Rx(alpha_{i-1}) * Tx(a_{i-1}) * Rz(theta_i) * Tz(d_i)
% =========================================================
clc; clear; close all;

%% -----------------------------
% Robot geometry (SI units)
%% -----------------------------
b  = 0.025;
L1 = 0.05;
L2 = 0.22;
L3 = 0.18;
L4 = 0.10;
N  = 5;

%% -----------------------------
% Link masses (kg)
%% -----------------------------
m = [0.073; 0.093; 0.083; 0.071; 0.049];

pc_local = [ ...
   -0.0247,  0.1750, 0.1500, 0.0000,  0.0000;
    0.0000,  0.0000, 0.0000, 0.0887,  0.0000;
    0.0000,  0.0000, 0.0000, 0.0000, -0.0100];

%% -----------------------------
% Payload (heaviest chess piece)
%% -----------------------------
m_payload = 0.060;  % kg

%% -----------------------------
% Motor nominal torque (rated continuous @ 6V)
% STS3215: 6.5 kg*cm
% STS3045M: 2.0 kg*cm
%% -----------------------------
kgcm_to_Nm = 0.0980665;
tau_STS3215  = 6.5 * kgcm_to_Nm;   % 0.6374 N*m
tau_STS3045M = 2.0 * kgcm_to_Nm;   % 0.1961 N*m

% Joints 1-4: STS3215, Joint 5: STS3045M
tau_nominal = [tau_STS3215; tau_STS3215; tau_STS3215; tau_STS3215; tau_STS3045M];

%% -----------------------------
% Gravity vector in base frame
%% -----------------------------
g0 = [0; 0; -9.81];

%% -----------------------------
% Workspace sampling (similar style to A1)
% You can increase/decrease density
%% -----------------------------
q1Range = deg2rad(linspace(-180, 180, 13));
q2Range = deg2rad(linspace(-90,   90,  9));
q3Range = deg2rad(linspace(-120, 120,  9));
q4Range = deg2rad(linspace(-120, 120,  9));
q5Range = deg2rad(linspace(-90,   90,  7));

numSamples = numel(q1Range)*numel(q2Range)*numel(q3Range)*numel(q4Range)*numel(q5Range);
fprintf('Total samples: %d\n', numSamples);

%% -----------------------------
% Search max required gravity torque
%% -----------------------------
maxAbsTau = zeros(N,1);
qAtMax = zeros(N,N);              % each column stores q where joint j is max
tauAtMaxConfig = zeros(N,N);      % tau vector at that config for each joint j

count = 0;
tic;
for q1 = q1Range
for q2 = q2Range
for q3 = q3Range
for q4 = q4Range
for q5 = q5Range

    q = [q1;q2;q3;q4;q5];

    [T_all, o, z] = fk_all(q, b, L1, L2, L3, L4);  % frames 1..5
    oE = o(:,5);

    tau_g = zeros(N,1);

    % Link self-weight contributions
    for i = 1:N
        Ri = T_all(1:3,1:3,i);
        p_ci = Ri * pc_local(:,i) + o(:,i);

        % COM Jacobian Jv_ci with j<=i active
        Jv_ci = jacobian_point_on_link(i, p_ci, o, z);
        tau_g = tau_g + Jv_ci' * (m(i) * g0);
    end

    % Payload contribution at end-effector origin
    Jv_E = jacobian_point_general(oE, o, z);
    tau_g = tau_g + Jv_E' * (m_payload * g0);

    % Track maxima per joint
    for j = 1:N
        if abs(tau_g(j)) > maxAbsTau(j)
            maxAbsTau(j) = abs(tau_g(j));
            qAtMax(:,j) = q;
            tauAtMaxConfig(:,j) = tau_g;
        end
    end

    count = count + 1;
    if mod(count,5000)==0
        fprintf('Processed %d / %d\n', count, numSamples);
    end

end
end
end
end
end
tElapsed = toc;

%% -----------------------------
% Results
%% -----------------------------
margin = tau_nominal - maxAbsTau;

fprintf('\n========== 2.2 Gravity Torque Results ==========\n');
fprintf('Processed: %d / %d (%.2f s)\n', count, numSamples, tElapsed);

for j = 1:N
    fprintf('Joint %d: max|tau_g| = %.4f N*m at q = [%.1f %.1f %.1f %.1f %.1f] deg\n', ...
        j, maxAbsTau(j), ...
        rad2deg(qAtMax(1,j)), rad2deg(qAtMax(2,j)), rad2deg(qAtMax(3,j)), ...
        rad2deg(qAtMax(4,j)), rad2deg(qAtMax(5,j)));
end

fprintf('\nNominal torque and margin:\n');
for j = 1:N
    fprintf('Joint %d: tau_nom = %.4f, margin = %+0.4f N*m\n', ...
        j, tau_nominal(j), margin(j));
end

if any(margin < 0)
    fprintf('\nWARNING: Some joints exceed nominal torque.\n');
else
    fprintf('\nPASS: All joints within nominal torque.\n');
end

%% -----------------------------
% Save result table (CSV)
%% -----------------------------
T = table((1:N)', maxAbsTau, tau_nominal, margin, ...
    'VariableNames', {'Joint','TauRequiredMax_Nm','TauNominal_Nm','Margin_Nm'});
writetable(T, 'Section22_torque_summary.csv');

%% -----------------------------
% Plot: required vs nominal
%% -----------------------------
fig = figure('Color','w','Position',[120 120 980 420]);
subplot(1,2,1);
bar(1:N, maxAbsTau, 0.65); grid on;
xlabel('Joint'); ylabel('Max required |tau_g| (N*m)');
title('Required Gravity Torque');

subplot(1,2,2);
bar(1:N, [tau_nominal, maxAbsTau], 0.8); grid on;
xlabel('Joint'); ylabel('Torque (N*m)');
legend('Nominal','Required','Location','best');
title('Nominal vs Required');

sgtitle('Section 2.2 Motor Feasibility');
saveas(fig, 'Section22_torque_result.png');

%% =========================================================
% Local functions
%% =========================================================

function [T_all, o, z] = fk_all(q, b, L1, L2, L3, L4)
    A1 = dhT(0,   0,      L1, q(1));
    A2 = dhT(-b,  pi/2,   0,  q(2) + pi/2);
    A3 = dhT(L2,  0,      0,  q(3));
    A4 = dhT(L3,  0,      0,  q(4) - pi/2);
    A5 = dhT(0,  -pi/2,   L4, q(5));

    A = cat(3,A1,A2,A3,A4,A5);

    T = eye(4);
    T_all = zeros(4,4,5);
    o = zeros(3,5);  % o_j
    z = zeros(3,5);  % z_j

    for j = 1:5
        T = T * A(:,:,j);
        T_all(:,:,j) = T;
        o(:,j) = T(1:3,4);
        z(:,j) = T(1:3,3);
    end
end

function Jv = jacobian_point_on_link(i, p, o, z)
    % COM Jacobian for link i: only joints j<=i contribute
    N = 5;
    Jv = zeros(3,N);
    for j = 1:N
        if j <= i
            Jv(:,j) = cross(z(:,j), p - o(:,j));
        else
            Jv(:,j) = [0;0;0];
        end
    end
end

function Jv = jacobian_point_general(p, o, z)
    % Full point Jacobian (all joints contribute)
    N = 5;
    Jv = zeros(3,N);
    for j = 1:N
        Jv(:,j) = cross(z(:,j), p - o(:,j));
    end
end

function A = dhT(a, alpha, d, theta)
    ca = cos(alpha); sa = sin(alpha);
    ct = cos(theta); st = sin(theta);

    A = [ct,    -st,      0,   a;
         st*ca, ct*ca,   -sa, -d*sa;
         st*sa, ct*sa,    ca,  d*ca;
         0,      0,       0,   1];
end
