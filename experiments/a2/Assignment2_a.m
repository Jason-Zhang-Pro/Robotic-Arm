% =========================================================
% File: Assignment2.m
% Purpose:
%   Assignment 2 (Section 2.2) gravity torque evaluation with:
%   1) full workspace scan
%   2) farthest-chess-piece pick trajectory (linked to A1 board setup)
%
% DH convention:
%   [a_{i-1}, alpha_{i-1}, d_i, theta_i]
%   A_i = Rx(alpha_{i-1}) * Tx(a_{i-1}) * Rz(theta_i) * Tz(d_i)
% =========================================================
clc; clear; close all;

%% -----------------------------
% Robot geometry (SI units)
%% -----------------------------
b  = -0.025;
L1 = 0.05;
L2 = 0.22;
L3 = 0.18;
L4 = 0.10;
N  = 5;

%% -----------------------------
% Link masses and COM (kg, m)
% joints 1-4: STS3215 (55 g)
% joint 5 + gripper motor: STS3045M (35 g)
% link4 COM uses local +y4 direction as requested
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
tau_nominal = [tau_STS3215; tau_STS3215; tau_STS3215; tau_STS3215; tau_STS3045M];

%% -----------------------------
% Gravity vector (base frame)
%% -----------------------------
g0 = [0; 0; -9.81];

%% =========================================================
% Part A: Full workspace scan (A2 requirement)
%% =========================================================
q1Range = deg2rad(linspace(-180, 180, 13));
q2Range = deg2rad(linspace(-90,   90,  9));
q3Range = deg2rad(linspace(-120, 120,  9));
q4Range = deg2rad(linspace(-120, 120,  9));
q5Range = deg2rad(linspace(-90,   90,  7));

numSamples = numel(q1Range)*numel(q2Range)*numel(q3Range)*numel(q4Range)*numel(q5Range);
fprintf('Total samples: %d\n', numSamples);

maxAbsTau_ws = zeros(N,1);
qAtMax_ws = zeros(N,N);

count = 0;
t0 = tic;
for q1 = q1Range
for q2 = q2Range
for q3 = q3Range
for q4 = q4Range
for q5 = q5Range

    q = [q1;q2;q3;q4;q5];
    tau_g = gravityTorqueAtQ(q, b, L1, L2, L3, L4, m, pc_local, m_payload, g0);

    for j = 1:N
        if abs(tau_g(j)) > maxAbsTau_ws(j)
            maxAbsTau_ws(j) = abs(tau_g(j));
            qAtMax_ws(:,j) = q;
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
t_ws = toc(t0);

margin_ws = tau_nominal - maxAbsTau_ws;

%% =========================================================
% Part B: Farthest-piece pick-and-place task trajectory
% (aligned with robot_chess_pick_place_sim.m board/task flow)
%% =========================================================
board_x0   = 0.08;
board_y0   = 0.00;
board_cell = 0.05;
board_n    = 8;
z_board    = 0.00;
z_lift     = 0.08;

% Pick: farthest chess square center from base
[P_pick, ~] = farthestBoardSquare(board_x0, board_y0, board_cell, board_n);
% Place: keep the simulation target from robot_chess_pick_place_sim
P_place = [0.305; 0.175; z_board];

P_pick_up  = [P_pick(1);  P_pick(2);  z_lift];
P_place_up = [P_place(1); P_place(2); z_lift];

q_home = deg2rad([0;0;0;0;0]);
q_pick_up   = inverseKinematicsPosition(P_pick_up,  q_home,    b, L1, L2, L3, L4);
q_pick      = inverseKinematicsPosition(P_pick,     q_pick_up, b, L1, L2, L3, L4);
q_place_up  = inverseKinematicsPosition(P_place_up, q_pick,    b, L1, L2, L3, L4);
q_place     = inverseKinematicsPosition(P_place,    q_place_up,b, L1, L2, L3, L4);

q_pick_up  = nearestEquivalentToReference(q_pick_up,  q_home);
q_pick     = nearestEquivalentToReference(q_pick,     q_pick_up);
q_place_up = nearestEquivalentToReference(q_place_up, q_pick);
q_place    = nearestEquivalentToReference(q_place,    q_place_up);

% Same segment style as robot_chess_pick_place_sim
N1 = 40;   % home -> pick_up
N2 = 25;   % pick_up -> pick
N3 = 20;   % close pause
N4 = 25;   % pick -> pick_up
N5 = 45;   % pick_up -> place_up
N6 = 25;   % place_up -> place
N7 = 20;   % open pause
N8 = 25;   % place -> place_up

Q1 = jointTrajectory(q_home,     q_pick_up,  N1);
Q2 = jointTrajectory(q_pick_up,  q_pick,     N2);
Q3 = repmat(q_pick, 1, N3);
Q4 = jointTrajectory(q_pick,     q_pick_up,  N4);
Q5 = jointTrajectory(q_pick_up,  q_place_up, N5);
Q6 = jointTrajectory(q_place_up, q_place,    N6);
Q7 = repmat(q_place, 1, N7);
Q8 = jointTrajectory(q_place,    q_place_up, N8);

Q_task = [Q1 Q2 Q3 Q4 Q5 Q6 Q7 Q8];
Q_task = unwrapJointTrajectory(Q_task);

tau_task = zeros(N, size(Q_task,2));
for k = 1:size(Q_task,2)
    tau_task(:,k) = gravityTorqueAtQ(Q_task(:,k), b, L1, L2, L3, L4, m, pc_local, m_payload, g0);
end
maxAbsTau_task = max(abs(tau_task), [], 2);
margin_task = tau_nominal - maxAbsTau_task;

%% -----------------------------
% Console summary
%% -----------------------------
fprintf('\n========== 2.2 Gravity Torque Results ==========\n');
fprintf('Processed (workspace): %d / %d (%.2f s)\n', count, numSamples, t_ws);
fprintf('Farthest pick square center: [%.3f, %.3f, %.3f] m\n', P_pick(1), P_pick(2), P_pick(3));
fprintf('Task trajectory samples: %d\n', size(Q_task,2));

fprintf('\n[Workspace max]\n');
for j = 1:N
    fprintf('Joint %d: max|tau_g| = %.4f N*m at q = [%.1f %.1f %.1f %.1f %.1f] deg\n', ...
        j, maxAbsTau_ws(j), ...
        rad2deg(qAtMax_ws(1,j)), rad2deg(qAtMax_ws(2,j)), rad2deg(qAtMax_ws(3,j)), ...
        rad2deg(qAtMax_ws(4,j)), rad2deg(qAtMax_ws(5,j)));
end

fprintf('\n[Farthest-piece pick-and-place trajectory max]\n');
for j = 1:N
    fprintf('Joint %d: max|tau_g| = %.4f N*m, tau_nom = %.4f, margin = %+0.4f N*m\n', ...
        j, maxAbsTau_task(j), tau_nominal(j), margin_task(j));
end

if any(margin_ws < 0)
    fprintf('\nWARNING: Workspace scan exceeds nominal torque on some joints.\n');
else
    fprintf('\nPASS: Workspace scan is within nominal torque.\n');
end

if any(margin_task < 0)
    fprintf('WARNING: Farthest-piece task trajectory exceeds nominal torque on some joints.\n');
else
    fprintf('PASS: Farthest-piece task trajectory is within nominal torque.\n');
end

%% -----------------------------
% Save summary tables
%% -----------------------------
T_ws = table((1:N)', maxAbsTau_ws, tau_nominal, margin_ws, ...
    'VariableNames', {'Joint','TauRequiredMax_WS_Nm','TauNominal_Nm','Margin_WS_Nm'});
writetable(T_ws, 'Section22_torque_summary_workspace.csv');

T_task = table((1:N)', maxAbsTau_task, tau_nominal, margin_task, ...
    'VariableNames', {'Joint','TauRequiredMax_FarthestTask_Nm','TauNominal_Nm','Margin_FarthestTask_Nm'});
writetable(T_task, 'Section22_torque_summary_farthest_task.csv');

%% -----------------------------
% Plot report figure
%% -----------------------------
fig = figure('Color','w','Position',[90 90 1300 820]);

% 1) Board top view with farthest pick + place
subplot(2,2,1); hold on; axis equal; grid on;
drawBoardTop(board_x0, board_y0, board_cell, board_n);
plot(0, 0, 'ko', 'MarkerFaceColor','k', 'MarkerSize', 6);
plot(P_pick(1), P_pick(2), 'rp', 'MarkerSize', 12, 'MarkerFaceColor','r');
plot(P_place(1), P_place(2), 'bo', 'MarkerSize', 8, 'MarkerFaceColor','b');
text(P_pick(1)+0.01, P_pick(2)+0.01, 'Farthest pick', 'FontSize', 10);
text(P_place(1)+0.01, P_place(2)+0.01, 'Place', 'FontSize', 10);
xlabel('X (m)'); ylabel('Y (m)');
title('Chessboard, Farthest Pick, and Place');

% 2) Joint trajectory
subplot(2,2,2); hold on; grid on;
plot(rad2deg(Q_task(1,:)), 'LineWidth', 1.2);
plot(rad2deg(Q_task(2,:)), 'LineWidth', 1.2);
plot(rad2deg(Q_task(3,:)), 'LineWidth', 1.2);
plot(rad2deg(Q_task(4,:)), 'LineWidth', 1.2);
plot(rad2deg(Q_task(5,:)), 'LineWidth', 1.2);
xlabel('Frame'); ylabel('Joint angle (deg)');
title('Farthest-Piece Pick-and-Place Trajectory');
legend('q1','q2','q3','q4','q5','Location','best');

% 3) Torque profile on trajectory
subplot(2,2,3); hold on; grid on;
for j = 1:N
    plot(abs(tau_task(j,:)), 'LineWidth', 1.2);
    yline(tau_nominal(j), '--', sprintf('J%d nom',j), 'LineWidth', 0.8);
end
xlabel('Frame'); ylabel('|tau_g| (N*m)');
title('Gravity Torque Along Farthest-Piece Task Trajectory');
legend('J1','J2','J3','J4','J5','Location','best');

% 4) Bar compare
subplot(2,2,4); hold on; grid on;
bar(1:N, [tau_nominal, maxAbsTau_ws, maxAbsTau_task], 0.88);
xlabel('Joint'); ylabel('Torque (N*m)');
title('Nominal vs Required');
legend('Nominal','Workspace max','Farthest task max','Location','best');

sgtitle('Section 2.2: Workspace + Farthest-Piece Pick-and-Place Torque Evaluation');
saveas(fig, 'Section22_farthest_piece_task_evaluation.png');

%% =========================================================
% Local functions
%% =========================================================
function tau_g = gravityTorqueAtQ(q, b, L1, L2, L3, L4, m, pc_local, m_payload, g0)
    N = 5;
    [T_all, o, z] = fk_all(q, b, L1, L2, L3, L4); % frames 1..5
    oE = o(:,5);
    tau_g = zeros(N,1);

    % link self-weight
    for i = 1:N
        Ri = T_all(1:3,1:3,i);
        p_ci = Ri * pc_local(:,i) + o(:,i);

        Jv_ci = jacobian_point_on_link(i, p_ci, o, z);
        tau_g = tau_g + Jv_ci' * (m(i) * g0);
    end

    % payload at end-effector origin
    Jv_E = jacobian_point_general(oE, o, z);
    tau_g = tau_g + Jv_E' * (m_payload * g0);
end

function [P_far, P_all] = farthestBoardSquare(board_x0, board_y0, board_cell, board_n)
    xCenters = board_x0 + board_cell/2 + (0:board_n-1)*board_cell;
    yStart = board_y0 - board_n*board_cell/2;
    yCenters = yStart + board_cell/2 + (0:board_n-1)*board_cell;

    P_all = zeros(3, board_n*board_n);
    k = 1;
    for ix = 1:board_n
        for iy = 1:board_n
            P_all(:,k) = [xCenters(ix); yCenters(iy); 0];
            k = k + 1;
        end
    end

    dist = sqrt(P_all(1,:).^2 + P_all(2,:).^2);
    [~, idx] = max(dist);
    P_far = P_all(:,idx);
end

function q = inverseKinematicsPosition(pE, q_ref, b, L1, L2, L3, L4)
    q = q_ref;
    q(5) = 0;

    max_iters = 220;
    tol = 1e-4;
    lambda = 1e-2;
    step_limit = deg2rad(12);
    eps_val = 1e-6;

    for iter = 1:max_iters
        [~, o, ~] = fk_all(q, b, L1, L2, L3, L4);
        p_now = o(:,5);
        err = pE - p_now;

        if norm(err) < tol
            break;
        end

        J = zeros(3,4);
        for j = 1:4
            q_test = q;
            q_test(j) = q_test(j) + eps_val;
            [~, o_test, ~] = fk_all(q_test, b, L1, L2, L3, L4);
            p_test = o_test(:,5);
            J(:,j) = (p_test - p_now) / eps_val;
        end

        dq = J' * ((J*J' + lambda^2*eye(3)) \ err);
        dq = max(min(dq, step_limit), -step_limit);

        q(1:4) = q(1:4) + dq;
        q = nearestEquivalentToReference(q, q_ref);
        q(5) = 0;
    end

    [~, o_end, ~] = fk_all(q, b, L1, L2, L3, L4);
    final_err = norm(pE - o_end(:,5));
    if final_err > 5e-3
        error('IK did not converge to target. Final position error = %.6f m', final_err);
    end
end

function [T_all, o, z] = fk_all(q, b, L1, L2, L3, L4)
    A1 = dhT(0,   0,      L1, q(1));
    A2 = dhT(-b,  pi/2,   0,  q(2) + pi/2);
    A3 = dhT(L2,  0,      0,  q(3));
    A4 = dhT(L3,  0,      0,  q(4) - pi/2);
    A5 = dhT(0,  -pi/2,   L4, q(5));

    A = cat(3,A1,A2,A3,A4,A5);

    T = eye(4);
    T_all = zeros(4,4,5);
    o = zeros(3,5);
    z = zeros(3,5);

    for j = 1:5
        T = T * A(:,:,j);
        T_all(:,:,j) = T;
        o(:,j) = T(1:3,4);
        z(:,j) = T(1:3,3);
    end
end

function Jv = jacobian_point_on_link(i, p, o, z)
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
         st*ca,  ct*ca,  -sa, -d*sa;
         st*sa,  ct*sa,   ca,  d*ca;
         0,      0,       0,   1];
end

function q_adj = nearestEquivalentToReference(q, q_ref)
    q_adj = q;
    for i = 1:numel(q)
        delta = q_adj(i) - q_ref(i);
        q_adj(i) = q_adj(i) - 2*pi*round(delta/(2*pi));
    end
end

function Q_unwrapped = unwrapJointTrajectory(Q)
    Q_unwrapped = Q;
    for k = 2:size(Q,2)
        for j = 1:size(Q,1)
            delta = Q_unwrapped(j,k) - Q_unwrapped(j,k-1);
            Q_unwrapped(j,k) = Q_unwrapped(j,k) - 2*pi*round(delta/(2*pi));
        end
    end
end

function Q = jointTrajectory(q_start, q_end, Npts)
    Q = zeros(numel(q_start), Npts);
    for i = 1:Npts
        t = (i-1)/(Npts-1);
        s = 3*t^2 - 2*t^3;
        Q(:,i) = (1-s)*q_start + s*q_end;
    end
end

function drawBoardTop(board_x0, board_y0, board_cell, board_n)
    yStart = board_y0 - board_n*board_cell/2;
    for i = 0:board_n-1
        for j = 0:board_n-1
            x1 = board_x0 + i*board_cell;
            y1 = yStart + j*board_cell;
            X = [x1, x1+board_cell, x1+board_cell, x1];
            Y = [y1, y1, y1+board_cell, y1+board_cell];
            if mod(i+j,2)==0
                c = [0.92 0.92 0.92];
            else
                c = [0.25 0.25 0.25];
            end
            patch(X, Y, c, 'EdgeColor', [0.1 0.1 0.1]);
        end
    end
end
