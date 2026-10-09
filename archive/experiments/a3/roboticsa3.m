%% Generate figures for Assignment 3 pages 7-10
% MATLAB R2022b compatible.
% This script creates:
%   Fig1_xyz_vs_time.png
%   Fig2_xy_projection.png
%   Fig3_zx_projection.png
%   Fig4_robot_poses_D1_to_H8_jacobian_IK.png
%
% Units are metres and radians.

clear; close all; clc;

%% Output folder
outDir = pwd;

%% Robot dimensions from Assignment 1
b  = -0.025;   % 2.5 cm
L1 = 0.050;   % 5 cm
L2 = 0.220;   % 22 cm
L3 = 0.180;   % 18 cm
L4 = 0.100;   % 10 cm

%% Chessboard and task definition from Assignment 3 report
a = 0.050;       % square size, 0.40/8 = 0.05 m
boardEdgeOffset = 0.100;  % chessboard front edge is 10 cm from base
zb = 0.00;        % board height
hp = 0.095;      % chess piece height

% Keep original value from your code.
zclr = 1.05*hp + hp;

% File index: A=1, B=2, C=3, D=4, ..., H=8
fileA = 1;
fileB = 2;
fileC = 3;
fileD = 4;
fileH = 8;

% Square centre convention used in the report:
% x(r) = 0.10 + (r - 1/2)a, y(f) = (f - 4.5)a
P0 = squareCentre(5, fileA, a, boardEdgeOffset, zb + hp);  % A5
P3 = squareCentre(6, fileC, a, boardEdgeOffset, zb + hp);  % C6
P1 = [P0(1); P0(2); zclr];                  % via point above A5
P2 = [P3(1); P3(2); zclr];                  % via point above C6

% Obstacles for plotting: rook at A6 and pawn at B6
rookPos = squareCentre(6, fileA, a, boardEdgeOffset, zb);
pawnPos = squareCentre(6, fileB, a, boardEdgeOffset, zb);

waypoints = [P0 P1 P2 P3];
Tseg = [1.0 1.0 1.0];
tf = sum(Tseg);

% Separate trajectory used only for the final feasibility demonstration.
% D1 to H8 feasibility move.
P0_feas = squareCentre(1, fileD, a, boardEdgeOffset, zb + hp); % D1
P3_feas = squareCentre(8, fileH, a, boardEdgeOffset, zb + hp); % H8
P1_feas = [P0_feas(1); P0_feas(2); zclr];
P2_feas = [P3_feas(1); P3_feas(2); zclr];
waypointsFeas = [P0_feas P1_feas P2_feas P3_feas];

%% Cubic coefficients for the three translational segments
% p_s(tau) = c0 + c1 tau + c2 tau^2 + c3 tau^3
% Zero velocity is used at both ends of every segment.
C = zeros(3, 4, 3);  % segment, coefficient, xyz
for s = 1:3
    Cs = cubicCoefficients(waypoints(:,s), waypoints(:,s+1), Tseg(s));
    C(s,:,:) = reshape(Cs.', 1, 4, 3);
end

disp('Cubic coefficient matrices Cx, Cy, Cz. Rows are segments; columns are [a0 a1 a2 a3].');
Cx = squeeze(C(:,:,1))
Cy = squeeze(C(:,:,2))
Cz = squeeze(C(:,:,3))

%% Sample the trajectory
N = 400;
t = linspace(0, tf, N);
pd = zeros(3, N);
for k = 1:N
    pd(:,k) = evalPiecewiseTrajectory(t(k), C, Tseg);
end

viaTimes = [0 cumsum(Tseg)];

%% Plot 1: reference x, y, z position vs time
figure('Color','w');
plot(t, pd(1,:), 'LineWidth', 1.8); hold on;
plot(t, -pd(2,:), 'LineWidth', 1.8);
plot(t, pd(3,:), 'LineWidth', 1.8);

plot(viaTimes, waypoints(1,:), 'ko', 'MarkerFaceColor','w', 'LineWidth',1.2);
plot(viaTimes, waypoints(2,:), 'ks', 'MarkerFaceColor','w', 'LineWidth',1.2);
plot(viaTimes, waypoints(3,:), 'kd', 'MarkerFaceColor','w', 'LineWidth',1.2);

xline(Tseg(1), '--', 'Via 1');
xline(Tseg(1)+Tseg(2), '--', 'Via 2');

grid on;
xlabel('Time, t (s)');
ylabel('Position (m)');
title('Reference end-effector position for A5 to C6 move');
legend('x(t)','y(t)','z(t)','x via','y via','z via','Location','best');

exportgraphics(gcf, fullfile(outDir, 'Fig1_xyz_vs_time.png'), 'Resolution', 300);

%% Plot 2: XY projection
figure('Color','w');
plot(pd(1,:), pd(2,:), 'k-', 'LineWidth', 2.0); hold on;

plot(waypoints(1,:), waypoints(2,:), 'ro', 'MarkerFaceColor','w', 'LineWidth',1.4);
plot(P0(1), P0(2), 'go', 'MarkerFaceColor','g', 'MarkerSize', 8);
plot(P3(1), P3(2), 'bo', 'MarkerFaceColor','b', 'MarkerSize', 8);

plot(rookPos(1), rookPos(2), 'ms', 'MarkerFaceColor','m', 'MarkerSize', 8);
plot(pawnPos(1), pawnPos(2), 'cs', 'MarkerFaceColor','c', 'MarkerSize', 8);

text(P0(1), P0(2)-0.012, 'A5 start', 'HorizontalAlignment','center');
text(P3(1), P3(2)+0.012, 'C6 final', 'HorizontalAlignment','center');
text(rookPos(1), rookPos(2)+0.012, 'A6 rook', 'HorizontalAlignment','center');
text(pawnPos(1), pawnPos(2)+0.012, 'B6 pawn', 'HorizontalAlignment','center');

axis equal; grid on;
xlabel('x (m)');
ylabel('y (m)');
title('XY projection of end-effector trajectory');
legend('trajectory','via points','start','final','rook','pawn','Location','bestoutside');

exportgraphics(gcf, fullfile(outDir, 'Fig2_xy_projection.png'), 'Resolution', 300);

%% Plot 3: ZX projection
figure('Color','w');
plot(pd(1,:), pd(3,:), 'k-', 'LineWidth', 2.0); hold on;
plot(waypoints(1,:), waypoints(3,:), 'ro', 'MarkerFaceColor','w', 'LineWidth',1.4);

yline(zclr, '--r', 'z_{clr}', 'LineWidth', 1.2);

drawPieceZX(rookPos(1), zb, hp, 0.012, [0.8 0.2 0.8]);
drawPieceZX(pawnPos(1), zb, hp, 0.010, [0.2 0.7 0.9]);
drawPieceZX(P0(1), zb, hp, 0.010, [0.3 0.3 0.3]);

text(P0(1), hp+0.008, 'A5 knight', 'HorizontalAlignment','center');
text(rookPos(1), hp+0.018, 'A6 rook', 'HorizontalAlignment','center');
text(pawnPos(1), hp+0.008, 'B6 pawn', 'HorizontalAlignment','center');

axis equal; grid on;
xlabel('x (m)');
ylabel('z (m)');
title('ZX projection showing lifting clearance');
legend('trajectory','via points','clearance height','Location','best');

exportgraphics(gcf, fullfile(outDir, 'Fig3_zx_projection.png'), 'Resolution', 300);

%% Plot 4: Robot poses using Jacobian-based inverse kinematics
% Feasibility move: D1 to H8.
%
% New logic:
%   1. Solve pick pose normally.
%   2. Solve place pose normally.
%   3. Compute:
%          gamma_pick  = q2_pick  + q3_pick  + q4_pick
%          gamma_place = q2_place + q3_place + q4_place
%   4. Select:
%          gamma_hold = max(gamma_pick, gamma_place)
%   5. Only enforce gamma_hold at pick and place.
%      During raise / transport / lower motion, q4 frame-0 absolute angle
%      is allowed to change.

Cfeas = zeros(3, 4, 3);
for s = 1:3
    Cs = cubicCoefficients(waypointsFeas(:,s), waypointsFeas(:,s+1), Tseg(s));
    Cfeas(s,:,:) = reshape(Cs.', 1, 4, 3);
end

pdFeas = zeros(3, N);
for k = 1:N
    pdFeas(:,k) = evalPiecewiseTrajectory(t(k), Cfeas, Tseg);
end

sampleTimes = linspace(0, tf, 11);
qLog = zeros(5, numel(sampleTimes));
errLog = zeros(1, numel(sampleTimes));
gammaLog = zeros(1, numel(sampleTimes));

% Initial guess
q0 = [0; 0.45; -1.15; 0.70; 0];

% Solve pick pose normally.
[q_pick_free, err_pick_free] = ikJacobianPositionMultiSeed( ...
    waypointsFeas(:,1), q0, b, L1, L2, L3, L4, zb);

% Solve place pose normally.
[q_place_free, err_place_free] = ikJacobianPositionMultiSeed( ...
    waypointsFeas(:,4), q_pick_free, b, L1, L2, L3, L4, zb);

% Compute wrist absolute angles in frame 0.
gamma_pick_free  = q_pick_free(2)  + q_pick_free(3)  + q_pick_free(4);
gamma_place_free = q_place_free(2) + q_place_free(3) + q_place_free(4);

% Select the larger wrist absolute angle.
gamma_hold = max(gamma_pick_free, gamma_place_free);

disp('Free IK pick q:');
disp(q_pick_free);

disp('Free IK place q:');
disp(q_place_free);

disp('Free IK pick wrist absolute angle:');
disp(gamma_pick_free);

disp('Free IK place wrist absolute angle:');
disp(gamma_place_free);

disp('Selected gamma_hold = max(gamma_pick, gamma_place):');
disp(gamma_hold);

% Start from the pick solution.
q = q_pick_free;

figure('Color','w');
plot(pdFeas(1,:), pdFeas(3,:), 'k--', 'LineWidth', 1.5); hold on;
yline(zclr, '--r', 'z_{clr}', 'LineWidth', 1.0);

drawPieceZX(P0_feas(1), zb, hp, 0.010, [0.3 0.3 0.3]);
drawPieceZX(P3_feas(1), zb, hp, 0.010, [0.2 0.7 0.9]);
text(P0_feas(1), hp+0.010, 'D1 start', 'HorizontalAlignment','center');
text(P3_feas(1), hp+0.010, 'H8 final', 'HorizontalAlignment','center');

col = parula(numel(sampleTimes));

for k = 1:numel(sampleTimes)
    pk = evalPiecewiseTrajectory(sampleTimes(k), Cfeas, Tseg);

    if k == 1
        % Pick pose:
        % Enforce selected gamma_hold only at pick.
        [q, errNorm] = ikJacobianPositionHoldSelectedWristAngle( ...
            pk, q_pick_free, b, L1, L2, L3, L4, gamma_hold);

    elseif k == numel(sampleTimes)
        % Place pose:
        % Enforce the same gamma_hold at place.
        [q, errNorm] = ikJacobianPositionHoldSelectedWristAngle( ...
            pk, q, b, L1, L2, L3, L4, gamma_hold);

    else
        % Middle poses:
        % Use normal IK. q4 frame-0 absolute angle is allowed to change.
        [q, errNorm] = ikJacobianPosition(pk, q, b, L1, L2, L3, L4);
    end

    qLog(:,k) = q;
    errLog(k) = errNorm;
    gammaLog(k) = q(2) + q(3) + q(4);

    [~, jointPos] = fkStandardDH(q, b, L1, L2, L3, L4);

    plot(jointPos(1,:), jointPos(3,:), '-o', ...
        'Color', col(k,:), 'LineWidth', 1.2, ...
        'MarkerSize', 3, 'MarkerFaceColor', col(k,:));
end

axis equal; grid on;
xlabel('x (m)');
ylabel('z (m)');
title('Feasibility poses: D1 to H8, same wrist frame-0 angle only at pick and place');

exportgraphics(gcf, fullfile(outDir, 'Fig4_robot_poses_D1_to_H8_jacobian_IK.png'), 'Resolution', 300);

disp('Sample times t = k tf/10:');
disp(sampleTimes);

disp('Joint configurations q(t_k), columns correspond to sample times:');
disp(qLog);

disp('Final IK position residuals at sampled poses:');
disp(errLog);

disp('Wrist absolute angle in frame 0 at sampled poses:');
disp('gammaLog = q2 + q3 + q4');
disp(gammaLog);

disp('Selected held wrist angle gamma_hold:');
disp(gamma_hold);

disp('Pick gamma deviation from gamma_hold:');
disp(gammaLog(1) - gamma_hold);

disp('Place gamma deviation from gamma_hold:');
disp(gammaLog(end) - gamma_hold);

disp('q4 values at sampled poses:');
disp(qLog(4,:));

disp('q5 values at sampled poses:');
disp(qLog(5,:));

%% Local functions

function P = squareCentre(rank, fileIndex, a, boardEdgeOffset, z)
    x = boardEdgeOffset + (rank - 0.5)*a;
    y = (fileIndex - 4.5)*a;
    P = [x; y; z];
end

function Cs = cubicCoefficients(Pa, Pb, T)
    dP = Pb - Pa;
    c0 = Pa;
    c1 = zeros(3,1);
    c2 = 3*dP/(T^2);
    c3 = -2*dP/(T^3);
    Cs = [c0 c1 c2 c3];
end

function p = evalPiecewiseTrajectory(t, C, Tseg)
    if t <= Tseg(1)
        s = 1;
        tau = t;
    elseif t <= Tseg(1) + Tseg(2)
        s = 2;
        tau = t - Tseg(1);
    else
        s = 3;
        tau = t - Tseg(1) - Tseg(2);
    end

    tau = max(0, min(tau, Tseg(s)));

    p = zeros(3,1);
    for j = 1:3
        c = squeeze(C(s,:,j));
        p(j) = c(1) + c(2)*tau + c(3)*tau^2 + c(4)*tau^3;
    end
end

function [qBest, errBest] = ikJacobianPositionMultiSeed(pDes, qPrev, b, L1, L2, L3, L4, zBoard)
    baseAngle = atan2(pDes(2), pDes(1));

    seeds = [
        qPrev.';
        baseAngle, -0.55,  0.70, -1.20, 0;
        baseAngle, -0.75,  1.00, -1.45, 0;
        baseAngle, -0.35,  0.45, -1.10, 0;
        baseAngle,  0.35, -0.75,  1.10, 0;
        baseAngle,  0.55, -1.10,  1.35, 0;
        baseAngle, -0.80, -0.80, -1.20, 0;
        baseAngle, -1.10, -0.60, -1.40, 0;
        baseAngle, -1.30, -1.00, -1.60, 0
    ];

    bestCost = inf;
    qBest = qPrev;
    errBest = inf;

    for i = 1:size(seeds,1)
        [qCand, errCand] = ikJacobianPosition(pDes, seeds(i,:).', b, L1, L2, L3, L4);
        [~, jointPos] = fkStandardDH(qCand, b, L1, L2, L3, L4);

        belowBoard = max(0, zBoard - min(jointPos(3,2:5)));
        elbowLow = max(0, zBoard + 0.16 - min(jointPos(3,3:5)));
        elbowFoldBack = max(0, jointPos(1,3) - jointPos(1,4));

        toolVec = jointPos(:,6) - jointPos(:,5);
        wristFlip = max(0, toolVec(3));
        wristSide = norm(toolVec(1:2));

        elbowSign = max(0, qCand(3));
        wristSign = max(0, qCand(4));

        continuity = norm(wrapToPiLocal(qCand(1:4) - qPrev(1:4)));

        cost = 250.0*errCand + ...
               4.0*belowBoard + ...
               20.0*elbowLow + ...
               2.0*elbowFoldBack + ...
               12.0*elbowSign + ...
               12.0*wristSign + ...
               4.0*wristFlip + ...
               0.2*wristSide + ...
               0.02*continuity;

        if cost < bestCost
            bestCost = cost;
            qBest = qCand;
            errBest = errCand;
        end
    end
end

function [q, errNorm] = ikJacobianPosition(pDes, q0, b, L1, L2, L3, L4)
    q = q0;
    q(5) = 0;

    lambda = 0.02;
    tol = 1e-5;
    maxIter = 250;
    errNorm = inf;

    for iter = 1:maxIter
        [T, ~] = fkStandardDH(q, b, L1, L2, L3, L4);
        pNow = T(1:3,4);
        e = pDes - pNow;
        errNorm = norm(e);

        if errNorm < tol
            break;
        end

        J = numericalJacobianPosition(q, b, L1, L2, L3, L4);
        J4 = J(:,1:4);
        dq4 = J4' * ((J4*J4' + lambda^2*eye(3)) \ e);

        dq4 = max(min(dq4, 0.12), -0.12);

        q(1:4) = q(1:4) + dq4;
        q(5) = 0;
    end

    q(1:4) = wrapToPiLocal(q(1:4));
    q(5) = 0;
end

function [q, errNorm] = ikJacobianPositionHoldSelectedWristAngle(pDes, q0, b, L1, L2, L3, L4, gamma_hold)
    q = q0;
    q(4) = gamma_hold - q(2) - q(3);
    q(5) = 0;

    lambda = 0.02;
    tol = 1e-5;
    maxIter = 250;
    errNorm = inf;

    for iter = 1:maxIter
        q(4) = gamma_hold - q(2) - q(3);
        q(5) = 0;

        [T, ~] = fkStandardDH(q, b, L1, L2, L3, L4);
        pNow = T(1:3,4);

        e = pDes - pNow;
        errNorm = norm(e);

        if errNorm < tol
            break;
        end

        J3 = numericalJacobianHoldSelectedWristAngle(q, b, L1, L2, L3, L4, gamma_hold);

        dq3 = J3' * ((J3*J3' + lambda^2*eye(3)) \ e);
        dq3 = max(min(dq3, 0.12), -0.12);

        q(1:3) = q(1:3) + dq3;

        q(4) = gamma_hold - q(2) - q(3);
        q(5) = 0;
    end

    q(1:3) = wrapToPiLocal(q(1:3));
    q(4) = gamma_hold - q(2) - q(3);
    q(4) = wrapToPiLocal(q(4));
    q(5) = 0;
end

function J3 = numericalJacobianHoldSelectedWristAngle(q, b, L1, L2, L3, L4, gamma_hold)
    epsVal = 1e-6;

    qBase = q;
    qBase(4) = gamma_hold - qBase(2) - qBase(3);
    qBase(5) = 0;

    [T0, ~] = fkStandardDH(qBase, b, L1, L2, L3, L4);
    p0 = T0(1:3,4);

    J3 = zeros(3,3);

    for j = 1:3
        qp = qBase;
        qp(j) = qp(j) + epsVal;

        qp(4) = gamma_hold - qp(2) - qp(3);
        qp(5) = 0;

        [Tp, ~] = fkStandardDH(qp, b, L1, L2, L3, L4);
        pp = Tp(1:3,4);

        J3(:,j) = (pp - p0)/epsVal;
    end
end

function a = wrapToPiLocal(a)
    a = atan2(sin(a), cos(a));
end

function J = numericalJacobianPosition(q, b, L1, L2, L3, L4)
    epsVal = 1e-6;

    [T0, ~] = fkStandardDH(q, b, L1, L2, L3, L4);
    p0 = T0(1:3,4);

    J = zeros(3,5);

    for j = 1:5
        qp = q;
        qp(j) = qp(j) + epsVal;

        [Tp, ~] = fkStandardDH(qp, b, L1, L2, L3, L4);

        J(:,j) = (Tp(1:3,4) - p0)/epsVal;
    end
end

function [T, jointPos] = fkStandardDH(q, b, L1, L2, L3, L4)
    dh = [
        -b,  pi/2, L1, q(1);
         L2, 0,    0,  q(2) + pi/2;
         L3, 0,    0,  q(3);
         0, -pi/2, 0,  q(4) - pi/2;
         0,  0,    L4, q(5)
    ];

    T = eye(4);
    jointPos = zeros(3,6);
    jointPos(:,1) = T(1:3,4);

    for i = 1:5
        T = T * dhTransform(dh(i,1), dh(i,2), dh(i,3), dh(i,4));
        jointPos(:,i+1) = T(1:3,4);
    end
end

function A = dhTransform(a, alpha, d, theta)
    ct = cos(theta);
    st = sin(theta);
    ca = cos(alpha);
    sa = sin(alpha);

    A = [
        ct, -st*ca,  st*sa, a*ct;
        st,  ct*ca, -ct*sa, a*st;
        0,      sa,     ca,    d;
        0,       0,      0,    1
    ];
end

function drawPieceZX(x, z0, h, halfWidth, colour)
    X = [x-halfWidth, x+halfWidth, x+halfWidth, x-halfWidth];
    Z = [z0, z0, z0+h, z0+h];

    patch(X, Z, colour, ...
        'FaceAlpha', 0.28, ...
        'EdgeColor', colour, ...
        'LineWidth', 1.0);
end