clc;
clear;
close all;

%% ================= Settings =================
robot = importrobot("Sao arm.urdf");
robot.DataFormat = "row";
robot.Gravity = [0 0 -9.81];

eeName = "gripper_center_link";
elbowLinkName = "lower_arm_link";

pickSquare  = "A5";
placeSquare = "A6";

square = 0.05;
boardThickness = 0.01;
boardCenter = [0.30, 0, 0];
boardTopZ = boardThickness;

hp = 0.095;
pickHeight = boardTopZ + hp;

plotMargin = [0.20, 0.20, 0.08];

% Fixed base rotation: 180 degrees
baseFixed = 0;

% IMPORTANT:
% This defines which local end-effector axis is treated as the wrist/tool
% direction for measuring the absolute wrist angle in frame 0.
%
% If the blue line in the final figure does not align with your visual wrist
% direction, try one of these:
%   toolAxisLocal = [1;0;0];
%   toolAxisLocal = [0;1;0];
%   toolAxisLocal = [0;0;1];
%   toolAxisLocal = [-1;0;0];
%
% Start with local x-axis:
toolAxisLocal = [1; 0; 0];

% Scale for angle error inside IK.
% Position error is in metres, angle error is in radians.
gammaScale = 0.08;

%% ================= Initial Setup =================
q0 = homeConfiguration(robot);
nq = numel(q0);

if nq < 4
    error("The robot must have at least 4 movable joints.");
end

q0(1) = baseFixed;

disp("Movable joint order used by importrobot:");
disp(getMovableJointNames(robot));

P_A5 = chess2world(pickSquare, boardCenter, square, pickHeight);
P_A6 = chess2world(placeSquare, boardCenter, square, pickHeight);

disp("Target Cartesian positions:");
disp(table( ...
    ["A5"; "A6"], ...
    [P_A5(1); P_A6(1)], ...
    [P_A5(2); P_A6(2)], ...
    [P_A5(3); P_A6(3)], ...
    'VariableNames', {'Point','X_m','Y_m','Z_m'}));

%% ================= Step 1: Free IK with Base Fixed =================
% Here only q1 is fixed. Wrist angle is still free.
qA5Free = solveBaseFixedPositionIKMultiSeedURDF(robot, eeName, elbowLinkName, ...
    P_A5, q0, baseFixed, boardTopZ);

qA6Free = solveBaseFixedPositionIKMultiSeedURDF(robot, eeName, elbowLinkName, ...
    P_A6, qA5Free, baseFixed, boardTopZ);

gammaA5Free = wristAbsAngleFromURDF(robot, qA5Free, eeName, toolAxisLocal);
gammaA6Free = wristAbsAngleFromURDF(robot, qA6Free, eeName, toolAxisLocal);

gammaHold = max(gammaA5Free, gammaA6Free);

disp("Base-fixed free IK result:");
disp("qA5Free:");
disp(qA5Free);
disp("qA6Free:");
disp(qA6Free);

disp("Free URDF wrist absolute angle comparison:");
disp(table( ...
    ["A5 free"; "A6 free"; "selected hold"], ...
    [gammaA5Free; gammaA6Free; gammaHold], ...
    rad2deg([gammaA5Free; gammaA6Free; gammaHold]), ...
    'VariableNames', {'Pose','Gamma_rad','Gamma_deg'}));

%% ================= Step 2: Position + URDF Wrist Angle IK =================
% Now solve A5 and A6 again, but constrain the URDF-measured wrist angle.

qA5Hold = solveBaseFixedPositionGammaIKMultiSeedURDF(robot, eeName, elbowLinkName, ...
    P_A5, qA5Free, baseFixed, gammaHold, toolAxisLocal, gammaScale, boardTopZ);

qA6Hold = solveBaseFixedPositionGammaIKMultiSeedURDF(robot, eeName, elbowLinkName, ...
    P_A6, qA6Free, baseFixed, gammaHold, toolAxisLocal, gammaScale, boardTopZ);

posA5Hold = tform2trvec(getTransform(robot, qA5Hold, eeName));
posA6Hold = tform2trvec(getTransform(robot, qA6Hold, eeName));

errA5Hold = norm(posA5Hold - P_A5);
errA6Hold = norm(posA6Hold - P_A6);

gammaA5Hold = wristAbsAngleFromURDF(robot, qA5Hold, eeName, toolAxisLocal);
gammaA6Hold = wristAbsAngleFromURDF(robot, qA6Hold, eeName, toolAxisLocal);

disp("Final base-fixed and URDF-wrist-angle-constrained IK result:");
disp(table( ...
    ["A5"; "A6"], ...
    [errA5Hold; errA6Hold], ...
    [qA5Hold(1); qA6Hold(1)], ...
    rad2deg([qA5Hold(1); qA6Hold(1)]), ...
    [gammaA5Hold; gammaA6Hold], ...
    rad2deg([gammaA5Hold; gammaA6Hold]), ...
    [wrapToPiLocal(gammaA5Hold - gammaHold); wrapToPiLocal(gammaA6Hold - gammaHold)], ...
    'VariableNames', {'Pose','PositionError_m','q1_rad','q1_deg','Gamma_rad','Gamma_deg','GammaError_rad'}));

disp("qA5Hold:");
disp(qA5Hold);

disp("qA6Hold:");
disp(qA6Hold);

%% ================= Figure 1: Separate A5 and A6 =================
figure("Name","URDF A5 and A6 - Same True Wrist Angle","Color","w");

ax1 = subplot(1,2,1);
hold(ax1, "on");
drawChessboardSideZX(boardCenter, square, boardThickness);

show(robot, qA5Hold, ...
    "Frames","off", ...
    "Visuals","on", ...
    "Parent", ax1);

scatter3(P_A5(1), P_A5(2), P_A5(3), 80, "filled", ...
    "MarkerFaceColor",[0.0 0.45 0.9], ...
    "MarkerEdgeColor","k");

text(P_A5(1), P_A5(2), P_A5(3)+0.025, "A5", ...
    "FontWeight","bold", ...
    "HorizontalAlignment","center");

plotWristDirection(robot, qA5Hold, eeName, toolAxisLocal, 0.08, ax1);

formatStickFigureScene(ax1, boardCenter, square, plotMargin);
title(ax1, sprintf("A5, q_1=%.1f deg, \\gamma=%.1f deg", ...
    rad2deg(qA5Hold(1)), rad2deg(gammaA5Hold)));

ax2 = subplot(1,2,2);
hold(ax2, "on");
drawChessboardSideZX(boardCenter, square, boardThickness);

show(robot, qA6Hold, ...
    "Frames","off", ...
    "Visuals","on", ...
    "Parent", ax2);

scatter3(P_A6(1), P_A6(2), P_A6(3), 80, "filled", ...
    "MarkerFaceColor",[0.9 0.25 0.1], ...
    "MarkerEdgeColor","k");

text(P_A6(1), P_A6(2), P_A6(3)+0.025, "A6", ...
    "FontWeight","bold", ...
    "HorizontalAlignment","center");

plotWristDirection(robot, qA6Hold, eeName, toolAxisLocal, 0.08, ax2);

formatStickFigureScene(ax2, boardCenter, square, plotMargin);
title(ax2, sprintf("A6, q_1=%.1f deg, \\gamma=%.1f deg", ...
    rad2deg(qA6Hold(1)), rad2deg(gammaA6Hold)));

sgtitle(sprintf("Same URDF wrist absolute angle in frame 0: \\gamma = %.2f deg", ...
    rad2deg(gammaHold)));

exportgraphics(gcf, "Fig_URDF_A5_A6_same_true_wrist_angle.png", "Resolution", 300);

%% ================= Figure 2: Overlay =================
figure("Name","Overlay - Same True Wrist Angle","Color","w");
ax = axes;
hold(ax, "on");

drawChessboardSideZX(boardCenter, square, boardThickness);

show(robot, qA5Hold, ...
    "Frames","off", ...
    "Visuals","on", ...
    "Parent", ax);

show(robot, qA6Hold, ...
    "Frames","off", ...
    "Visuals","on", ...
    "PreservePlot", true, ...
    "Parent", ax);

scatter3(P_A5(1), P_A5(2), P_A5(3), 80, "filled", ...
    "MarkerFaceColor",[0.0 0.45 0.9], ...
    "MarkerEdgeColor","k");

scatter3(P_A6(1), P_A6(2), P_A6(3), 80, "filled", ...
    "MarkerFaceColor",[0.9 0.25 0.1], ...
    "MarkerEdgeColor","k");

text(P_A5(1), P_A5(2), P_A5(3)+0.025, "A5", ...
    "FontWeight","bold", ...
    "HorizontalAlignment","center");

text(P_A6(1), P_A6(2), P_A6(3)+0.025, "A6", ...
    "FontWeight","bold", ...
    "HorizontalAlignment","center");

plotWristDirection(robot, qA5Hold, eeName, toolAxisLocal, 0.08, ax);
plotWristDirection(robot, qA6Hold, eeName, toolAxisLocal, 0.08, ax);

formatStickFigureScene(ax, boardCenter, square, plotMargin);
title(ax, sprintf("Overlay, same URDF wrist angle \\gamma = %.2f deg", ...
    rad2deg(gammaHold)));

exportgraphics(gcf, "Fig_URDF_A5_A6_overlay_same_true_wrist_angle.png", "Resolution", 300);

%% =========================================================
%% Local Functions
%% =========================================================

function names = getMovableJointNames(robot)
    names = strings(0,1);

    for k = 1:numel(robot.Bodies)
        joint = robot.Bodies{k}.Joint;

        if ~strcmp(joint.Type, "fixed")
            names(end+1,1) = string(joint.Name); %#ok<AGROW>
        end
    end
end

function gamma = wristAbsAngleFromURDF(robot, q, eeName, toolAxisLocal)
    % True URDF-based wrist absolute angle in frame 0.
    %
    % This uses the actual end-effector transform from the URDF instead of
    % assuming gamma = q2 + q3 + q4.
    %
    % The angle is measured in the XZ plane:
    %
    %   gamma = atan2(tool_axis_z, tool_axis_x)

    T = getTransform(robot, q, eeName);
    R = T(1:3,1:3);

    toolAxisWorld = R * toolAxisLocal;
    toolAxisWorld = toolAxisWorld / norm(toolAxisWorld);

    gamma = atan2(toolAxisWorld(3), toolAxisWorld(1));
end

function pos = chess2world(squareName, boardCenter, square, zHeight)
    squareName = char(squareName);

    col = double(squareName(1)) - double('A') + 1;
    row = str2double(squareName(2));

    boardHalf = 4 * square;

    x = boardCenter(1) - boardHalf + (row - 0.5) * square;
    y = boardCenter(2) + boardHalf - (col - 0.5) * square;
    z = zHeight;

    pos = [x, y, z];
end

function qBest = solveBaseFixedPositionIKMultiSeedURDF(robot, eeName, elbowLinkName, targetPos, qSeed, baseFixed, boardTopZ)
    n = numel(qSeed);

    seedList = repmat(qSeed, 8, 1);

    if n >= 6
        seedList(1,1:6) = [baseFixed,  1.2, -1.2,  0.8,  0.0, 0];
        seedList(2,1:6) = [baseFixed,  1.6, -1.6,  1.0,  0.0, 0];
        seedList(3,1:6) = [baseFixed,  0.8, -0.8,  0.6,  0.0, 0];
        seedList(4,1:6) = [baseFixed,  1.2,  1.2, -0.8,  0.0, 0];
        seedList(5,1:6) = [baseFixed, -0.8,  1.2,  0.8,  0.0, 0];
        seedList(6,1:6) = [baseFixed,  0.4, -1.0,  1.0,  0.0, 0];
        seedList(7,1:6) = [baseFixed,  1.0, -0.4,  0.4,  0.0, 0];
        seedList(8,1:6) = qSeed(1:6);
        seedList(8,1) = baseFixed;
    end

    bestCost = inf;
    qBest = qSeed;
    qBest(1) = baseFixed;

    for k = 1:size(seedList,1)
        q0 = seedList(k,:);
        q0(1) = baseFixed;

        [qCand, err] = ikPositionBaseFixedURDF(robot, eeName, targetPos, q0, baseFixed);

        qCand(1) = baseFixed;

        eePos = tform2trvec(getTransform(robot, qCand, eeName));
        elbowPos = tform2trvec(getTransform(robot, qCand, elbowLinkName));

        posErr = norm(eePos - targetPos);
        continuityCost = norm(wrapToPiLocal(qCand - qSeed));
        limitCost = jointLimitSoftCost(robot, qCand);
        belowBoardPenalty = max(0, boardTopZ - elbowPos(3));

        cost = 700.0*posErr + ...
               0.2*continuityCost + ...
               5.0*limitCost + ...
               20.0*belowBoardPenalty - ...
               elbowPos(3);

        if err < inf && cost < bestCost
            bestCost = cost;
            qBest = qCand;
        end
    end

    finalErr = norm(tform2trvec(getTransform(robot, qBest, eeName)) - targetPos);

    if finalErr > 0.015
        warning("Base-fixed position IK residual is %.4f m.", finalErr);
    end
end

function [q, errNorm] = ikPositionBaseFixedURDF(robot, eeName, targetPos, q0, baseFixed)
    q = q0;
    q(1) = baseFixed;

    n = numel(q);

    activeIdx = 1:n;
    activeIdx(activeIdx == 1) = [];

    lambda = 0.02;
    tol = 1e-5;
    maxIter = 450;
    maxStep = 0.08;

    errNorm = inf;

    for iter = 1:maxIter
        q(1) = baseFixed;

        eePos = tform2trvec(getTransform(robot, q, eeName));
        e = (targetPos - eePos).';
        errNorm = norm(e);

        if errNorm < tol
            break;
        end

        J = numericalTaskJacobian(robot, eeName, q, activeIdx, baseFixed, ...
            [], [], [], false);

        dq = J' * ((J*J' + lambda^2 * eye(size(J,1))) \ e);
        dq = max(min(dq, maxStep), -maxStep);

        q(activeIdx) = q(activeIdx) + dq.';
        q = wrapToPiLocal(q);
        q(1) = baseFixed;
    end
end

function qBest = solveBaseFixedPositionGammaIKMultiSeedURDF(robot, eeName, elbowLinkName, targetPos, qSeed, baseFixed, gammaTarget, toolAxisLocal, gammaScale, boardTopZ)
    n = numel(qSeed);

    seedList = repmat(qSeed, 10, 1);

    if n >= 6
        seedList(1,1:6)  = [baseFixed,  1.2, -1.2,  0.8,  0.0, 0];
        seedList(2,1:6)  = [baseFixed,  1.6, -1.6,  1.0,  0.0, 0];
        seedList(3,1:6)  = [baseFixed,  0.8, -0.8,  0.6,  0.0, 0];
        seedList(4,1:6)  = [baseFixed,  1.2,  1.2, -0.8,  0.0, 0];
        seedList(5,1:6)  = [baseFixed, -0.8,  1.2,  0.8,  0.0, 0];
        seedList(6,1:6)  = [baseFixed,  0.4, -1.0,  1.0,  0.0, 0];
        seedList(7,1:6)  = [baseFixed,  1.0, -0.4,  0.4,  0.0, 0];
        seedList(8,1:6)  = [baseFixed,  0.6, -1.4,  1.2,  0.0, 0];
        seedList(9,1:6)  = [baseFixed,  1.4, -0.6,  0.2,  0.0, 0];
        seedList(10,1:6) = qSeed(1:6);
        seedList(10,1) = baseFixed;
    end

    bestCost = inf;
    qBest = qSeed;
    qBest(1) = baseFixed;

    for k = 1:size(seedList,1)
        q0 = seedList(k,:);
        q0(1) = baseFixed;

        [qCand, posErr, gammaErr] = ikPositionGammaBaseFixedURDF(robot, eeName, ...
            targetPos, q0, baseFixed, gammaTarget, toolAxisLocal, gammaScale);

        qCand(1) = baseFixed;

        elbowPos = tform2trvec(getTransform(robot, qCand, elbowLinkName));

        continuityCost = norm(wrapToPiLocal(qCand - qSeed));
        limitCost = jointLimitSoftCost(robot, qCand);
        belowBoardPenalty = max(0, boardTopZ - elbowPos(3));

        cost = 900.0*posErr + ...
               350.0*abs(gammaErr) + ...
               0.2*continuityCost + ...
               5.0*limitCost + ...
               20.0*belowBoardPenalty - ...
               elbowPos(3);

        if cost < bestCost
            bestCost = cost;
            qBest = qCand;
        end
    end

    finalPosErr = norm(tform2trvec(getTransform(robot, qBest, eeName)) - targetPos);
    finalGammaErr = abs(wrapToPiLocal(wristAbsAngleFromURDF(robot, qBest, eeName, toolAxisLocal) - gammaTarget));

    if finalPosErr > 0.015
        warning("Position residual is %.4f m.", finalPosErr);
    end

    if finalGammaErr > deg2rad(1.0)
        warning("URDF wrist angle residual is %.2f deg.", rad2deg(finalGammaErr));
    end
end

function [q, posErr, gammaErr] = ikPositionGammaBaseFixedURDF(robot, eeName, targetPos, q0, baseFixed, gammaTarget, toolAxisLocal, gammaScale)
    q = q0;
    q(1) = baseFixed;

    n = numel(q);

    activeIdx = 1:n;
    activeIdx(activeIdx == 1) = [];

    lambda = 0.02;
    tolPos = 1e-5;
    tolGamma = deg2rad(0.05);
    maxIter = 600;
    maxStep = 0.06;

    posErr = inf;
    gammaErr = inf;

    for iter = 1:maxIter
        q(1) = baseFixed;

        eePos = tform2trvec(getTransform(robot, q, eeName));
        gammaNow = wristAbsAngleFromURDF(robot, q, eeName, toolAxisLocal);

        ePos = (targetPos - eePos).';
        eGamma = wrapToPiLocal(gammaTarget - gammaNow);

        posErr = norm(ePos);
        gammaErr = eGamma;

        if posErr < tolPos && abs(gammaErr) < tolGamma
            break;
        end

        e = [ePos; gammaScale * eGamma];

        J = numericalTaskJacobian(robot, eeName, q, activeIdx, baseFixed, ...
            gammaTarget, toolAxisLocal, gammaScale, true);

        dq = J' * ((J*J' + lambda^2 * eye(size(J,1))) \ e);
        dq = max(min(dq, maxStep), -maxStep);

        q(activeIdx) = q(activeIdx) + dq.';
        q = wrapToPiLocal(q);
        q(1) = baseFixed;
    end
end

function J = numericalTaskJacobian(robot, eeName, q, activeIdx, baseFixed, gammaTarget, toolAxisLocal, gammaScale, includeGamma)
    epsVal = 1e-6;

    qBase = q;
    qBase(1) = baseFixed;

    p0 = tform2trvec(getTransform(robot, qBase, eeName)).';

    if includeGamma
        gamma0 = wristAbsAngleFromURDF(robot, qBase, eeName, toolAxisLocal);
        h0 = [p0; gammaScale * gamma0];
        J = zeros(4, numel(activeIdx));
    else
        h0 = p0;
        J = zeros(3, numel(activeIdx));
    end

    for j = 1:numel(activeIdx)
        idx = activeIdx(j);

        qp = qBase;
        qp(idx) = qp(idx) + epsVal;
        qp(1) = baseFixed;

        pp = tform2trvec(getTransform(robot, qp, eeName)).';

        if includeGamma
            gp = wristAbsAngleFromURDF(robot, qp, eeName, toolAxisLocal);

            dg = wrapToPiLocal(gp - gamma0);

            hp = [pp; gammaScale * (gamma0 + dg)];
            J(:,j) = (hp - h0) / epsVal;
        else
            J(:,j) = (pp - h0) / epsVal;
        end
    end
end

function cost = jointLimitSoftCost(robot, q)
    cost = 0;
    jointIndex = 0;

    for k = 1:numel(robot.Bodies)
        joint = robot.Bodies{k}.Joint;

        if strcmp(joint.Type, "fixed")
            continue;
        end

        jointIndex = jointIndex + 1;
        lim = joint.PositionLimits;

        if any(isnan(lim)) || any(isinf(lim)) || lim(1) >= lim(2)
            continue;
        end

        normalized = abs(q(jointIndex) - mean(lim)) / (0.5 * (lim(2) - lim(1)));
        cost = cost + max(0, normalized - 0.85)^2;
    end
end

function a = wrapToPiLocal(a)
    a = atan2(sin(a), cos(a));
end

function plotWristDirection(robot, q, eeName, toolAxisLocal, lengthScale, ax)
    T = getTransform(robot, q, eeName);
    p = T(1:3,4);
    R = T(1:3,1:3);

    dir = R * toolAxisLocal;
    dir = dir / norm(dir);

    p2 = p + lengthScale * dir;

    plot3(ax, [p(1) p2(1)], [p(2) p2(2)], [p(3) p2(3)], ...
        "b-", "LineWidth", 3);
end

function drawChessboardSideZX(boardCenter, square, thickness)
    boardHalf = 4 * square;
    x1 = boardCenter(1) - boardHalf;
    x2 = boardCenter(1) + boardHalf;
    y0 = boardCenter(2);

    patch([x1 x2 x2 x1], ...
          [y0 y0 y0 y0], ...
          [0 0 thickness thickness], ...
          [0.70 0.55 0.35], ...
          "FaceAlpha", 0.35, ...
          "EdgeColor", [0.30 0.20 0.10], ...
          "LineWidth", 1.0);

    plot3([x1 x2], [y0 y0], [thickness thickness], ...
        "k-", "LineWidth", 1.2);
end

function formatStickFigureScene(ax, boardCenter, square, margin)
    boardHalf = 4 * square;

    axis(ax, "equal");

    xlim(ax, [boardCenter(1) - boardHalf - margin(1), ...
              boardCenter(1) + boardHalf + margin(1)]);

    ylim(ax, [boardCenter(2) - boardHalf - margin(2), ...
              boardCenter(2) + boardHalf + margin(2)]);

    zlim(ax, [0, 0.45 + margin(3)]);

    grid(ax, "on");
    xlabel(ax, "X (m)");
    ylabel(ax, "Y (m)");
    zlabel(ax, "Z (m)");

    % X right, Z up.
    view(ax, 0, 0);
    camproj(ax, "orthographic");
end