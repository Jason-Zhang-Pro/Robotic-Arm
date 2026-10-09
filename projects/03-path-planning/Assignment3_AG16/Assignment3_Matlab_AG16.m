clc;
clear;
close all;

%% ================= Settings =================
robot = importrobot("Robotic Arm.urdf");
robot.DataFormat = "row";
robot.Gravity = [0 0 -9.81];

eeName = "gripper_center_link";
elbowLinkName = "lower_arm_link";

pickSquare  = "A5";
placeSquare = "C6";

square = 0.05;
boardThickness = 0.01;
boardCenter = [0.30, 0, 0];
boardTopZ = boardThickness;

hp = 0.095;
pickHeight = boardTopZ + hp;

plotMargin = [0.20, 0.20, 0.08];

% Fixed base rotation: 180 degrees
baseFixed = pi;

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

%% ================= Assignment Reference Figures =================
generateAssignmentReferenceFigures(pwd);

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
P_C6 = chess2world(placeSquare, boardCenter, square, pickHeight);

disp("Target Cartesian positions:");
disp(table( ...
    [pickSquare; placeSquare], ...
    [P_A5(1); P_C6(1)], ...
    [P_A5(2); P_C6(2)], ...
    [P_A5(3); P_C6(3)], ...
    'VariableNames', {'Point','X_m','Y_m','Z_m'}));

%% ================= Step 1: Free IK with Base Fixed =================
% Here only q1 is fixed. Wrist angle is still free.
qA5Free = solveBaseFixedPositionIKMultiSeedURDF(robot, eeName, elbowLinkName, ...
    P_A5, q0, baseFixed, boardTopZ);

qC6Free = solveBaseFixedPositionIKMultiSeedURDF(robot, eeName, elbowLinkName, ...
    P_C6, qA5Free, baseFixed, boardTopZ);

gammaA5Free = wristAbsAngleFromURDF(robot, qA5Free, eeName, toolAxisLocal);
gammaC6Free = wristAbsAngleFromURDF(robot, qC6Free, eeName, toolAxisLocal);

gammaHold = max(gammaA5Free, gammaC6Free);

disp("Base-fixed free IK result:");
disp("qA5Free:");
disp(qA5Free);
disp("qC6Free:");
disp(qC6Free);

disp("Free URDF wrist absolute angle comparison:");
disp(table( ...
    [pickSquare + " free"; placeSquare + " free"; "selected hold"], ...
    [gammaA5Free; gammaC6Free; gammaHold], ...
    rad2deg([gammaA5Free; gammaC6Free; gammaHold]), ...
    'VariableNames', {'Pose','Gamma_rad','Gamma_deg'}));

%% ================= Step 2: Position + URDF Wrist Angle IK =================
% Now solve A5 and C6 again, but constrain the URDF-measured wrist angle.

qA5Hold = solveBaseFixedPositionGammaIKMultiSeedURDF(robot, eeName, elbowLinkName, ...
    P_A5, qA5Free, baseFixed, gammaHold, toolAxisLocal, gammaScale, boardTopZ);

qC6Hold = solveBaseFixedPositionGammaIKMultiSeedURDF(robot, eeName, elbowLinkName, ...
    P_C6, qC6Free, baseFixed, gammaHold, toolAxisLocal, gammaScale, boardTopZ);

posA5Hold = tform2trvec(getTransform(robot, qA5Hold, eeName));
posC6Hold = tform2trvec(getTransform(robot, qC6Hold, eeName));

errA5Hold = norm(posA5Hold - P_A5);
errC6Hold = norm(posC6Hold - P_C6);

gammaA5Hold = wristAbsAngleFromURDF(robot, qA5Hold, eeName, toolAxisLocal);
gammaC6Hold = wristAbsAngleFromURDF(robot, qC6Hold, eeName, toolAxisLocal);

disp("Final base-fixed and URDF-wrist-angle-constrained IK result:");
disp(table( ...
    [pickSquare; placeSquare], ...
    [errA5Hold; errC6Hold], ...
    [qA5Hold(1); qC6Hold(1)], ...
    rad2deg([qA5Hold(1); qC6Hold(1)]), ...
    [gammaA5Hold; gammaC6Hold], ...
    rad2deg([gammaA5Hold; gammaC6Hold]), ...
    [wrapToPiLocal(gammaA5Hold - gammaHold); wrapToPiLocal(gammaC6Hold - gammaHold)], ...
    'VariableNames', {'Pose','PositionError_m','q1_rad','q1_deg','Gamma_rad','Gamma_deg','GammaError_rad'}));

disp("qA5Hold:");
disp(qA5Hold);

disp("qC6Hold:");
disp(qC6Hold);

%% ================= Figure 1: Separate A5 and C6 =================
figSeparate = figure("Name","URDF A5 and C6 - Same True Wrist Angle","Color","w");

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

show(robot, qC6Hold, ...
    "Frames","off", ...
    "Visuals","on", ...
    "Parent", ax2);

scatter3(P_C6(1), P_C6(2), P_C6(3), 80, "filled", ...
    "MarkerFaceColor",[0.9 0.25 0.1], ...
    "MarkerEdgeColor","k");

text(P_C6(1), P_C6(2), P_C6(3)+0.025, placeSquare, ...
    "FontWeight","bold", ...
    "HorizontalAlignment","center");

plotWristDirection(robot, qC6Hold, eeName, toolAxisLocal, 0.08, ax2);

formatStickFigureScene(ax2, boardCenter, square, plotMargin);
title(ax2, sprintf("%s, q_1=%.1f deg, \\gamma=%.1f deg", ...
    placeSquare, rad2deg(qC6Hold(1)), rad2deg(gammaC6Hold)));

sgtitle(sprintf("Same URDF wrist absolute angle in frame 0: \\gamma = %.2f deg", ...
    rad2deg(gammaHold)));

exportgraphics(figSeparate, "Fig_URDF_A5_C6_same_true_wrist_angle.png", "Resolution", 300);

%% ================= Figure 2: Overlay =================
figOverlay = figure("Name","Overlay - Same True Wrist Angle","Color","w");
ax = axes;
hold(ax, "on");

drawChessboardSideZX(boardCenter, square, boardThickness);

show(robot, qA5Hold, ...
    "Frames","off", ...
    "Visuals","on", ...
    "Parent", ax);

show(robot, qC6Hold, ...
    "Frames","off", ...
    "Visuals","on", ...
    "PreservePlot", true, ...
    "Parent", ax);

scatter3(P_A5(1), P_A5(2), P_A5(3), 80, "filled", ...
    "MarkerFaceColor",[0.0 0.45 0.9], ...
    "MarkerEdgeColor","k");

scatter3(P_C6(1), P_C6(2), P_C6(3), 80, "filled", ...
    "MarkerFaceColor",[0.9 0.25 0.1], ...
    "MarkerEdgeColor","k");

text(P_A5(1), P_A5(2), P_A5(3)+0.025, "A5", ...
    "FontWeight","bold", ...
    "HorizontalAlignment","center");

text(P_C6(1), P_C6(2), P_C6(3)+0.025, placeSquare, ...
    "FontWeight","bold", ...
    "HorizontalAlignment","center");

plotWristDirection(robot, qA5Hold, eeName, toolAxisLocal, 0.08, ax);
plotWristDirection(robot, qC6Hold, eeName, toolAxisLocal, 0.08, ax);

formatStickFigureScene(ax, boardCenter, square, plotMargin);
title(ax, sprintf("Overlay, same URDF wrist angle \\gamma = %.2f deg", ...
    rad2deg(gammaHold)));

exportgraphics(figOverlay, "Fig_URDF_A5_C6_overlay_same_true_wrist_angle.png", "Resolution", 300);

%% =========================================================
%% Local Functions
%% =========================================================

function generateAssignmentReferenceFigures(outDir)
    % Generate the report reference trajectory figures in the same script as
    % the URDF simulation, while keeping these DH-model variables local.

    b  = -0.025;
    L1 = 0.050;
    L2 = 0.220;
    L3 = 0.180;
    L4 = 0.100;

    a = 0.050;
    boardEdgeOffset = 0.100;
    zb = 0;
    hp = 0.095;
    zclr = 1.05*hp + hp;

    fileA = 1;
    fileB = 2;
    fileC = 3;
    fileD = 4;
    fileH = 8;

    P0 = refSquareCentre(5, fileA, a, boardEdgeOffset, zb + hp);
    P3 = refSquareCentre(6, fileC, a, boardEdgeOffset, zb + hp);
    P1 = [P0(1); P0(2); zclr];
    P2 = [P3(1); P3(2); zclr];

    rookPos = refSquareCentre(6, fileA, a, boardEdgeOffset, zb);
    pawnPos = refSquareCentre(6, fileB, a, boardEdgeOffset, zb);

    waypoints = [P0 P1 P2 P3];
    Tseg = [1.0 1.0 1.0];
    tf = sum(Tseg);

    P0Feas = refSquareCentre(1, fileD, a, boardEdgeOffset, zb + hp);
    P3Feas = refSquareCentre(8, fileH, a, boardEdgeOffset, zb + hp);
    P1Feas = [P0Feas(1); P0Feas(2); zclr];
    P2Feas = [P3Feas(1); P3Feas(2); zclr];
    waypointsFeas = [P0Feas P1Feas P2Feas P3Feas];

    C = refBuildCubicSegments(waypoints, Tseg);

    disp("Cubic coefficient matrices Cx, Cy, Cz. Rows are segments; columns are [a0 a1 a2 a3].");
    Cx = squeeze(C(:,:,1)) %#ok<NOPRT>
    Cy = squeeze(C(:,:,2)) %#ok<NOPRT>
    Cz = squeeze(C(:,:,3)) %#ok<NOPRT>

    N = 400;
    t = linspace(0, tf, N);
    pd = zeros(3, N);
    for k = 1:N
        pd(:,k) = refEvalPiecewiseTrajectory(t(k), C, Tseg);
    end

    viaTimes = [0 cumsum(Tseg)];

    figXyz = figure("Color","w");
    plot(t, pd(1,:), "LineWidth", 1.8); hold on;
    plot(t, pd(2,:), "LineWidth", 1.8);
    plot(t, pd(3,:), "LineWidth", 1.8);

    plot(viaTimes, waypoints(1,:), "ko", "MarkerFaceColor","w", "LineWidth",1.2);
    plot(viaTimes, waypoints(2,:), "ks", "MarkerFaceColor","w", "LineWidth",1.2);
    plot(viaTimes, waypoints(3,:), "kd", "MarkerFaceColor","w", "LineWidth",1.2);

    xline(Tseg(1), "--", "Via 1");
    xline(Tseg(1)+Tseg(2), "--", "Via 2");

    grid on;
    xlabel("Time, t (s)");
    ylabel("Position (m)");
    title("Reference end-effector position for A5 to C6 move");
    legend("x(t)","y(t)","z(t)","x via","y via","z via","Location","best");
    exportgraphics(figXyz, fullfile(outDir, "Fig1_xyz_vs_time.png"), "Resolution", 300);

    figXy = figure("Color","w", "Position", [100 100 920 640]);
    yPlot = pd(2,:);
    waypointYPlot = waypoints(2,:);
    P0YPlot = P0(2);
    P3YPlot = P3(2);
    rookYPlot = rookPos(2);
    pawnYPlot = pawnPos(2);

    refDrawBoardLabelsXY(a, boardEdgeOffset);
    hold on;

    hTraj = plot(pd(1,:), yPlot, "r-", "LineWidth", 2.6);

    hVia = plot(waypoints(1,:), waypointYPlot, "ro", "MarkerFaceColor","w", "LineWidth",1.5, "MarkerSize", 7);
    hStart = plot(P0(1), P0YPlot, "go", "MarkerFaceColor","g", "MarkerSize", 9);
    hFinal = plot(P3(1), P3YPlot, "bo", "MarkerFaceColor","b", "MarkerSize", 9);

    hRook = plot(rookPos(1), rookYPlot, "ms", "MarkerFaceColor","m", "MarkerSize", 9);
    hPawn = plot(pawnPos(1), pawnYPlot, "cs", "MarkerFaceColor","c", "MarkerSize", 9);

    text(P0(1), P0(2)-0.014, "A5 start", "HorizontalAlignment","center", "FontSize", 10);
    text(P3(1), P3(2)+0.014, "C6 final", "HorizontalAlignment","center", "FontSize", 10);
    text(rookPos(1), rookPos(2)+0.014, "A6 rook", "HorizontalAlignment","center", "FontSize", 10);
    text(pawnPos(1), pawnPos(2)+0.014, "B6 pawn", "HorizontalAlignment","center", "FontSize", 10);

    yline(0, "-", "Color", [0.35 0.35 0.35], "LineWidth", 0.8, "HandleVisibility","off");

    axis equal;
    xlim([boardEdgeOffset - 0.03, boardEdgeOffset + 8*a + 0.03]);
    yHalfRange = max(abs([yPlot, waypointYPlot, rookYPlot, pawnYPlot, 0]));
    yHalfRange = max(yHalfRange + 0.025, 0.20);
    ylim([-yHalfRange, yHalfRange]);
    set(gca, "XDir", "reverse", "YDir", "normal");
    ax = gca;
    ax.Toolbar.Visible = "off";
    ax.FontSize = 11;
    grid on;
    xlabel("x (m)");
    ylabel("y (m)");
    title("Reference end-effector motion projected onto the XY plane", "FontSize", 13);
    legend([hTraj, hVia, hStart, hFinal, hRook, hPawn], ...
        "trajectory","via points","start","final","rook","pawn","Location","bestoutside");
    exportgraphics(figXy, fullfile(outDir, "Fig2_xy_projection.png"), "Resolution", 300);

    figZx = figure("Color","w");
    plot(pd(1,:), pd(3,:), "k-", "LineWidth", 2.0); hold on;
    plot(waypoints(1,:), waypoints(3,:), "ro", "MarkerFaceColor","w", "LineWidth",1.4);

    yline(zclr, "--r", "z_{clr}", "LineWidth", 1.2);

    refDrawPieceZX(rookPos(1), zb, hp, 0.012, [0.8 0.2 0.8]);
    refDrawPieceZX(pawnPos(1), zb, hp, 0.010, [0.2 0.7 0.9]);
    refDrawPieceZX(P0(1), zb, hp, 0.010, [0.3 0.3 0.3]);

    text(P0(1), hp+0.008, "A5 knight", "HorizontalAlignment","center");
    text(rookPos(1), hp+0.018, "A6 rook", "HorizontalAlignment","center");
    text(pawnPos(1), hp+0.008, "B6 pawn", "HorizontalAlignment","center");

    axis equal; grid on;
    xlabel("x (m)");
    ylabel("z (m)");
    title("ZX projection showing lifting clearance");
    legend("trajectory","via points","clearance height","Location","best");
    exportgraphics(figZx, fullfile(outDir, "Fig3_zx_projection.png"), "Resolution", 300);

    Cfeas = refBuildCubicSegments(waypointsFeas, Tseg);
    pdFeas = zeros(3, N);
    for k = 1:N
        pdFeas(:,k) = refEvalPiecewiseTrajectory(t(k), Cfeas, Tseg);
    end

    sampleTimes = linspace(0, tf, 11);
    qLog = zeros(5, numel(sampleTimes));
    errLog = zeros(1, numel(sampleTimes));
    gammaLog = zeros(1, numel(sampleTimes));

    q0 = [0; 0.45; -1.15; 0.70; 0];

    [qPickFree, ~] = refIkJacobianPositionMultiSeed( ...
        waypointsFeas(:,1), q0, b, L1, L2, L3, L4, zb);

    [qPlaceFree, ~] = refIkJacobianPositionMultiSeed( ...
        waypointsFeas(:,4), qPickFree, b, L1, L2, L3, L4, zb);

    gammaPickFree  = qPickFree(2)  + qPickFree(3)  + qPickFree(4);
    gammaPlaceFree = qPlaceFree(2) + qPlaceFree(3) + qPlaceFree(4);
    gammaHold = max(gammaPickFree, gammaPlaceFree);

    disp("Free IK pick q:");
    disp(qPickFree);
    disp("Free IK place q:");
    disp(qPlaceFree);
    disp("Selected gamma_hold = max(gamma_pick, gamma_place):");
    disp(gammaHold);

    q = qPickFree;

    figFeas = figure("Color","w");
    plot(pdFeas(1,:), pdFeas(3,:), "k--", "LineWidth", 1.5); hold on;
    yline(zclr, "--r", "z_{clr}", "LineWidth", 1.0);

    refDrawPieceZX(P0Feas(1), zb, hp, 0.010, [0.3 0.3 0.3]);
    refDrawPieceZX(P3Feas(1), zb, hp, 0.010, [0.2 0.7 0.9]);
    text(P0Feas(1), hp+0.010, "D1 start", "HorizontalAlignment","center");
    text(P3Feas(1), hp+0.010, "H8 final", "HorizontalAlignment","center");

    col = parula(numel(sampleTimes));

    for k = 1:numel(sampleTimes)
        pk = refEvalPiecewiseTrajectory(sampleTimes(k), Cfeas, Tseg);

        if k == 1
            [q, errNorm] = refIkJacobianPositionHoldSelectedWristAngle( ...
                pk, qPickFree, b, L1, L2, L3, L4, gammaHold);
        elseif k == numel(sampleTimes)
            [q, errNorm] = refIkJacobianPositionHoldSelectedWristAngle( ...
                pk, q, b, L1, L2, L3, L4, gammaHold);
        else
            [q, errNorm] = refIkJacobianPosition(pk, q, b, L1, L2, L3, L4);
        end

        qLog(:,k) = q;
        errLog(k) = errNorm;
        gammaLog(k) = q(2) + q(3) + q(4);

        [~, jointPos] = refFkStandardDH(q, b, L1, L2, L3, L4);

        plot(jointPos(1,:), jointPos(3,:), "-o", ...
            "Color", col(k,:), "LineWidth", 1.2, ...
            "MarkerSize", 3, "MarkerFaceColor", col(k,:));
    end

    axis equal; grid on;
    xlabel("x (m)");
    ylabel("z (m)");
    title("Feasibility poses: D1 to H8, same wrist frame-0 angle only at pick and place");
    exportgraphics(figFeas, fullfile(outDir, "Fig4_robot_poses_D1_to_H8_jacobian_IK.png"), "Resolution", 300);

    disp("Sample times t = k tf/10:");
    disp(sampleTimes);
    disp("Joint configurations q(t_k), columns correspond to sample times:");
    disp(qLog);
    disp("Final IK position residuals at sampled poses:");
    disp(errLog);
    disp("Wrist absolute angle in frame 0 at sampled poses:");
    disp("gammaLog = q2 + q3 + q4");
    disp(gammaLog);
    disp("Selected held wrist angle gamma_hold:");
    disp(gammaHold);
end

function P = refSquareCentre(rank, fileIndex, a, boardEdgeOffset, z)
    x = boardEdgeOffset + (rank - 0.5)*a;
    y = (fileIndex - 4.5)*a;
    P = [x; y; z];
end

function C = refBuildCubicSegments(waypoints, Tseg)
    C = zeros(3, 4, 3);
    for s = 1:3
        Cs = refCubicCoefficients(waypoints(:,s), waypoints(:,s+1), Tseg(s));
        C(s,:,:) = reshape(Cs.', 1, 4, 3);
    end
end

function Cs = refCubicCoefficients(Pa, Pb, T)
    dP = Pb - Pa;
    c0 = Pa;
    c1 = zeros(3,1);
    c2 = 3*dP/(T^2);
    c3 = -2*dP/(T^3);
    Cs = [c0 c1 c2 c3];
end

function p = refEvalPiecewiseTrajectory(t, C, Tseg)
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

function [qBest, errBest] = refIkJacobianPositionMultiSeed(pDes, qPrev, b, L1, L2, L3, L4, zBoard)
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
        [qCand, errCand] = refIkJacobianPosition(pDes, seeds(i,:).', b, L1, L2, L3, L4);
        [~, jointPos] = refFkStandardDH(qCand, b, L1, L2, L3, L4);

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

function [q, errNorm] = refIkJacobianPosition(pDes, q0, b, L1, L2, L3, L4)
    q = q0;
    q(5) = 0;

    lambda = 0.02;
    tol = 1e-5;
    maxIter = 250;
    errNorm = inf;

    for iter = 1:maxIter
        [T, ~] = refFkStandardDH(q, b, L1, L2, L3, L4);
        pNow = T(1:3,4);
        e = pDes - pNow;
        errNorm = norm(e);

        if errNorm < tol
            break;
        end

        J = refNumericalJacobianPosition(q, b, L1, L2, L3, L4);
        J4 = J(:,1:4);
        dq4 = J4' * ((J4*J4' + lambda^2*eye(3)) \ e);

        dq4 = max(min(dq4, 0.12), -0.12);

        q(1:4) = q(1:4) + dq4;
        q(5) = 0;
    end

    q(1:4) = wrapToPiLocal(q(1:4));
    q(5) = 0;
end

function [q, errNorm] = refIkJacobianPositionHoldSelectedWristAngle(pDes, q0, b, L1, L2, L3, L4, gammaHold)
    q = q0;
    q(4) = gammaHold - q(2) - q(3);
    q(5) = 0;

    lambda = 0.02;
    tol = 1e-5;
    maxIter = 250;
    errNorm = inf;

    for iter = 1:maxIter
        q(4) = gammaHold - q(2) - q(3);
        q(5) = 0;

        [T, ~] = refFkStandardDH(q, b, L1, L2, L3, L4);
        pNow = T(1:3,4);

        e = pDes - pNow;
        errNorm = norm(e);

        if errNorm < tol
            break;
        end

        J3 = refNumericalJacobianHoldSelectedWristAngle(q, b, L1, L2, L3, L4, gammaHold);

        dq3 = J3' * ((J3*J3' + lambda^2*eye(3)) \ e);
        dq3 = max(min(dq3, 0.12), -0.12);

        q(1:3) = q(1:3) + dq3;

        q(4) = gammaHold - q(2) - q(3);
        q(5) = 0;
    end

    q(1:3) = wrapToPiLocal(q(1:3));
    q(4) = gammaHold - q(2) - q(3);
    q(4) = wrapToPiLocal(q(4));
    q(5) = 0;
end

function J3 = refNumericalJacobianHoldSelectedWristAngle(q, b, L1, L2, L3, L4, gammaHold)
    epsVal = 1e-6;

    qBase = q;
    qBase(4) = gammaHold - qBase(2) - qBase(3);
    qBase(5) = 0;

    [T0, ~] = refFkStandardDH(qBase, b, L1, L2, L3, L4);
    p0 = T0(1:3,4);

    J3 = zeros(3,3);

    for j = 1:3
        qp = qBase;
        qp(j) = qp(j) + epsVal;

        qp(4) = gammaHold - qp(2) - qp(3);
        qp(5) = 0;

        [Tp, ~] = refFkStandardDH(qp, b, L1, L2, L3, L4);
        pp = Tp(1:3,4);

        J3(:,j) = (pp - p0)/epsVal;
    end
end

function J = refNumericalJacobianPosition(q, b, L1, L2, L3, L4)
    epsVal = 1e-6;

    [T0, ~] = refFkStandardDH(q, b, L1, L2, L3, L4);
    p0 = T0(1:3,4);

    J = zeros(3,5);

    for j = 1:5
        qp = q;
        qp(j) = qp(j) + epsVal;

        [Tp, ~] = refFkStandardDH(qp, b, L1, L2, L3, L4);

        J(:,j) = (Tp(1:3,4) - p0)/epsVal;
    end
end

function [T, jointPos] = refFkStandardDH(q, b, L1, L2, L3, L4)
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
        T = T * refDhTransform(dh(i,1), dh(i,2), dh(i,3), dh(i,4));
        jointPos(:,i+1) = T(1:3,4);
    end
end

function A = refDhTransform(a, alpha, d, theta)
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

function refDrawBoardLabelsXY(squareSize, boardEdgeOffset)
    x0 = boardEdgeOffset;
    y0 = -4 * squareSize;

    rectangle("Position", [x0, y0, 8*squareSize, 8*squareSize], ...
        "EdgeColor", [0.45 0.45 0.45], ...
        "LineWidth", 1.0, ...
        "HandleVisibility", "off");

    for rank = 1:8
        x = x0 + (rank - 0.5) * squareSize;
        text(x, y0 + 0.012, string(rank), ...
            "HorizontalAlignment","center", ...
            "VerticalAlignment","bottom", ...
            "FontSize",9, ...
            "HandleVisibility","off");
    end

    for fileIdx = 1:8
        y = y0 + (fileIdx - 0.5) * squareSize;
        text(x0 - 0.018, y, char(double('A') + fileIdx - 1), ...
            "HorizontalAlignment","center", ...
            "FontSize",9, ...
            "HandleVisibility","off");
    end
end

function refDrawPieceZX(x, z0, h, halfWidth, colour)
    X = [x-halfWidth, x+halfWidth, x+halfWidth, x-halfWidth];
    Z = [z0, z0, z0+h, z0+h];

    patch(X, Z, colour, ...
        "FaceAlpha", 0.28, ...
        "EdgeColor", colour, ...
        "LineWidth", 1.0);
end

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

    row = str2double(squareName(2));

    boardHalf = 4 * square;

    x = boardCenter(1) - boardHalf + (row - 0.5) * square;
    % This simulation constrains the base and shows the arm in the side
    % ZX plane, so the reachable board line must stay in that same Y plane.
    y = boardCenter(2);
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
