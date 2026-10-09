clc;
clear;
close all;

%% ================= Settings =================
robot = importrobot("Sao arm.urdf");
robot.DataFormat = "row";
robot.Gravity = [0 0 -9.81];

eeName = "gripper_center_link";
elbowLinkName = "lower_arm_link";

startSquare = "A5";
goalSquare = "C6";

square = 0.05;
boardThickness = 0.01;
boardCenter = [0.30, 0, 0];
boardTopZ = boardThickness;

pickHeight = boardTopZ + 0.045;
clearHeight = boardTopZ + 0.15;

Tstart = 2.0;
Tseg = 1.5;
dt = 0.03;
targetFrameRate = 30;
animationStep = 3;
playbackSpeed = 0.5;   % 1.0 = real time, 0.5 = half speed.
plotMargin = [0.20, 0.20, 0.08];

%% ================= Cartesian Path =================
q0 = homeConfiguration(robot);

P_start = chess2world(startSquare, boardCenter, square, boardTopZ);
P_goal = chess2world(goalSquare, boardCenter, square, boardTopZ);

viaPoints = [
    P_start(1), P_start(2), pickHeight
    P_start(1), P_start(2), clearHeight
    P_goal(1),  P_goal(2),  clearHeight
    P_goal(1),  P_goal(2),  pickHeight
];

homePoint = tform2trvec(getTransform(robot, q0, eeName));
pointNames = ["Home/zero"; "P0 start pick"; "P1 start clear"; "P2 goal clear"; "P3 goal pick"];
keyPoints = [homePoint; viaPoints];
disp(table(pointNames, keyPoints(:,1), keyPoints(:,2), keyPoints(:,3), ...
    'VariableNames',{'Point','X_m','Y_m','Z_m'}));

[tPath, pPath] = generateMultiSegmentCubic(viaPoints, Tseg, dt);

%% ================= Elbow-up IK =================
ik = inverseKinematics("RigidBodyTree", robot);
weights = [0 0 0 1 1 1];  % [orientation_xyz position_xyz]

qTask = zeros(length(tPath), numel(q0));
qGuess = q0;

for i = 1:length(tPath)
    targetPose = trvec2tform(pPath(i,:));

    if i == 1
        qTask(i,:) = solveElbowUpIK(ik, robot, eeName, elbowLinkName, ...
            targetPose, weights, qGuess, pPath(i,:));
    else
        [qCandidate, ~] = ik(eeName, targetPose, weights, qGuess);
        posErr = norm(tform2trvec(getTransform(robot, qCandidate, eeName)) - pPath(i,:));

        if posErr <= 0.01 && isElbowUp(robot, qCandidate, elbowLinkName)
            qTask(i,:) = qCandidate;
        else
            qTask(i,:) = solveElbowUpIK(ik, robot, eeName, elbowLinkName, ...
                targetPose, weights, qGuess, pPath(i,:));
        end
    end

    qGuess = qTask(i,:);
end

%% ================= Home to Start Pose =================
tStart = (0:dt:Tstart).';
qStart = cubicJointTrajectory(q0, qTask(1,:), tStart, Tstart);

tRobot = [tStart; Tstart + tPath(2:end)];
qTraj = [qStart; qTask(2:end,:)];

%% ================= Path Overview =================
figure("Name","Path Overview","Color","w");
axPath = axes;
hold(axPath, "on");
drawChessboard(boardCenter, square, boardThickness);
plot3(pPath(:,1), pPath(:,2), pPath(:,3), "r", "LineWidth", 2);
scatter3(viaPoints(:,1), viaPoints(:,2), viaPoints(:,3), 38, "filled", ...
    "MarkerFaceColor",[0.0 0.35 0.9], "MarkerEdgeColor","none");
text(P_start(1), P_start(2), pickHeight + 0.015, startSquare, ...
    "FontWeight","bold", "HorizontalAlignment","center");
text(P_goal(1), P_goal(2), pickHeight + 0.015, goalSquare, ...
    "FontWeight","bold", "HorizontalAlignment","center");

formatScene(boardCenter, square, plotMargin);
title("Gripper-center path");
view(225, 28);

%% ================= Robot Animation =================
figure("Name","Robot Animation","Color","w");
axRobot = axes;
show(robot, qTraj(1,:), "Frames","off", "Visuals","on", "Parent",axRobot);
formatScene(boardCenter, square, plotMargin);
view(180, 30);

hold(axRobot, "on");
drawChessboard(boardCenter, square, boardThickness);
plot3(pPath(:,1), pPath(:,2), pPath(:,3), "r", "LineWidth", 2);
scatter3(viaPoints(:,1), viaPoints(:,2), viaPoints(:,3), 38, "filled", ...
    "MarkerFaceColor",[0.0 0.35 0.9], "MarkerEdgeColor","none");
title(sprintf("%s to %s", startSquare, goalSquare));

frameStep = max(1, animationStep);
frameIdx = unique([1:frameStep:length(tRobot), length(tRobot)]);

for k = 1:length(frameIdx)
    i = frameIdx(k);

    show(robot, qTraj(i,:), ...
        "Frames","off", ...
        "Visuals","on", ...
        "PreservePlot",false, ...
        "FastUpdate",true, ...
        "Parent",axRobot);

    title(sprintf("%s to %s, t = %.2f s", startSquare, goalSquare, tRobot(i)));
    drawnow limitrate;

    if k < length(frameIdx)
        nextI = frameIdx(k+1);
        pause((tRobot(nextI) - tRobot(i)) / playbackSpeed);
    end
end

%% =========================================================
%% Local Functions
%% =========================================================

function pos = chess2world(squareName, boardCenter, square, zHeight)
    squareName = char(squareName);
    col = double(squareName(1)) - double('A') + 1;
    row = str2double(squareName(2));

    boardHalf = 4 * square;
    x = boardCenter(1) - boardHalf + (row - 0.5) * square;
    y = boardCenter(2) + boardHalf - (col - 0.5) * square;
    pos = [x, y, zHeight];
end

function [tAll, pAll] = generateMultiSegmentCubic(points, Tseg, dt)
    tAll = [];
    pAll = [];
    globalTime = 0;

    for s = 1:size(points,1)-1
        coeff = zeros(3,4);
        for dim = 1:3
            coeff(dim,:) = cubicCoeff(points(s,dim), 0, points(s+1,dim), 0, Tseg);
        end

        tLocal = 0:dt:Tseg;
        if s > 1
            tLocal = tLocal(2:end);
        end

        pSeg = zeros(length(tLocal),3);
        for dim = 1:3
            a = coeff(dim,:);
            pSeg(:,dim) = a(1) + a(2)*tLocal(:) + a(3)*tLocal(:).^2 + a(4)*tLocal(:).^3;
        end

        tAll = [tAll; globalTime + tLocal(:)];
        pAll = [pAll; pSeg];
        globalTime = globalTime + Tseg;
    end
end

function q = cubicJointTrajectory(qStart, qEnd, tVec, T)
    q = zeros(length(tVec), numel(qStart));
    for j = 1:numel(qStart)
        a = cubicCoeff(qStart(j), 0, qEnd(j), 0, T);
        q(:,j) = a(1) + a(2)*tVec + a(3)*tVec.^2 + a(4)*tVec.^3;
    end
end

function coeff = cubicCoeff(p0, v0, pf, vf, T)
    A = [1 0 0 0;
         0 1 0 0;
         1 T T^2 T^3;
         0 1 2*T 3*T^2];
    coeff = (A \ [p0; v0; pf; vf]).';
end

function qBest = solveElbowUpIK(ik, robot, eeName, elbowLinkName, targetPose, weights, qGuess, targetPos)
    seedList = [
        qGuess
        qGuess(1),  1.2, -1.2,  0.8,  0.0, 0
        qGuess(1),  1.6, -1.6,  1.0,  0.0, 0
        qGuess(1),  0.8, -0.8,  0.6,  0.0, 0
        qGuess(1),  1.2,  1.2, -0.8,  0.0, 0
        qGuess(1), -0.8,  1.2,  0.8,  0.0, 0
    ];

    bestCost = inf;
    qBest = qGuess;

    for k = 1:size(seedList,1)
        [q, ~] = ik(eeName, targetPose, weights, seedList(k,:));
        eePos = tform2trvec(getTransform(robot, q, eeName));
        posErr = norm(eePos - targetPos);
        if posErr > 0.01
            continue;
        end

        elbowPos = tform2trvec(getTransform(robot, q, elbowLinkName));
        continuityCost = norm(q - qGuess);
        limitCost = jointLimitSoftCost(robot, q);
        cost = 0.35 * continuityCost + 4.0 * limitCost - elbowPos(3);

        if cost < bestCost
            bestCost = cost;
            qBest = q;
        end
    end

    if isinf(bestCost)
        [qBest, ~] = ik(eeName, targetPose, weights, qGuess);
    end
end

function ok = isElbowUp(robot, q, elbowLinkName)
    elbowPos = tform2trvec(getTransform(robot, q, elbowLinkName));
    ok = elbowPos(3) > 0.15;
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
        normalized = abs(q(jointIndex) - mean(lim)) / (0.5 * (lim(2) - lim(1)));
        cost = cost + max(0, normalized - 0.85)^2;
    end
end

function drawChessboard(boardCenter, square, thickness)
    boardHalf = 4 * square;
    zBoard = thickness + 0.0005;

    for col = 1:8
        for row = 1:8
            x0 = boardCenter(1) - boardHalf + (row-1)*square;
            y0 = boardCenter(2) + boardHalf - col*square;

            if mod(col+row,2)==0
                color = [0.94 0.94 0.94];
            else
                color = [0.12 0.12 0.12];
            end

            patch([x0 x0+square x0+square x0], ...
                  [y0 y0 y0+square y0+square], ...
                  [zBoard zBoard zBoard zBoard], ...
                  color, ...
                  "EdgeColor",[0.35 0.35 0.35], ...
                  "LineWidth",0.3);
        end
    end

    for col = 1:8
        y = boardCenter(2) + boardHalf - (col - 0.5) * square;
        text(boardCenter(1) - boardHalf - 0.015, y, zBoard + 0.002, ...
            char('A' + col - 1), ...
            "FontSize",8, ...
            "HorizontalAlignment","center", ...
            "Color",[0.15 0.15 0.15]);
    end
end

function formatScene(boardCenter, square, margin)
    boardHalf = 4 * square;
    axis equal;
    xlim([boardCenter(1) - boardHalf - margin(1), boardCenter(1) + boardHalf + margin(1)]);
    ylim([boardCenter(2) - boardHalf - margin(2), boardCenter(2) + boardHalf + margin(2)]);
    zlim([0, 0.45 + margin(3)]);
    grid on;
    xlabel("X (m)");
    ylabel("Y (m)");
    zlabel("Z (m)");
end
