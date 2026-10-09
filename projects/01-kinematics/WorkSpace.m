clear; clc; close all;

%% ============================================================
% Robot parameters
% ============================================================
b  = 2.5;
L1 = 5;
L2 = 22;
L3 = 18;
L4 = 10;

%% ============================================================
% Zero configuration
% ============================================================
q1 = 0;
q2 = 0;
q3 = 0;
q4 = 0;
q5 = 0;

%% ============================================================
% Chessboard parameters
% ============================================================
boardSquareSize   = 5;                  % cm
boardSize         = 8 * boardSquareSize;
boardHalf         = boardSize / 2;

boardThickness    = 1;                % cm
boardTopHeight    = 0;                  % cm
requiredLiftHeight = 18;                % cm

% Chessboard center position in the XY plane
boardCenter = [28, 0];                  % [x, y]

%% ============================================================
% Forward kinematics at zero configuration
% ============================================================
DH = buildDH(b, L1, L2, L3, L4, q1, q2, q3, q4, q5);
[jointPos, T_ee] = forwardKinematics(DH);

p0 = jointPos(1,:);
p1 = jointPos(2,:);
p2 = jointPos(3,:);
p3 = jointPos(4,:);
p4 = jointPos(5,:);
p5 = jointPos(6,:);

eePos = T_ee(1:3,4);

fprintf('\n=== End-Effector Position ===\n');
fprintf('X = %.2f cm\n', eePos(1));
fprintf('Y = %.2f cm\n', eePos(2));
fprintf('Z = %.2f cm\n', eePos(3));

fprintf('\n=== Joint Positions ===\n');
for i = 1:size(jointPos,1)
    fprintf('P%d: [%.2f, %.2f, %.2f]\n', ...
        i-1, jointPos(i,1), jointPos(i,2), jointPos(i,3));
end

%% ============================================================
% Workspace sampling
% ============================================================
q1Range = deg2rad(linspace(-180, 180, 50));
q2Range = deg2rad([linspace(-90,-10,20), linspace(10,90,20)]);
q3Range = deg2rad([linspace(-120,-10,20), linspace(10,120,20)]);
q4Range = deg2rad([linspace(-120,-10,20), linspace(10,120,20)]);
q5Fixed = 0;

numPoints = numel(q1Range) * numel(q2Range) * numel(q3Range) * numel(q4Range);
workspace = zeros(numPoints, 3);

idx = 1;
for q1w = q1Range
    for q2w = q2Range
        for q3w = q3Range
            for q4w = q4Range
                DHw = buildDH(b, L1, L2, L3, L4, q1w, q2w, q3w, q4w, q5Fixed);
                [~, T_w] = forwardKinematics(DHw);
                workspace(idx,:) = T_w(1:3,4)';
                idx = idx + 1;
            end
        end
    end
end

X = workspace(:,1);
Y = workspace(:,2);
Z = workspace(:,3);

%% ============================================================
% Figure 1: 3D workspace, robot and chessboard
% ============================================================
figure('Color','w','Renderer','opengl');
hold on; grid on; axis equal;
view(40,25);

xlabel('X (cm)');
ylabel('Y (cm)');
zlabel('Z (cm)');
title('Robot Workspace with Chessboard');

xlim([-35 70]);
ylim([-35 35]);
zlim([-5 35]);

set(gca, 'FontSize', 11, 'LineWidth', 1.0);

lighting gouraud;
camlight headlight;
camlight left;
material([0.3 0.7 0.12 10 1]);

drawGroundPlane([-35 70], [-35 35], -0.01);

scatter3(X, Y, Z, 10, [0.1 0.45 0.9], 'filled', ...
    'MarkerFaceAlpha', 0.18, 'MarkerEdgeAlpha', 0.18);

drawChessboard3D(boardCenter, boardSize, boardTopHeight, boardThickness);

drawTubeBetweenPoints(p0, p1, 1.25, [0.65 0.65 0.70], 40);
drawTubeBetweenPoints(p1, p2, 1.00, [0.60 0.60 0.65], 36);
drawTubeBetweenPoints(p2, p3, 1.10, [0.55 0.55 0.60], 36);
drawTubeBetweenPoints(p3, p4, 0.90, [0.50 0.50 0.55], 36);
drawTubeBetweenPoints(p4, p5, 0.72, [0.45 0.45 0.50], 36);

drawJointCylinder(p0, [0 0 1], 1.4, 2.0, [0.95 0.95 0.97], 40);
drawJointSphere(p1, 1.55, [0.97 0.97 0.98], 36);
drawJointSphere(p2, 1.35, [0.97 0.97 0.98], 36);
drawJointSphere(p3, 1.15, [0.98 0.98 0.99], 36);

drawSmallMotorCap(p2, [0 0 -1], 0.85, 1.0, [0.12 0.12 0.13], 28);
drawSmallMotorCap(p3, [0 0 -1], 0.75, 0.9, [0.12 0.12 0.13], 28);
drawSmallMotorCap(p4, [0 0 -1], 0.60, 0.75, [0.12 0.12 0.13], 28);

drawSimpleGripper(T_ee, 2.2, 1.0, 2.2);

labelOffset = [1.5, 3, 1.5];
text(p1(1)+labelOffset(1), p1(2)+labelOffset(2), p1(3)+labelOffset(3), 'J1', ...
    'FontSize',10, 'Color',[0.1 0.1 0.1], 'BackgroundColor','w', 'Margin',1);
text(p2(1)+labelOffset(1), p2(2)+labelOffset(2), p2(3)+labelOffset(3), 'J2', ...
    'FontSize',10, 'Color',[0.1 0.1 0.1], 'BackgroundColor','w', 'Margin',1);
text(p3(1)+labelOffset(1), p3(2)+labelOffset(2), p3(3)+labelOffset(3), 'J3', ...
    'FontSize',10, 'Color',[0.1 0.1 0.1], 'BackgroundColor','w', 'Margin',1);
text(p5(1)+labelOffset(1), p5(2)+labelOffset(2), p5(3)+labelOffset(3), 'EE', ...
    'FontSize',10, 'Color',[0.2 0.1 0.1], 'BackgroundColor','w', 'Margin',1);

plot3(0,0,0,'ko','MarkerFaceColor','k','MarkerSize',6);
text(1,1,0.8,'Base','FontSize',10,'BackgroundColor','w');

%% ============================================================
% Figure 2: XY top view at chessboard height
% ============================================================
xyTolerance = 0.6;
idxXY = abs(Z - boardTopHeight) < xyTolerance;

X_xy = X(idxXY);
Y_xy = Y(idxXY);

figure('Color','w');
hold on; grid on; axis equal;

drawChessboardTopView(boardCenter, boardSize);

scatter(X_xy, Y_xy, 8, [0.1 0.45 0.9], 'filled', ...
    'MarkerFaceAlpha', 0.35, 'MarkerEdgeAlpha', 0.35);

plot(0,0,'ko','MarkerFaceColor','k','MarkerSize',6);
text(1,1,'Base','FontSize',10,'BackgroundColor','w');

xlabel('X (cm)');
ylabel('Y (cm)');
title('Top View: XY Workspace at Chessboard Height');

xlim([-35 70]);
ylim([-35 35]);

%% ============================================================
% Figure 3: ZX side view at Y = 0
% ============================================================
zxTolerance = 0.6;
idxZX = abs(Y) < zxTolerance;

X_zx = X(idxZX);
Z_zx = Z(idxZX);

figure('Color','w');
hold on; grid on; axis equal;

scatter(X_zx, Z_zx, 8, [0.1 0.45 0.9], 'filled', ...
    'MarkerFaceAlpha', 0.35, 'MarkerEdgeAlpha', 0.35);

boardX1 = boardCenter(1) - boardHalf;
boardX2 = boardCenter(1) + boardHalf;
boardZ1 = boardTopHeight - boardThickness;
boardZ2 = boardTopHeight;

patch([boardX1 boardX2 boardX2 boardX1], ...
      [boardZ1 boardZ1 boardZ2 boardZ2], ...
      [0.88 0.74 0.52], ...
      'FaceAlpha', 0.75, 'EdgeColor', 'k', 'LineWidth', 1.2);

yline(boardTopHeight, 'r-', 'LineWidth', 2);
yline(requiredLiftHeight, 'k--', 'LineWidth', 2);

text(boardX2 + 1.0, boardTopHeight + 0.1, 'Board top', ...
    'FontSize', 10, 'BackgroundColor', 'w');
text(boardX2 + 1.0, requiredLiftHeight + 0.2, 'Required lift height', ...
    'FontSize', 10, 'BackgroundColor', 'w');

xlabel('X (cm)');
ylabel('Z (cm)');
title('Side View: ZX Workspace');

xlim([-35 70]);
ylim([-5 35]);

%% ============================================================
% Local functions
% ============================================================
function DH = buildDH(b, L1, L2, L3, L4, q1, q2, q3, q4, q5)
    DH = [ ...
       -b,   pi/2,   L1,      q1;
        L2,    0,     0,      q2 + pi/2;
        L3,    0,     0,      q3;
         0,   -pi/2,  0,      q4 - pi/2;
         0,    0,    L4,      q5
    ];
end

function [jointPos, T] = forwardKinematics(DH)
    numJoints = size(DH,1);
    T = eye(4);

    jointPos = zeros(numJoints + 1, 3);
    jointPos(1,:) = [0 0 0];

    for i = 1:numJoints
        A = dhTransform(DH(i,1), DH(i,2), DH(i,3), DH(i,4));
        T = T * A;
        jointPos(i+1,:) = T(1:3,4)';
    end
end

function A = dhTransform(a, alpha, d, theta)
    A = [ cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
          sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
          0,           sin(alpha),             cos(alpha),            d;
          0,           0,                      0,                     1 ];
end

function drawGroundPlane(xRange, yRange, z0)
    [X,Y] = meshgrid(linspace(xRange(1), xRange(2), 2), ...
                     linspace(yRange(1), yRange(2), 2));
    Z = z0 * ones(size(X));

    surf(X, Y, Z, ...
        'FaceColor', [0.98 0.98 0.98], ...
        'EdgeColor', 'none', ...
        'FaceAlpha', 0.96, ...
        'FaceLighting', 'gouraud');
end

function drawChessboard3D(boardCenter, boardSize, boardTopHeight, boardThickness)
    x0 = boardCenter(1) - boardSize/2;
    y0 = boardCenter(2) - boardSize/2;
    z0 = boardTopHeight - boardThickness;

    drawBlock([boardCenter(1), boardCenter(2), z0 + boardThickness/2], ...
              eye(3), [boardSize, boardSize, boardThickness], [0.55 0.35 0.15]);

    patch([x0 x0+boardSize x0+boardSize x0], ...
          [y0 y0 y0+boardSize y0+boardSize], ...
          boardTopHeight * [1 1 1 1], ...
          [0.85 0.75 0.55], ...
          'EdgeColor', 'k', ...
          'LineWidth', 1.2);
end

function drawChessboardTopView(boardCenter, boardSize)
    x0 = boardCenter(1) - boardSize/2;
    y0 = boardCenter(2) - boardSize/2;

    patch([x0 x0+boardSize x0+boardSize x0], ...
          [y0 y0 y0+boardSize y0+boardSize], ...
          [0.92 0.88 0.75], ...
          'EdgeColor', 'k', ...
          'LineWidth', 1.2);
end

function drawTubeBetweenPoints(p1, p2, radius, colorRGB, numSides)
    v = p2 - p1;
    L = norm(v);
    if L < 1e-8
        return;
    end

    [X,Y,Z] = cylinder(radius, numSides);
    Z = Z * L;

    R = rotFromZToVec(v);
    pts = R * [X(:)'; Y(:)'; Z(:)'];

    Xr = reshape(pts(1,:), size(X)) + p1(1);
    Yr = reshape(pts(2,:), size(Y)) + p1(2);
    Zr = reshape(pts(3,:), size(Z)) + p1(3);

    surf(Xr, Yr, Zr, ...
        'FaceColor', colorRGB, ...
        'EdgeColor', 'none', ...
        'FaceLighting', 'gouraud', ...
        'SpecularStrength', 0.12, ...
        'DiffuseStrength', 0.85, ...
        'AmbientStrength', 0.30);
end

function drawJointSphere(center, radius, colorRGB, numFaces)
    [X,Y,Z] = sphere(numFaces);
    X = radius * X + center(1);
    Y = radius * Y + center(2);
    Z = radius * Z + center(3);

    surf(X, Y, Z, ...
        'FaceColor', colorRGB, ...
        'EdgeColor', 'none', ...
        'FaceLighting', 'gouraud', ...
        'SpecularStrength', 0.15, ...
        'DiffuseStrength', 0.85, ...
        'AmbientStrength', 0.30);
end

function drawJointCylinder(center, axisVec, radius, lengthVal, colorRGB, numSides)
    axisVec = axisVec(:) / norm(axisVec);

    [X,Y,Z] = cylinder(radius, numSides);
    Z = Z * lengthVal - lengthVal/2;

    R = rotFromZToVec(axisVec);
    pts = R * [X(:)'; Y(:)'; Z(:)'];

    Xr = reshape(pts(1,:), size(X)) + center(1);
    Yr = reshape(pts(2,:), size(Y)) + center(2);
    Zr = reshape(pts(3,:), size(Z)) + center(3);

    surf(Xr, Yr, Zr, ...
        'FaceColor', colorRGB, ...
        'EdgeColor', 'none', ...
        'FaceLighting', 'gouraud', ...
        'SpecularStrength', 0.12);
end

function drawSmallMotorCap(center, dirVec, radius, lengthVal, colorRGB, numSides)
    dirVec = dirVec(:);
    if norm(dirVec) < 1e-10
        dirVec = [0;0;1];
    end
    dirVec = dirVec / norm(dirVec);

    startPoint = center(:) + 0.2 * radius * dirVec;
    drawOrientedCylinder(startPoint, dirVec, radius, lengthVal, colorRGB, numSides);
end

function drawOrientedCylinder(startPoint, axisVec, radius, lengthVal, colorRGB, numSides)
    axisVec = axisVec(:) / norm(axisVec);

    [X,Y,Z] = cylinder(radius, numSides);
    Z = Z * lengthVal;

    R = rotFromZToVec(axisVec);
    pts = R * [X(:)'; Y(:)'; Z(:)'];

    Xr = reshape(pts(1,:), size(X)) + startPoint(1);
    Yr = reshape(pts(2,:), size(Y)) + startPoint(2);
    Zr = reshape(pts(3,:), size(Z)) + startPoint(3);

    surf(Xr, Yr, Zr, ...
        'FaceColor', colorRGB, ...
        'EdgeColor', 'none', ...
        'FaceLighting', 'gouraud', ...
        'SpecularStrength', 0.12);
end

function drawSimpleGripper(T, palmLen, jawGap, fingerLen)
    origin = T(1:3,4);
    R = T(1:3,1:3);

    yAxis = R(:,2);
    zAxis = R(:,3);

    drawOrientedCylinder(origin - 0.5*zAxis, zAxis, 0.75, 1.0, [0.15 0.15 0.17], 28);

    palmCenter = origin + 0.45 * palmLen * zAxis;
    drawBlock(palmCenter, R, [1.2, 1.8, palmLen], [0.25 0.25 0.28]);

    finger1Center = origin + palmLen*zAxis + jawGap*yAxis + 0.5*fingerLen*zAxis;
    finger2Center = origin + palmLen*zAxis - jawGap*yAxis + 0.5*fingerLen*zAxis;

    drawBlock(finger1Center, R, [0.35, 0.35, fingerLen], [0.88 0.45 0.15]);
    drawBlock(finger2Center, R, [0.35, 0.35, fingerLen], [0.88 0.45 0.15]);
end

function drawBlock(center, R, dims, colorRGB)
    lx = dims(1)/2;
    ly = dims(2)/2;
    lz = dims(3)/2;

    vertsLocal = [ ...
        -lx -ly -lz;
         lx -ly -lz;
         lx  ly -lz;
        -lx  ly -lz;
        -lx -ly  lz;
         lx -ly  lz;
         lx  ly  lz;
        -lx  ly  lz]';

    vertsGlobal = R * vertsLocal + center(:);
    V = vertsGlobal';

    F = [1 2 3 4;
         5 6 7 8;
         1 2 6 5;
         2 3 7 6;
         3 4 8 7;
         4 1 5 8];

    patch('Vertices', V, 'Faces', F, ...
          'FaceColor', colorRGB, ...
          'EdgeColor', 'none', ...
          'FaceLighting', 'gouraud');
end

function R = rotFromZToVec(v)
    v = v(:);
    if norm(v) < 1e-10
        R = eye(3);
        return;
    end

    v = v / norm(v);
    zAxis = [0;0;1];
    c = dot(zAxis, v);

    if abs(c - 1) < 1e-10
        R = eye(3);
        return;
    elseif abs(c + 1) < 1e-10
        R = [-1 0 0; 0 -1 0; 0 0 1];
        return;
    end

    axisRot = cross(zAxis, v);
    axisRot = axisRot / norm(axisRot);
    angle = acos(c);

    K = [0 -axisRot(3) axisRot(2);
         axisRot(3) 0 -axisRot(1);
        -axisRot(2) axisRot(1) 0];

    R = eye(3) + sin(angle)*K + (1-cos(angle))*(K*K);
end