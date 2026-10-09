%% Task 1 - single move simulation
% Simulates only Task 1 move 1: e2 -> e4.

clear; clc; close all;

scriptPath = mfilename('fullpath');
projectDir = fileparts(scriptPath);
urdfFile = fullfile(projectDir, 'Robotic_Arm_modified.urdf');

if ~isfile(urdfFile)
    error('Cannot find URDF file: %s', urdfFile);
end

robot = importrobot(urdfFile);
robot.DataFormat = 'row';
endEffector = 'gripper_center_link';

ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.MaxIterations = 180;
ik.SolverParameters.AllowRandomRestart = false;
ikWeights = [1 1 1 1 1 1];

% Stable elbow-up home posture.
homeConfig = homeConfiguration(robot);
homeConfig(1) = 3.14158;
homeConfig(2:5) = [-0.35, -0.85, -0.50, 0.00];
homeConfig(6) = 0.00;

squareSize = 0.030;          % each square is 3 cm x 3 cm
boardSize = 8 * squareSize;  % 24 cm x 24 cm chessboard
boardThickness = 0.003;
boardNearEdgeX = 0.135;      % nearest board edge is 13.5 cm from robot center
boardCenter = [boardNearEdgeX + boardSize/2, 0, 0];
boardTopZ = boardThickness;

pieceHeight = 0.045;         % all chess pieces are 45 mm tall
pieceRadius = 0.008;
graspDepthFromTop = 0.010;   % grasp point is 10 mm below the piece top
graspZ = boardTopZ + pieceHeight - graspDepthFromTop;
approachZ = 0.175;
homePos = [0.075, 0.000, 0.220];

fromSquare = 'e2';
toSquare = 'e4';
fromPos = squareToWorld(fromSquare, boardCenter, squareSize, boardTopZ);
toPos = squareToWorld(toSquare, boardCenter, squareSize, boardTopZ);

figure('Name', 'Task 1 Single Move: e2 to e4', 'Color', 'w');
show(robot, homeConfig, ...
    'Visuals', 'on', ...
    'Collisions', 'off', ...
    'Frames', 'off', ...
    'PreservePlot', false);
hold on;

drawChessboard(boardCenter, squareSize, boardThickness);
pieces = drawStartingPieces(boardCenter, squareSize, boardTopZ, pieceRadius, pieceHeight);
movingPieceIdx = findPieceAt(pieces, fromSquare);

axis equal;
axis off;
grid off;
xlim([-0.08, 0.42]);
ylim([-0.15, 0.15]);
zlim([0.00, 0.45]);
view(135, 25);
camlight;
lighting gouraud;
title('Task 1 Move 1: e2 -> e4');

disp('Task 1 single-move simulation started.');
disp(['End-effector: ', endEffector]);
disp(['Move: ', fromSquare, ' -> ', toSquare]);
disp(['Chessboard center: x = ', num2str(boardCenter(1)), ' m, y = ', num2str(boardCenter(2)), ' m']);

waypointPos = [
    homePos
    fromPos(1), fromPos(2), approachZ
    fromPos(1), fromPos(2), graspZ
    fromPos(1), fromPos(2), approachZ
    toPos(1),   toPos(2),   approachZ
    toPos(1),   toPos(2),   graspZ
    toPos(1),   toPos(2),   approachZ
    homePos
];

graspRotation = gripperDownRotation(fromPos);
waypointConfigs = solveWaypointConfigs(robot, ik, ikWeights, endEffector, homeConfig, waypointPos, graspRotation);
[homeConfig, pieces] = animateSingleMove(robot, homeConfig, waypointConfigs, waypointPos, ...
    pieces, movingPieceIdx, toSquare, toPos, pieceRadius, pieceHeight, graspDepthFromTop);

disp('Task 1 single-move simulation completed.');

function waypointConfigs = solveWaypointConfigs(robot, ik, weights, endEffector, homeConfig, waypointPos, graspRotation)
    waypointConfigs = zeros(size(waypointPos, 1), numel(homeConfig));
    waypointConfigs(1, :) = homeConfig;
    waypointConfigs(end, :) = homeConfig;

    seedConfig = homeConfig;
    for i = 2:size(waypointPos, 1)-1
        target = gripperFixedAngleTform(waypointPos(i, :), graspRotation);
        [candidateConfig, info] = ik(endEffector, target, weights, seedConfig);
        if info.ExitFlag <= 0
            warning('IK did not fully converge at waypoint %d.', i);
        end
        candidateConfig = keepElbowUp(seedConfig, candidateConfig);
        candidateConfig = clampJointLimits(candidateConfig);
        waypointConfigs(i, :) = candidateConfig;
        seedConfig = candidateConfig;
    end
end

function [config, pieces] = animateSingleMove(robot, config, waypointConfigs, waypointPos, ...
        pieces, movingPieceIdx, finalSquare, finalPos, pieceRadius, pieceHeight, graspDepthFromTop)
    stepsPerSegment = 144;

    for segmentIdx = 1:size(waypointConfigs, 1)-1
        q0 = waypointConfigs(segmentIdx, :);
        q1 = waypointConfigs(segmentIdx + 1, :);
        p0 = waypointPos(segmentIdx, :);
        p1 = waypointPos(segmentIdx + 1, :);
        pieceHeld = ismember(segmentIdx, [3, 4, 5]);

        for stepIdx = 1:stepsPerSegment
            alpha = stepIdx / stepsPerSegment;
            s = 0.5 - 0.5 * cos(pi * alpha);
            config = (1 - s) * q0 + s * q1;
            config = clampJointLimits(config);
            graspPos = (1 - s) * p0 + s * p1;

            show(robot, config, ...
                'Visuals', 'on', ...
                'Collisions', 'off', ...
                'Frames', 'off', ...
                'PreservePlot', false, ...
                'FastUpdate', true);

            if pieceHeld
                pieces(movingPieceIdx).handle = redrawPieceAtGrasp( ...
                    pieces(movingPieceIdx).handle, graspPos, pieces(movingPieceIdx).color, ...
                    pieceRadius, pieceHeight, graspDepthFromTop);
            end

            drawnow limitrate;
        end

        if segmentIdx == 5
            pieces(movingPieceIdx).square = finalSquare;
            pieces(movingPieceIdx).pos = finalPos;
            pieces(movingPieceIdx).handle = redrawPiece(pieces(movingPieceIdx).handle, ...
                finalPos, pieces(movingPieceIdx).color, pieceRadius, pieceHeight);
        end
    end
end

function config = keepElbowUp(previousConfig, candidateConfig)
    elbowIdx = 3;
    if candidateConfig(elbowIdx) > -0.02
        config = previousConfig;
    else
        config = candidateConfig;
    end
end

function config = clampJointLimits(config)
    margin = 1e-5;
    lower = [-3.14159, -2.26893, -2.26893, -2.26893, -2.26893, -3.14159] + margin;
    upper = [ 3.14159,  2.26893,  2.26893,  2.26893,  2.26893,  3.14159] - margin;
    config = min(max(config, lower), upper);
end

function pieces = drawStartingPieces(boardCenter, squareSize, boardTopZ, radius, height)
    pieces = struct('id', {}, 'color', {}, 'square', {}, 'pos', {}, 'handle', {});
    files = 'abcdefgh';
    for i = 1:8
        whiteSquare = [files(i), '2'];
        blackSquare = [files(i), '7'];
        pieces(end+1) = makePiece(['wP', files(i)], 'white', whiteSquare, ...
            squareToWorld(whiteSquare, boardCenter, squareSize, boardTopZ), radius, height);
        pieces(end+1) = makePiece(['bP', files(i)], 'black', blackSquare, ...
            squareToWorld(blackSquare, boardCenter, squareSize, boardTopZ), radius, height);
    end

    backRank = {
        'wR1','white','a1'; 'wN1','white','b1'; 'wB1','white','c1'; 'wQ','white','d1';
        'wK','white','e1'; 'wB2','white','f1'; 'wN2','white','g1'; 'wR2','white','h1';
        'bR1','black','a8'; 'bN1','black','b8'; 'bB1','black','c8'; 'bQ','black','d8';
        'bK','black','e8'; 'bB2','black','f8'; 'bN2','black','g8'; 'bR2','black','h8';
    };

    for i = 1:size(backRank, 1)
        square = backRank{i, 3};
        pieces(end+1) = makePiece(backRank{i, 1}, backRank{i, 2}, square, ...
            squareToWorld(square, boardCenter, squareSize, boardTopZ), radius, height);
    end
end

function piece = makePiece(id, colorName, square, pos, radius, height)
    piece.id = id;
    piece.color = colorName;
    piece.square = square;
    piece.pos = pos;
    piece.handle = drawPiece(pos, colorName, radius, height);
end

function idx = findPieceAt(pieces, square)
    idx = [];
    for i = 1:numel(pieces)
        if strcmp(pieces(i).square, square)
            idx = i;
            return;
        end
    end
end

function h = redrawPieceAtGrasp(oldHandle, graspPos, colorName, radius, height, graspDepthFromTop)
    basePos = graspPos(:).' - [0, 0, height - graspDepthFromTop];
    h = redrawPiece(oldHandle, basePos, colorName, radius, height);
end

function h = redrawPiece(oldHandle, pos, colorName, radius, height)
    if isgraphics(oldHandle)
        delete(oldHandle);
    end
    h = drawPiece(pos, colorName, radius, height);
end

function h = drawPiece(pos, colorName, radius, height)
    [x, y, z] = cylinder(radius, 20);
    z = z * height + pos(3);
    if strcmp(colorName, 'white')
        faceColor = [0.96 0.94 0.88];
        edgeColor = [0.35 0.35 0.35];
    else
        faceColor = [0.08 0.08 0.08];
        edgeColor = [0.02 0.02 0.02];
    end
    h = surf(x + pos(1), y + pos(2), z, ...
        'FaceColor', faceColor, ...
        'EdgeColor', edgeColor, ...
        'LineWidth', 0.2);
end

function pos = squareToWorld(square, boardCenter, squareSize, boardTopZ)
    fileIdx = double(lower(square(1))) - double('a');
    rankIdx = str2double(square(2)) - 1;
    boardSize = 8 * squareSize;
    x0 = boardCenter(1) - boardSize / 2;
    y0 = boardCenter(2) - boardSize / 2;
    pos = [x0 + (fileIdx + 0.5) * squareSize, ...
           y0 + (rankIdx + 0.5) * squareSize, ...
           boardTopZ];
end

function rotation = gripperDownRotation(pos)
    radial = [pos(1); pos(2); 0];
    if norm(radial) < 1e-6
        xAxis = [1; 0; 0];
    else
        xAxis = radial / norm(radial);
    end
    zAxis = [0; 0; -1];
    yAxis = cross(zAxis, xAxis);
    yAxis = yAxis / norm(yAxis);
    xAxis = cross(yAxis, zAxis);
    rotation = [xAxis, yAxis, zAxis];
end

function target = gripperFixedAngleTform(pos, rotation)
    target = eye(4);
    target(1:3, 1:3) = rotation;
    target(1:3, 4) = pos(:);
end

function drawChessboard(center, squareSize, boardThickness)
    boardSize = 8 * squareSize;
    x0 = center(1) - boardSize / 2;
    y0 = center(2) - boardSize / 2;
    zBottom = center(3);
    zTop = center(3) + boardThickness;

    verts = [
        x0,             y0,             zBottom
        x0 + boardSize, y0,             zBottom
        x0 + boardSize, y0 + boardSize, zBottom
        x0,             y0 + boardSize, zBottom
        x0,             y0,             zTop
        x0 + boardSize, y0,             zTop
        x0 + boardSize, y0 + boardSize, zTop
        x0,             y0 + boardSize, zTop
    ];
    faces = [
        1 2 3 4
        1 5 6 2
        2 6 7 3
        3 7 8 4
        4 8 5 1
    ];
    patch('Vertices', verts, 'Faces', faces, ...
        'FaceColor', [0.18 0.18 0.18], ...
        'EdgeColor', 'none', ...
        'FaceAlpha', 1);

    lightSquare = [0.92 0.88 0.78];
    darkSquare = [0.34 0.42 0.36];
    for fileIdx = 0:7
        for rankIdx = 0:7
            x = x0 + fileIdx * squareSize;
            y = y0 + rankIdx * squareSize;
            if mod(fileIdx + rankIdx, 2) == 0
                color = lightSquare;
            else
                color = darkSquare;
            end
            patch([x, x + squareSize, x + squareSize, x], ...
                  [y, y, y + squareSize, y + squareSize], ...
                  [zTop, zTop, zTop, zTop], ...
                  color, ...
                  'EdgeColor', [0.15 0.15 0.15], ...
                  'LineWidth', 0.25);
        end
    end
end
