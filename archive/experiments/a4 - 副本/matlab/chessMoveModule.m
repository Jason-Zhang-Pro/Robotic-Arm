%% chessMoveModule.m
% Movement module for the Assignment 4 chess robot.
%
% Usage:
%   setup_a4
%   chessMoveModule
%   q = arm.ikXYZ([0.10, 0.15, 0.08]);
%   xy = arm.squareToXY("A1");
%   arm.moveToXYZSafe(bot, 0.10, 0.15, 0.08);
%   arm.pickAndPlaceXY(bot, arm.squareToXY("A1"), arm.squareToXY("B2"));
%   arm.applySavedOffset(bot);
%   arm.runTask1(bot);
%
% All user-set variables are placed at the top of this file. The values are
% copied into the arm module when this script is run.

%% User variables
% Edit these values after measuring the real robot.

b = 0.025;                  % DH table offset b [m]
L1 = 0.035;                 % Base height [m]
L2 = 0.19;                 % Link 2 length [m]
L3 = 0.13;                 % Link 3 length [m]
L4 = 0.09;                 % Tool / wrist offset [m]

baseOffsetXYZ = [0, 0, 0];        % Robot base in world/chessboard frame [m]
boardOffsetXYZ = [0, 0, 0];       % Later chessboard calibration offset [m]
homeXYZ = [0, 0, 0];              % Reserved only if Cartesian home is needed

boardZ = -0.060;                  % Chessboard / captured pool height [m]
squareSize = 0.030;               % 24 cm board / 8 squares [m]
boardNearEdgeX = -0.135;          % Nearest board edge to robot [m]
boardFarEdgeX = -0.375;           % Farthest board edge from robot [m]
boardYMin = -0.120;               % One side edge of board [m]
boardYMax = 0.120;                % Opposite side edge of board [m]
boardOriginXY = [boardNearEdgeX - 0.5 * squareSize, ...
                 boardYMin + 0.5 * squareSize];
                                  % A1 centre [m], assuming A is near yMin
boardXDirection = [0, 1];         % File direction: A -> H
boardYDirection = [-1, 0];        % Rank direction: 1 -> 8, away from robot
capturedPoolCenterXY = [0.150, -0.200]; % Captured pool centre [m]
capturedPoolSpacing = 0.030;      % Spacing between captured pieces [m]
capturedPoolRows = 3;             % Number of generated captured pool rows
capturedPoolCols = 3;             % Number of generated captured pool columns

qHome = [0, 0, 0, 0, 0, 0];       % Joint-space home fallback [rad]
qBias = [0, 0, 0, 0, 0, 0];       % Preferred posture for IK [rad]
qMin = deg2rad([-150, -100, -120, -130, -150, -150]);
qMax = deg2rad([ 150,  100,  120,  130,  150,  150]);
jointDirection = [-1, 1, 1, 1, 1, 1];
                                  % Set a joint to -1 if its motor direction is reversed
useJointHome = true;              % Return to qHome instead of Cartesian homeXYZ

positionTolerance = 1e-3;         % IK target tolerance [m]
pickPlaceTolerance = 2e-3;        % Required final pick/place position tolerance [m]
ikMaxIterations = 300;
ikDamping = 0.02;                 % Damped least-squares IK damping
ikStepGain = 0.7;                 % Scale applied to each Jacobian update
ikMaxJointStep = deg2rad(5);      % Max joint change per IK iteration [rad]
jacobianDelta = 1e-5;             % Numerical Jacobian finite difference [rad]
requireElbowAboveFrame5 = true;   % Reject elbow-down IK poses
elbowClearance = 0.005;           % Elbow must be this much above frame 5 origin [m]
elbowPenaltyWeight = 1000;        % Cost penalty when elbow is too low
frame5AxisToHold = 1;             % 1=x5, 2=y5, 3=z5 axis used for ground angle
frame5AngleWeight = 1.0;          % Weight for frame5-ground angle error in constrained IK
frame5PreferredGroundAngle = deg2rad(-45);
                                  % Preferred shared angle; negative means x5 points downward
frame5AngleSearchMin = deg2rad(-85);
frame5AngleSearchMax = deg2rad(85);
frame5AngleSearchStep = deg2rad(5);
frame5AngleTolerance = deg2rad(2);

numSteps = 1;                     % Minimum interpolation steps
stepPause = 0.01;                  % Pause between commands [s]
maxJointCommandStep = deg2rad(1); % Max commanded joint change per step [rad]
gripZ = boardZ + 0.01;                   % Gripping / placing height [m]
moveZ = boardZ + 0.100;           % Safe travel height while moving [m]
defaultMoveTime = 1.2;            % Reserved for future timing logic [s]

numJoints = 6;
driveMode = zeros(1, numJoints);  % 0 = position mode
wrapJointCommands = false;        % Disabled: do not wrap angles before commanding

gripperOpenAngle = deg2rad(30);   % Gripper open command angle [rad]
gripperCloseAngle = deg2rad(0);   % Gripper closed command angle [rad]
gripperMotorIndex = 6;            % Motor index used for gripper open/close
gripperPause = 0.30;              % Wait after open/close command [s]

opponentReadyChar = 'r';          % Serial character received after black move
waitForOpponent = true;           % Set false to dry-run Task 1 without waiting
autoApplySavedOffset = true;      % Apply saved servo offset before Task 1
autoApplyOffsetForMotion = true;  % Apply saved offset before any motion/read
servoOffsetFile = fullfile(fileparts(fileparts(mfilename('fullpath'))), ...
    "servo_zero_offsets.mat");    % Saved MATLAB zero reference file

%% Build module
cfg.link.b = b;
cfg.link.L1 = L1;
cfg.link.L2 = L2;
cfg.link.L3 = L3;
cfg.link.L4 = L4;

cfg.baseOffsetXYZ = baseOffsetXYZ;
cfg.boardOffsetXYZ = boardOffsetXYZ;
cfg.homeXYZ = homeXYZ;
cfg.board.squareSize = squareSize;
cfg.board.originXY = boardOriginXY;
cfg.board.xDirection = boardXDirection / norm(boardXDirection);
cfg.board.yDirection = boardYDirection / norm(boardYDirection);
cfg.board.nearEdgeX = boardNearEdgeX;
cfg.board.farEdgeX = boardFarEdgeX;
cfg.board.yMin = boardYMin;
cfg.board.yMax = boardYMax;
cfg.capture.poolXYList = generatePoolXYList( ...
    capturedPoolCenterXY, capturedPoolSpacing, ...
    capturedPoolRows, capturedPoolCols);
cfg.capture.nextIndex = 1;

cfg.qHome = qHome;
cfg.qBias = qBias;
cfg.qMin = qMin;
cfg.qMax = qMax;
cfg.jointDirection = jointDirection;
cfg.useJointHome = useJointHome;

cfg.positionTolerance = positionTolerance;
cfg.pickPlaceTolerance = pickPlaceTolerance;
cfg.ikMaxIterations = ikMaxIterations;
cfg.ikDamping = ikDamping;
cfg.ikStepGain = ikStepGain;
cfg.ikMaxJointStep = ikMaxJointStep;
cfg.jacobianDelta = jacobianDelta;
cfg.ik.requireElbowAboveFrame5 = requireElbowAboveFrame5;
cfg.ik.elbowClearance = elbowClearance;
cfg.ik.elbowPenaltyWeight = elbowPenaltyWeight;
cfg.frame5.axisToHold = frame5AxisToHold;
cfg.frame5.angleWeight = frame5AngleWeight;
cfg.frame5.axisWeight = frame5AngleWeight;  % Compatibility for optional full-axis IK helper.
cfg.frame5.preferredGroundAngle = frame5PreferredGroundAngle;
cfg.frame5.searchMin = frame5AngleSearchMin;
cfg.frame5.searchMax = frame5AngleSearchMax;
cfg.frame5.searchStep = frame5AngleSearchStep;
cfg.frame5.angleTolerance = frame5AngleTolerance;

cfg.motion.numSteps = numSteps;
cfg.motion.stepPause = stepPause;
cfg.motion.maxJointCommandStep = maxJointCommandStep;
cfg.motion.gripZ = gripZ;
cfg.motion.moveZ = moveZ;
cfg.motion.defaultMoveTime = defaultMoveTime;

cfg.motor.numJoints = numJoints;
cfg.motor.driveMode = driveMode;
cfg.motor.wrapJointCommands = wrapJointCommands;

cfg.gripper.openAngle = gripperOpenAngle;
cfg.gripper.closeAngle = gripperCloseAngle;
cfg.gripper.motorIndex = gripperMotorIndex;
cfg.gripper.pause = gripperPause;
cfg.arm.numKinematicJoints = 5;

cfg.task1.opponentReadyChar = opponentReadyChar;
cfg.task1.waitForOpponent = waitForOpponent;
cfg.task1.autoApplySavedOffset = autoApplySavedOffset;
cfg.offset.file = servoOffsetFile;
cfg.offset.autoApplyForMotion = autoApplyOffsetForMotion;

arm = makeChessMoveModule(cfg);

fprintf('Chess move module loaded. Use arm.moveToXYZSafe(bot, x, y, z).\n');

%% Local functions
function arm = makeChessMoveModule(cfg)
offsetApplied = false;
offsetWarningShown = false;
motorOffset = zeros(1, cfg.motor.numJoints);

arm.config = cfg;
arm.fk = @fk;
arm.ikXYZ = @ikXYZ;
arm.moveJoints = @moveJoints;
arm.moveToXYZ = @moveToXYZ;
arm.moveToXYZSafe = @moveToXYZSafe;
arm.planPickAndPlaceXY = @planPickAndPlaceXY;
arm.pickAndPlaceXY = @pickAndPlaceXY;
arm.runTask1 = @runTask1;
arm.task1Moves = @task1Moves;
arm.saveCurrentOffset = @saveCurrentOffset;
arm.applySavedOffset = @applySavedOffset;
arm.clearOffset = @clearOffset;
arm.readCurrentQ = @readCurrentQ;
arm.dhTable = @dhTable;
arm.positionJacobian = @positionJacobian;
arm.jointPositions = @jointPositions;
arm.squareToXY = @squareToXY;
arm.frame5Axis = @frame5Axis;
arm.ikXYZHoldFrame5Angle = @ikXYZHoldFrame5Angle;
arm.ikXYZHoldFrame5Axis = @ikXYZHoldFrame5Axis;
arm.frame5GroundAngle = @frame5GroundAngle;
arm.absoluteFrame5Angle = @frame5GroundAngle;
arm.absoluteQ4 = @frame5GroundAngle;
arm.ikXYZHoldAbsoluteQ4 = @ikXYZHoldFrame5Angle;

    function q = ikXYZ(targetXYZ, qInitial)
        if nargin < 2 || isempty(qInitial)
            qInitial = cfg.qBias;
        end

        targetXYZ = row3(targetXYZ) - cfg.baseOffsetXYZ - cfg.boardOffsetXYZ;
        seedList = ikSeedList(qInitial);
        qBest = seedList(1, :);
        bestCost = inf;

        for seedIndex = 1:size(seedList, 1)
            qCandidate = solveIkXYZFromSeed(targetXYZ, seedList(seedIndex, :));
            cost = ikSolutionCost(qCandidate, targetXYZ, qInitial);

            if cost < bestCost
                bestCost = cost;
                qBest = qCandidate;
            end
        end

        q = qBest;

        actualXYZ = fk(q);
        err = norm(actualXYZ(1:3, 4)' - targetXYZ);
        if err > cfg.positionTolerance
            warning('IK position error is %.4f m, above tolerance %.4f m.', ...
                err, cfg.positionTolerance);
        end

        warnIfElbowDown(q);
    end

    function q = solveIkXYZFromSeed(targetXYZ, qSeed)
        q = clamp(rowN(qSeed, cfg.motor.numJoints), cfg.qMin, cfg.qMax);

        for iteration = 1:cfg.ikMaxIterations
            actualT = fk(q);
            actualXYZ = actualT(1:3, 4)';
            errorXYZ = targetXYZ - actualXYZ;

            if norm(errorXYZ) <= cfg.positionTolerance
                break;
            end

            J = positionJacobian(q);
            J = J(:, 1:cfg.arm.numKinematicJoints);
            lambda = cfg.ikDamping;
            dq = (J' / (J * J' + lambda^2 * eye(3))) * errorXYZ';
            dq = cfg.ikStepGain * dq';
            dq = clamp(dq, -cfg.ikMaxJointStep, cfg.ikMaxJointStep);

            q(1:cfg.arm.numKinematicJoints) = ...
                q(1:cfg.arm.numKinematicJoints) + dq;
            q = clamp(q, cfg.qMin, cfg.qMax);
        end
    end

    function moveToXYZ(bot, x, y, z, qStart)
        if nargin < 5 || isempty(qStart)
            qStart = readCurrentQ(bot);
        end

        qTarget = ikXYZ([x, y, z], qStart);
        moveJoints(bot, qStart, qTarget);
    end

    function moveToXYZSafe(bot, x, y, z, qStart)
        if nargin < 5 || isempty(qStart)
            qStart = readCurrentQ(bot);
        end

        currentT = fk(qStart);
        currentXYZ = currentT(1:3, 4)';

        safeCurrentXYZ = [currentXYZ(1), currentXYZ(2), cfg.motion.moveZ];
        safeTargetXYZ = [x, y, cfg.motion.moveZ];

        q1 = ikXYZ(safeCurrentXYZ, qStart);
        moveJoints(bot, qStart, q1);

        q2 = ikXYZ(safeTargetXYZ, q1);
        moveJoints(bot, q1, q2);

        q3 = ikXYZ([x, y, z], q2);
        moveJoints(bot, q2, q3);
    end

    function pickAndPlaceXY(bot, startXY, finalXY, qStart)
        if nargin < 4 || isempty(qStart)
            qStart = readCurrentQ(bot);
        end

        startXY = row2(startXY);
        finalXY = row2(finalXY);

        pickXYZ = [startXY, cfg.motion.gripZ];
        placeXYZ = [finalXY, cfg.motion.gripZ];
        pickAboveXYZ = [startXY, cfg.motion.moveZ];
        placeAboveXYZ = [finalXY, cfg.motion.moveZ];

        plan = planPickAndPlaceXY(startXY, finalXY, qStart);
        qPick = plan.qPick;
        qPlace = plan.qPlace;
        pickAboveXYZ = plan.pickAboveXYZ;
        placeAboveXYZ = plan.placeAboveXYZ;
        assertPosePosition('pick', qPick, pickXYZ, cfg.pickPlaceTolerance);
        assertPosePosition('place', qPlace, placeXYZ, cfg.pickPlaceTolerance);

        qAbovePick = withGripperAngle(ikXYZ(pickAboveXYZ, qStart), cfg.gripper.openAngle);
        moveJoints(bot, qStart, qAbovePick);

        setGripper(bot, cfg.gripper.openAngle);
        qPick = withGripperAngle(qPick, cfg.gripper.openAngle);
        moveJoints(bot, qAbovePick, qPick);
        setGripper(bot, cfg.gripper.closeAngle);
        qPick = withGripperAngle(qPick, cfg.gripper.closeAngle);

        qLift = withGripperAngle(ikXYZ(pickAboveXYZ, qPick), cfg.gripper.closeAngle);
        moveJoints(bot, qPick, qLift);

        qAbovePlace = withGripperAngle(ikXYZ(placeAboveXYZ, qLift), cfg.gripper.closeAngle);
        moveJoints(bot, qLift, qAbovePlace);

        qPlace = withGripperAngle(qPlace, cfg.gripper.closeAngle);
        moveJoints(bot, qAbovePlace, qPlace);
        setGripper(bot, cfg.gripper.openAngle);
        qPlace = withGripperAngle(qPlace, cfg.gripper.openAngle);

        qRetreat = withGripperAngle(ikXYZ(placeAboveXYZ, qPlace), cfg.gripper.openAngle);
        moveJoints(bot, qPlace, qRetreat);
    end

    function plan = planPickAndPlaceXY(startXY, finalXY, qStart)
        if nargin < 3 || isempty(qStart)
            qStart = cfg.qBias;
        end

        startXY = row2(startXY);
        finalXY = row2(finalXY);

        pickXYZ = [startXY, cfg.motion.gripZ];
        placeXYZ = [finalXY, cfg.motion.gripZ];
        pickAboveXYZ = [startXY, cfg.motion.moveZ];
        placeAboveXYZ = [finalXY, cfg.motion.moveZ];

        qPickFree = ikXYZ(pickXYZ, qStart);
        qPlaceFree = ikXYZ(placeXYZ, qPickFree);

        [qPick, qPlace, frame5AngleHold, angleStats] = ...
            chooseSharedFrame5GroundAngle(pickXYZ, placeXYZ, qPickFree, qPlaceFree);

        plan = struct();
        plan.pickXYZ = pickXYZ;
        plan.placeXYZ = placeXYZ;
        plan.pickAboveXYZ = pickAboveXYZ;
        plan.placeAboveXYZ = placeAboveXYZ;
        plan.qPickFree = qPickFree;
        plan.qPlaceFree = qPlaceFree;
        plan.qPick = qPick;
        plan.qPlace = qPlace;
        plan.qAbovePick = ikXYZ(pickAboveXYZ, qStart);
        plan.qAbovePlace = ikXYZ(placeAboveXYZ, plan.qAbovePick);
        plan.frame5AnglePickFree = frame5GroundAngle(qPickFree);
        plan.frame5AnglePlaceFree = frame5GroundAngle(qPlaceFree);
        plan.frame5AngleHold = frame5AngleHold;
        plan.frame5AngleStats = angleStats;
    end

    function runTask1(bot)
        moves = task1Moves();
        cfg.capture.nextIndex = 1;

        if cfg.task1.autoApplySavedOffset
            applySavedOffset(bot);
        end

        qCurrent = readCurrentQ(bot);
        qHomeCurrent = homeConfiguration(qCurrent);
        moveJoints(bot, qCurrent, qHomeCurrent);

        for moveIndex = 1:numel(moves)
            move = moves(moveIndex);
            fprintf('Task 1 robot move %d: %s -> %s\n', ...
                moveIndex, move.from, move.to);

            startXY = squareToXY(move.from);
            if strcmpi(move.to, 'captured pool')
                finalXY = nextCapturedPoolXY();
            else
                finalXY = squareToXY(move.to);
            end

            pickAndPlaceXY(bot, startXY, finalXY, qHomeCurrent);

            qAfterMove = readCurrentQ(bot);
            qHomeCurrent = homeConfiguration(qAfterMove);
            moveJoints(bot, qAfterMove, qHomeCurrent);

            if move.waitAfter
                waitUntilOpponentReady(bot);
            end
        end

        fprintf('Task 1 complete.\n');
    end

    function qHomeTarget = homeConfiguration(qCurrent)
        if cfg.useJointHome
            qHomeTarget = rowN(cfg.qHome, cfg.motor.numJoints);
        else
            qHomeTarget = ikXYZ(cfg.homeXYZ, qCurrent);
        end
    end

    function moves = task1Moves()
        moves = struct( ...
            'from', {}, ...
            'to', {}, ...
            'waitAfter', {});

        moves(end+1) = task1Move('e2', 'e4', true);
        moves(end+1) = task1Move('g1', 'f3', true);
        moves(end+1) = task1Move('f1', 'b5', true);
        moves(end+1) = task1Move('b1', 'c3', true);
        moves(end+1) = task1Move('e1', 'g1', false);
        moves(end+1) = task1Move('h1', 'f1', true);
        moves(end+1) = task1Move('d5', 'captured pool', false);
        moves(end+1) = task1Move('e4', 'd5', true);
        moves(end+1) = task1Move('d5', 'captured pool', false);
        moves(end+1) = task1Move('c3', 'd5', true);
        moves(end+1) = task1Move('c6', 'captured pool', false);
        moves(end+1) = task1Move('b5', 'c6', true);
        moves(end+1) = task1Move('c2', 'c3', true);
        moves(end+1) = task1Move('f3', 'g5', true);
        moves(end+1) = task1Move('d2', 'd4', true);
        moves(end+1) = task1Move('d1', 'f3', true);
        moves(end+1) = task1Move('d5', 'captured pool', false);
        moves(end+1) = task1Move('f3', 'd5', true);
        moves(end+1) = task1Move('c1', 'captured pool', false);
        moves(end+1) = task1Move('a1', 'c1', true);
        moves(end+1) = task1Move('g1', 'h1', true);
        moves(end+1) = task1Move('f2', 'f4', true);
        moves(end+1) = task1Move('g5', 'h3', true);
        moves(end+1) = task1Move('g2', 'g3', true);
        moves(end+1) = task1Move('c1', 'd1', true);
        moves(end+1) = task1Move('f1', 'e1', true);
    end

    function move = task1Move(fromSquare, toSquare, waitAfter)
        move.from = fromSquare;
        move.to = toSquare;
        move.waitAfter = waitAfter;
    end

    function saveCurrentOffset(bot)
        offsetApplied = false;
        clearOffset(bot);
        bot.getFB();
        savedMotorOffset = bot.q_FB;
        save(cfg.offset.file, "savedMotorOffset");
        motorOffset = savedMotorOffset(1:cfg.motor.numJoints);
        bot.setOffset(savedMotorOffset);
        offsetApplied = true;

        fprintf("Saved current motor feedback as offset:\n");
        disp(savedMotorOffset);
        fprintf("Offset file:\n%s\n", cfg.offset.file);
    end

    function applied = applySavedOffset(bot)
        applied = false;

        if exist(cfg.offset.file, "file") ~= 2
            warning("No saved servo offset file found: %s", cfg.offset.file);
            return;
        end

        data = loadOffsetFile(cfg.offset.file);
        loadedOffset = rowN(data, bot.max_id);
        bot.setOffset(loadedOffset);
        motorOffset = loadedOffset(1:cfg.motor.numJoints);
        applied = true;
        offsetApplied = true;

        fprintf("Applied saved motor offset:\n");
        disp(loadedOffset);
    end

    function clearOffset(bot)
        bot.setOffset(zeros(1, bot.max_id));
        motorOffset = zeros(1, cfg.motor.numJoints);
        offsetApplied = false;
    end

    function moveJoints(bot, qStart, qTarget)
        ensureOffsetApplied(bot);

        qStart = rowN(qStart, cfg.motor.numJoints);
        qTarget = rowN(qTarget, cfg.motor.numJoints);

        qDelta = qTarget - qStart;
        armJoints = 1:cfg.arm.numKinematicJoints;
        maxDelta = max(abs(qDelta(armJoints)));
        stepLimitedSteps = ceil(maxDelta / cfg.motion.maxJointCommandStep);
        effectiveSteps = max(cfg.motion.numSteps, stepLimitedSteps);
        estimatedDuration = effectiveSteps * cfg.motion.stepPause;

        fprintf('moveJoints: steps=%d, stepPause=%.3f s, estimatedDuration=%.2f s, maxDelta=%.2f deg\n', ...
            effectiveSteps, cfg.motion.stepPause, estimatedDuration, rad2deg(maxDelta));

        if ismethod(bot, 'setDriveControlMode')
            bot.setDriveControlMode(cfg.motor.driveMode);
        end

        for k = 1:effectiveSteps
            s = smoothStep(k / effectiveSteps);
            qCmd = qStart + s * qDelta;
            bot.setQ(toMotorQ(qCmd));
            bot.driveMotors();
            pause(cfg.motion.stepPause);
        end
    end

    function varargout = squareToXY(squareName)
        squareName = upper(strtrim(char(squareName)));
        if numel(squareName) ~= 2
            error('Square name must be like A1, B2, or H8.');
        end

        fileIndex = double(squareName(1)) - double('A') + 1;
        rankIndex = str2double(squareName(2));

        if fileIndex < 1 || fileIndex > 8 || isnan(rankIndex) || ...
                rankIndex < 1 || rankIndex > 8
            error('Square name must be in the range A1 to H8.');
        end

        localXY = cfg.board.originXY + ...
            (fileIndex - 1) * cfg.board.squareSize * cfg.board.xDirection + ...
            (rankIndex - 1) * cfg.board.squareSize * cfg.board.yDirection;

        xy = cfg.boardOffsetXYZ(1:2) + localXY;

        if nargout <= 1
            varargout{1} = xy;
        else
            varargout{1} = xy(1);
            varargout{2} = xy(2);
        end
    end

    function gamma = absoluteQ4(q)
        q = rowN(q, cfg.motor.numJoints);
        gamma = q(2) + q(3) + q(4);
    end

    function angle = frame5GroundAngle(q)
        T = fk(q);
        axisIndex = cfg.frame5.axisToHold;
        frameAxis = T(1:3, axisIndex);
        angle = atan2(frameAxis(3), norm(frameAxis(1:2)));
    end

    function axisVector = frame5Axis(q)
        T = fk(q);
        axisVector = T(1:3, cfg.frame5.axisToHold)';
        axisVector = axisVector / norm(axisVector);
    end

    function [qPick, qPlace, angleHold, stats] = chooseSharedFrame5GroundAngle( ...
            pickXYZ, placeXYZ, qPickInitial, qPlaceInitial)
        pickTarget = row3(pickXYZ) - cfg.baseOffsetXYZ - cfg.boardOffsetXYZ;
        placeTarget = row3(placeXYZ) - cfg.baseOffsetXYZ - cfg.boardOffsetXYZ;

        freePickAngle = frame5GroundAngle(qPickInitial);
        freePlaceAngle = frame5GroundAngle(qPlaceInitial);
        coarseAngles = cfg.frame5.searchMin:cfg.frame5.searchStep:cfg.frame5.searchMax;
        if isempty(coarseAngles) || coarseAngles(end) < cfg.frame5.searchMax
            coarseAngles = [coarseAngles, cfg.frame5.searchMax];
        end
        candidateAngles = unique([ ...
            cfg.frame5.preferredGroundAngle, ...
            freePickAngle, ...
            freePlaceAngle, ...
            coarseAngles], 'stable');

        bestFeasibleCost = inf;
        bestAnyCost = inf;
        bestFeasible = [];
        bestAny = [];

        for angleIndex = 1:numel(candidateAngles)
            candidateAngle = candidateAngles(angleIndex);
            [candidatePick, pickErr, pickAngleErr] = ...
                solveBestFrame5AngleTarget(pickTarget, candidateAngle, qPickInitial);
            [candidatePlace, placeErr, placeAngleErr] = ...
                solveBestFrame5AngleTarget(placeTarget, candidateAngle, qPlaceInitial);

            feasible = pickErr <= cfg.pickPlaceTolerance && ...
                placeErr <= cfg.pickPlaceTolerance && ...
                pickAngleErr <= cfg.frame5.angleTolerance && ...
                placeAngleErr <= cfg.frame5.angleTolerance;
            preferenceCost = abs(candidateAngle - cfg.frame5.preferredGroundAngle);
            jointCost = 0.001 * (norm(candidatePick - qPickInitial) + ...
                norm(candidatePlace - qPlaceInitial));
            feasibleCost = preferenceCost + jointCost;
            anyCost = 1000 * (pickErr + placeErr) + ...
                10 * (pickAngleErr + placeAngleErr) + ...
                0.01 * preferenceCost + jointCost;

            candidate = struct( ...
                'qPick', candidatePick, ...
                'qPlace', candidatePlace, ...
                'angleHold', candidateAngle, ...
                'pickErr', pickErr, ...
                'placeErr', placeErr, ...
                'pickAngleErr', pickAngleErr, ...
                'placeAngleErr', placeAngleErr, ...
                'feasible', feasible);

            if feasible && feasibleCost < bestFeasibleCost
                bestFeasibleCost = feasibleCost;
                bestFeasible = candidate;
            end
            if anyCost < bestAnyCost
                bestAnyCost = anyCost;
                bestAny = candidate;
            end
        end

        if ~isempty(bestFeasible)
            chosen = bestFeasible;
        else
            chosen = bestAny;
            warning(['No shared frame5 ground angle satisfied both pick/place ' ...
                'tolerances. Best candidate has pick %.4f m, place %.4f m, ' ...
                'pick angle %.2f deg, place angle %.2f deg.'], ...
                chosen.pickErr, chosen.placeErr, ...
                rad2deg(chosen.pickAngleErr), rad2deg(chosen.placeAngleErr));
        end

        qPick = chosen.qPick;
        qPlace = chosen.qPlace;
        angleHold = chosen.angleHold;
        stats = chosen;
        fprintf(['Selected shared frame5 x-axis ground angle: %.4f rad / %.2f deg ' ...
            '(pick %.4f m, place %.4f m)\n'], ...
            angleHold, rad2deg(angleHold), chosen.pickErr, chosen.placeErr);
    end

    function q = ikXYZHoldFrame5Angle(targetXYZ, frame5AngleHold, qInitial)
        if nargin < 3 || isempty(qInitial)
            qInitial = cfg.qBias;
        end

        targetXYZ = row3(targetXYZ) - cfg.baseOffsetXYZ - cfg.boardOffsetXYZ;
        [q, err, angleErr] = solveBestFrame5AngleTarget( ...
            targetXYZ, frame5AngleHold, qInitial);
        if err > cfg.positionTolerance
            warning('Frame5-angle IK position error is %.4f m, above tolerance %.4f m.', ...
                err, cfg.positionTolerance);
        end
        if angleErr > deg2rad(1)
            warning('Frame5-ground angle error is %.4f rad / %.2f deg.', ...
                angleErr, rad2deg(angleErr));
        end

        warnIfElbowDown(q);
    end

    function [qBest, errBest, angleErrBest] = solveBestFrame5AngleTarget( ...
            targetXYZ, frame5AngleHold, qInitial)
        seedList = ikSeedList(qInitial);
        qBest = seedList(1, :);
        bestCost = inf;
        errBest = inf;
        angleErrBest = inf;

        for seedIndex = 1:size(seedList, 1)
            qCandidate = solveIkXYZHoldFrame5AngleFromSeed( ...
                targetXYZ, frame5AngleHold, seedList(seedIndex, :));
            actualXYZ = fk(qCandidate);
            err = norm(actualXYZ(1:3, 4)' - targetXYZ);
            angleErr = abs(frame5AngleHold - frame5GroundAngle(qCandidate));
            cost = ikSolutionCost(qCandidate, targetXYZ, qInitial) + ...
                cfg.frame5.angleWeight * angleErr^2;

            if cost < bestCost
                bestCost = cost;
                qBest = qCandidate;
                errBest = err;
                angleErrBest = angleErr;
            end
        end
    end

    function q = solveIkXYZHoldFrame5AngleFromSeed(targetXYZ, frame5AngleHold, qSeed)
        q = clamp(rowN(qSeed, cfg.motor.numJoints), cfg.qMin, cfg.qMax);

        for iteration = 1:cfg.ikMaxIterations
            actualT = fk(q);
            actualXYZ = actualT(1:3, 4)';
            errorXYZ = targetXYZ - actualXYZ;
            errorAngle = frame5AngleHold - frame5GroundAngle(q);
            errorTask = [errorXYZ, cfg.frame5.angleWeight * errorAngle];

            if norm(errorXYZ) <= cfg.positionTolerance && abs(errorAngle) <= deg2rad(1)
                break;
            end

            J = positionFrame5AngleJacobian(q);
            J(4, :) = cfg.frame5.angleWeight * J(4, :);
            lambda = cfg.ikDamping;
            dq = (J' / (J * J' + lambda^2 * eye(4))) * errorTask';
            dq = cfg.ikStepGain * dq';
            dq = clamp(dq, -cfg.ikMaxJointStep, cfg.ikMaxJointStep);

            q(1:cfg.arm.numKinematicJoints) = ...
                q(1:cfg.arm.numKinematicJoints) + ...
                dq(1:cfg.arm.numKinematicJoints);
            q = clamp(q, cfg.qMin, cfg.qMax);
        end
    end

    function q = ikXYZHoldFrame5Axis(targetXYZ, frame5AxisHold, qInitial)
        if nargin < 3 || isempty(qInitial)
            qInitial = cfg.qBias;
        end

        targetXYZ = row3(targetXYZ) - cfg.baseOffsetXYZ - cfg.boardOffsetXYZ;
        frame5AxisHold = row3(frame5AxisHold);
        frame5AxisHold = frame5AxisHold / norm(frame5AxisHold);
        seedList = ikSeedList(qInitial);
        qBest = seedList(1, :);
        bestCost = inf;

        for seedIndex = 1:size(seedList, 1)
            qCandidate = solveIkXYZHoldFrame5AxisFromSeed( ...
                targetXYZ, frame5AxisHold, seedList(seedIndex, :));
            axisError = frame5AxisHold - frame5Axis(qCandidate);
            cost = ikSolutionCost(qCandidate, targetXYZ, qInitial) + ...
                cfg.frame5.axisWeight * norm(axisError)^2;

            if cost < bestCost
                bestCost = cost;
                qBest = qCandidate;
            end
        end

        q = qBest;

        actualXYZ = fk(q);
        err = norm(actualXYZ(1:3, 4)' - targetXYZ);
        axisErr = norm(frame5AxisHold - frame5Axis(q));
        if err > cfg.positionTolerance
            warning('Frame5-axis IK position error is %.4f m, above tolerance %.4f m.', ...
                err, cfg.positionTolerance);
        end
        if axisErr > 0.05
            warning('Frame5 axis error is %.4f. Axis may not be held closely.', axisErr);
        end

        warnIfElbowDown(q);
    end

    function q = solveIkXYZHoldFrame5AxisFromSeed(targetXYZ, frame5AxisHold, qSeed)
        q = clamp(rowN(qSeed, cfg.motor.numJoints), cfg.qMin, cfg.qMax);

        for iteration = 1:cfg.ikMaxIterations
            actualT = fk(q);
            actualXYZ = actualT(1:3, 4)';
            errorXYZ = targetXYZ - actualXYZ;
            errorAxis = frame5AxisHold - frame5Axis(q);
            errorTask = [errorXYZ, cfg.frame5.axisWeight * errorAxis];

            if norm(errorXYZ) <= cfg.positionTolerance && norm(errorAxis) <= 0.02
                break;
            end

            J = positionFrame5AxisJacobian(q);
            J(4:6, :) = cfg.frame5.axisWeight * J(4:6, :);
            lambda = cfg.ikDamping;
            dq = (J' / (J * J' + lambda^2 * eye(6))) * errorTask';
            dq = cfg.ikStepGain * dq';
            dq = clamp(dq, -cfg.ikMaxJointStep, cfg.ikMaxJointStep);

            q(1:cfg.arm.numKinematicJoints) = ...
                q(1:cfg.arm.numKinematicJoints) + ...
                dq(1:cfg.arm.numKinematicJoints);
            q = clamp(q, cfg.qMin, cfg.qMax);
        end
    end

    function xy = nextCapturedPoolXY()
        poolIndex = cfg.capture.nextIndex;
        if poolIndex > size(cfg.capture.poolXYList, 1)
            error('No captured pool positions left. Increase capturedPoolRows or capturedPoolCols.');
        end

        xy = cfg.capture.poolXYList(poolIndex, :);
        cfg.capture.nextIndex = cfg.capture.nextIndex + 1;
    end

    function T = fk(q)
        q = rowN(q, cfg.motor.numJoints);
        T = eye(4);
        table = dhTable(q);

        for i = 1:size(table, 1)
            alphaPrev = table(i, 2);
            aPrev = table(i, 3);
            dCurrent = table(i, 4);
            thetaCurrent = table(i, 5);

            % The supplied convention uses theta_i about the current z_i.
            A = rotX(alphaPrev) * transX(aPrev) * ...
                rotZ(thetaCurrent) * transZ(dCurrent);
            T = T * A;
        end
    end

    function table = dhTable(q)
        link = cfg.link;

        table = [
            1,  0,      0,        link.L1, q(1);
            2,  pi/2,  -link.b,   0,       q(2) + pi/2;
            3,  0,      link.L2,  0,       q(3);
            4,  0,      link.L3,  0,       q(4) - pi/2;
            5, -pi/2,   0,        link.L4, q(5)
        ];
    end

    function jointXYZ = jointPositions(q)
        q = rowN(q, cfg.motor.numJoints);
        table = dhTable(q);
        T = eye(4);
        jointXYZ = zeros(size(table, 1) + 1, 3);
        jointXYZ(1, :) = T(1:3, 4)';

        for frameIndex = 1:size(table, 1)
            alphaPrev = table(frameIndex, 2);
            aPrev = table(frameIndex, 3);
            dCurrent = table(frameIndex, 4);
            thetaCurrent = table(frameIndex, 5);

            A = rotX(alphaPrev) * transX(aPrev) * ...
                rotZ(thetaCurrent) * transZ(dCurrent);
            T = T * A;
            jointXYZ(frameIndex + 1, :) = T(1:3, 4)';
        end
    end

    function seeds = ikSeedList(qInitial)
        qInitial = rowN(qInitial, cfg.motor.numJoints);
        seeds = [
            qInitial;
            cfg.qBias;
            qInitial(1), -0.6, -0.8,  1.4, qInitial(5), qInitial(6);
            qInitial(1), -0.9, -0.6,  1.5, qInitial(5), qInitial(6);
            qInitial(1), -0.4, -1.0,  1.5, qInitial(5), qInitial(6);
            qInitial(1),  0.6, -1.2,  1.0, qInitial(5), qInitial(6);
            qInitial(1),  0.9, -1.4,  1.1, qInitial(5), qInitial(6)
        ];

        seeds = clamp(seeds, cfg.qMin, cfg.qMax);
    end

    function cost = ikSolutionCost(q, targetXYZ, qReference)
        T = fk(q);
        xyz = T(1:3, 4)';
        positionCost = norm(xyz - targetXYZ)^2;
        continuityCost = 0.001 * norm(rowN(q, cfg.motor.numJoints) - ...
            rowN(qReference, cfg.motor.numJoints))^2;
        elbowCost = cfg.ik.elbowPenaltyWeight * elbowViolation(q)^2;

        cost = positionCost + continuityCost + elbowCost;
    end

    function violation = elbowViolation(q)
        if ~cfg.ik.requireElbowAboveFrame5
            violation = 0;
            return;
        end

        jointXYZ = jointPositions(q);
        elbowZ = jointXYZ(4, 3);  % frame 3 origin
        frame5Z = jointXYZ(6, 3); % frame 5 origin / gripper centre
        violation = max(0, frame5Z + cfg.ik.elbowClearance - elbowZ);
    end

    function warnIfElbowDown(q)
        violation = elbowViolation(q);
        if violation > 0
            jointXYZ = jointPositions(q);
            warning(['IK pose is elbow-down: elbow z %.4f m, frame5 z %.4f m, ' ...
                'required clearance %.4f m.'], ...
                jointXYZ(4, 3), jointXYZ(6, 3), cfg.ik.elbowClearance);
        end
    end

    function assertPosePosition(label, q, targetXYZ, tolerance)
        T = fk(q);
        actualXYZ = T(1:3, 4)';
        targetXYZ = row3(targetXYZ) - cfg.baseOffsetXYZ - cfg.boardOffsetXYZ;
        err = norm(actualXYZ - targetXYZ);

        fprintf('%s pose position error: %.4f m\n', label, err);

        if err > tolerance
            error('%s pose IK is not accurate enough: %.4f m > %.4f m.', ...
                label, err, tolerance);
        end
    end

    function J = positionJacobian(q)
        q = rowN(q, cfg.motor.numJoints);
        J = zeros(3, cfg.motor.numJoints);
        T0 = fk(q);
        xyz0 = T0(1:3, 4);

        for jointIndex = 1:cfg.motor.numJoints
            qStepPlus = q;
            qStepMinus = q;
            qStepPlus(jointIndex) = min(q(jointIndex) + cfg.jacobianDelta, cfg.qMax(jointIndex));
            qStepMinus(jointIndex) = max(q(jointIndex) - cfg.jacobianDelta, cfg.qMin(jointIndex));
            actualDelta = qStepPlus(jointIndex) - qStepMinus(jointIndex);

            if actualDelta <= eps
                J(:, jointIndex) = zeros(3, 1);
            else
                TPlus = fk(qStepPlus);
                TMinus = fk(qStepMinus);
                J(:, jointIndex) = ...
                    (TPlus(1:3, 4) - TMinus(1:3, 4)) / actualDelta;
            end
        end
    end

    function J = positionFrame5AngleJacobian(q)
        q = rowN(q, cfg.motor.numJoints);
        q = clamp(q, cfg.qMin, cfg.qMax);

        J = zeros(4, cfg.arm.numKinematicJoints);

        for jointIndex = 1:cfg.arm.numKinematicJoints
            qStepPlus = q;
            qStepMinus = q;
            qStepPlus(jointIndex) = min(q(jointIndex) + cfg.jacobianDelta, cfg.qMax(jointIndex));
            qStepMinus(jointIndex) = max(q(jointIndex) - cfg.jacobianDelta, cfg.qMin(jointIndex));

            qStepPlus = clamp(qStepPlus, cfg.qMin, cfg.qMax);
            qStepMinus = clamp(qStepMinus, cfg.qMin, cfg.qMax);

            actualDelta = qStepPlus(jointIndex) - qStepMinus(jointIndex);

            if actualDelta <= eps
                J(:, jointIndex) = zeros(4, 1);
            else
                TPlus = fk(qStepPlus);
                TMinus = fk(qStepMinus);

                J(1:3, jointIndex) = ...
                    (TPlus(1:3, 4) - TMinus(1:3, 4)) / actualDelta;
                J(4, jointIndex) = ...
                    (frame5GroundAngle(qStepPlus) - frame5GroundAngle(qStepMinus)) / actualDelta;
            end
        end
    end

    function J = positionFrame5AxisJacobian(q)
        q = rowN(q, cfg.motor.numJoints);
        q = clamp(q, cfg.qMin, cfg.qMax);

        J = zeros(6, cfg.arm.numKinematicJoints);
        T0 = fk(q);
        xyz0 = T0(1:3, 4);
        axis0 = frame5Axis(q)';

        for jointIndex = 1:cfg.arm.numKinematicJoints
            qStepPlus = q;
            qStepMinus = q;
            qStepPlus(jointIndex) = min(q(jointIndex) + cfg.jacobianDelta, cfg.qMax(jointIndex));
            qStepMinus(jointIndex) = max(q(jointIndex) - cfg.jacobianDelta, cfg.qMin(jointIndex));

            qStepPlus = clamp(qStepPlus, cfg.qMin, cfg.qMax);
            qStepMinus = clamp(qStepMinus, cfg.qMin, cfg.qMax);

            actualDelta = qStepPlus(jointIndex) - qStepMinus(jointIndex);

            if actualDelta <= eps
                J(:, jointIndex) = zeros(6, 1);
            else
                TPlus = fk(qStepPlus);
                TMinus = fk(qStepMinus);
                axisPlus = frame5Axis(qStepPlus)';
                axisMinus = frame5Axis(qStepMinus)';

                J(1:3, jointIndex) = ...
                    (TPlus(1:3, 4) - TMinus(1:3, 4)) / actualDelta;
                J(4:6, jointIndex) = ...
                    (axisPlus - axisMinus) / actualDelta;
            end
        end
    end

    function setGripper(bot, gripperAngle)
        ensureOffsetApplied(bot);

        qCommand = readCurrentQ(bot);
        if cfg.gripper.motorIndex <= numel(qCommand)
            qCommand(cfg.gripper.motorIndex) = gripperAngle;
            bot.setQ(toMotorQ(qCommand));
            bot.driveMotors();
            pause(cfg.gripper.pause);
        else
            warning('Gripper motor index %d is outside q command length %d.', ...
                cfg.gripper.motorIndex, numel(qCommand));
        end
    end

    function waitUntilOpponentReady(bot)
        if ~cfg.task1.waitForOpponent
            fprintf('Skipping wait for opponent because waitForOpponent is false.\n');
            return;
        end

        fprintf('Waiting for opponent ready character "%s"...\n', ...
            cfg.task1.opponentReadyChar);

        while true
            readyChar = read(bot.serial_obj, 1, 'char');
            if strcmpi(readyChar, cfg.task1.opponentReadyChar)
                fprintf('Opponent ready received.\n');
                break;
            end
        end
    end

    function q = withGripperAngle(q, gripperAngle)
        if cfg.gripper.motorIndex <= numel(q)
            q(cfg.gripper.motorIndex) = gripperAngle;
        end
    end

    function qMotor = toMotorQ(qModel)
        qMotor = rowN(qModel, cfg.motor.numJoints) .* cfg.jointDirection;
    end

    function qModel = fromMotorQ(qMotor)
        qModel = rowN(qMotor, cfg.motor.numJoints) .* cfg.jointDirection;
    end

    function q = readCurrentQ(bot)
        ensureOffsetApplied(bot);

        if ismethod(bot, 'getFB')
            bot.getFB();
            q = fromMotorQ(bot.q_FB(1:cfg.motor.numJoints));
        else
            q = cfg.qHome;
        end
    end

    function ensureOffsetApplied(bot)
        if ~cfg.offset.autoApplyForMotion || offsetApplied
            return;
        end

        if exist(cfg.offset.file, "file") ~= 2
            if ~offsetWarningShown
                warning("No saved servo offset file found: %s", cfg.offset.file);
                offsetWarningShown = true;
            end
            return;
        end

        data = loadOffsetFile(cfg.offset.file);
        loadedOffset = rowN(data, bot.max_id);
        bot.setOffset(loadedOffset);
        motorOffset = loadedOffset(1:cfg.motor.numJoints);
        offsetApplied = true;

        fprintf("Auto-applied saved motor offset.\n");
    end
end

function T = rotX(theta)
T = [
    1,          0,           0, 0;
    0, cos(theta), -sin(theta), 0;
    0, sin(theta),  cos(theta), 0;
    0,          0,           0, 1
];
end

function T = rotZ(theta)
T = [
    cos(theta), -sin(theta), 0, 0;
    sin(theta),  cos(theta), 0, 0;
             0,           0, 1, 0;
             0,           0, 0, 1
];
end

function T = transX(distance)
T = eye(4);
T(1, 4) = distance;
end

function T = transZ(distance)
T = eye(4);
T(3, 4) = distance;
end

function y = clamp(x, lower, upper)
y = min(max(x, lower), upper);
end

function y = smoothStep(x)
y = x * x * (3 - 2 * x);
end

function y = row3(x)
y = reshape(x, 1, []);
if numel(y) ~= 3
    error('Expected a 3-element XYZ vector.');
end
end

function y = row2(x)
y = reshape(x, 1, []);
if numel(y) ~= 2
    error('Expected a 2-element XY vector.');
end
end

function y = rowN(x, n)
y = reshape(x, 1, []);
if numel(y) < n
    y = [y, zeros(1, n - numel(y))];
elseif numel(y) > n
    y = y(1:n);
end
end

function motorOffset = loadOffsetFile(offsetFile)
data = load(offsetFile);
if isfield(data, "motorOffset")
    motorOffset = data.motorOffset;
elseif isfield(data, "savedMotorOffset")
    motorOffset = data.savedMotorOffset;
else
    error("Offset file %s does not contain motorOffset or savedMotorOffset.", offsetFile);
end
end

function poolXYList = generatePoolXYList(centerXY, spacing, rows, cols)
centerXY = row2(centerXY);
poolXYList = zeros(rows * cols, 2);
index = 1;

for row = 1:rows
    for col = 1:cols
        xOffset = (col - (cols + 1) / 2) * spacing;
        yOffset = (row - (rows + 1) / 2) * spacing;
        poolXYList(index, :) = centerXY + [xOffset, yOffset];
        index = index + 1;
    end
end
end
