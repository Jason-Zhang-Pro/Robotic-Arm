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
L1 = 0.037;                 % Base height [m]
L2 = 0.17;                 % Link 2 length [m]
L3 = 0.13;                 % Link 3 length [m]
L4 = 0.16;                 % Tool / wrist offset [m]

baseOffsetXYZ = [0, 0, 0];        % Robot base in world/chessboard frame [m]
boardOffsetXYZ = [0, 0, 0];       % Later chessboard calibration offset [m]
homeXYZ = [0, 0, 0];              % Reserved only if Cartesian home is needed
frame1YCompensationAngle = -deg2rad(4);
                                  % Tilt compensation about frame 1 y-axis [rad].
                                  % Positive follows the right-hand rule about y1.

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
capturedPoolCenterXY = [-0.150, -0.200]; % Captured pool centre [m]
capturedPoolSpacing = 0.030;      % Spacing between captured pieces [m]
capturedPoolRows = 3;             % Number of generated captured pool rows
capturedPoolCols = 3;             % Number of generated captured pool columns

qHome = [-0.91169, -0.73631, 2.10616, 0.93573, -3.12472, 0];       % Joint-space home fallback [rad]
qBias = [0, 0, 0, 0, 0, 0];       % Preferred posture for IK [rad]
qMin = deg2rad([-150, -100, -140, -130, -180, -150]);
qMax = deg2rad([ 150,  100,  140,  130,  180,  150]);
jointDirection = [-1, 1, 1, 1, 1, 1];
                                  % Set a joint to -1 if its motor direction is reversed
useJointHome = true;              % Return to qHome instead of Cartesian homeXYZ

positionTolerance = 1e-3;         % IK target tolerance [m]
pickPlaceTolerance = 1e-3;        % Required final pick/place position tolerance [m]
ikMaxIterations = 300;
ikDamping = 0.02;                 % Damped least-squares IK damping
ikStepGain = 0.7;                 % Scale applied to each Jacobian update
ikMaxJointStep = deg2rad(5);      % Max joint change per IK iteration [rad]
jacobianDelta = 1e-5;             % Numerical Jacobian finite difference [rad]
requireElbowAboveFrame5 = true;   % Reject elbow-down IK poses
elbowClearance = 0.005;           % Elbow must be this much above frame 5 origin [m]
requireFrame4AboveFrame5 = true;  % Frame 4 origin must not be lower than frame 5 origin
frame4Clearance = 0.000;          % Extra required clearance from frame 4 to frame 5 [m]
elbowPenaltyWeight = 1000;        % Cost penalty when elbow is too low
frame5AxisToHold = 3;             % 1=x5, 2=y5, 3=z5 axis used for ground angle
frame5AngleWeight = 1.0;          % Weight for frame5-ground angle error in constrained IK
frame5AngleSearchMin = deg2rad(-90);
frame5AngleSearchMax = deg2rad(85);
frame5AngleSearchStep = deg2rad(5);
frame5PreferredGroundAngle = frame5AngleSearchMin;
                                  % Prefer selected frame5 axis as vertical/downward as search allows
frame5AngleTolerance = deg2rad(2);
restrictQ5ToInitialOr180 = true;  % q5 may only stay at initial angle or flip 180 deg
q5FlipAngle = pi;                 % Allowed q5 flip from initial angle [rad]

numSteps = 1;                     % Minimum interpolation steps
stepPause = 0.01;                  % Pause between commands [s]
maxEndEffectorSpeed = 1;       % Max frame5-origin speed during point-to-point moves [m/s]
maxEndEffectorSpeedLower = 0.10;  % Max speed when lowering to pick/place [m/s]
maxEndEffectorSpeedLift = 0.10;   % Max speed when lifting away from pick/place [m/s]
maxEndEffectorSpeedMove = 0.50;   % Max speed when moving at safe height [m/s]
maxQ5CommandStep = deg2rad(10); % Max q5 command change per step [rad]
q5ExtraSettleTime = 0.20;         % Extra wait after moves with q5 rotation [s]
gripZ = boardZ + 0.035;                   % Gripping / placing height [m]
moveZ = boardZ + 0.100;           % Safe travel height while moving [m]
defaultMoveTime = 1.2;            % Reserved for future timing logic [s]

numJoints = 6;
driveMode = zeros(1, numJoints);  % 0 = position mode
wrapJointCommands = false;        % Disabled: do not wrap angles before commanding

gripperOpenAngle = deg2rad(40);   % Gripper open command angle [rad]
gripperCloseAngle = deg2rad(-10);   % Gripper closed command angle [rad]
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
cfg.compensation.frame1YAngle = frame1YCompensationAngle;
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
cfg.ik.requireFrame4AboveFrame5 = requireFrame4AboveFrame5;
cfg.ik.frame4Clearance = frame4Clearance;
cfg.ik.elbowPenaltyWeight = elbowPenaltyWeight;
cfg.frame5.axisToHold = frame5AxisToHold;
cfg.frame5.angleWeight = frame5AngleWeight;
cfg.frame5.axisWeight = frame5AngleWeight;  % Compatibility for optional full-axis IK helper.
cfg.frame5.preferredGroundAngle = frame5PreferredGroundAngle;
cfg.frame5.searchMin = frame5AngleSearchMin;
cfg.frame5.searchMax = frame5AngleSearchMax;
cfg.frame5.searchStep = frame5AngleSearchStep;
cfg.frame5.angleTolerance = frame5AngleTolerance;
cfg.q5.restrictToInitialOr180 = restrictQ5ToInitialOr180;
cfg.q5.flipAngle = q5FlipAngle;

cfg.motion.numSteps = numSteps;
cfg.motion.stepPause = stepPause;
cfg.motion.maxEndEffectorSpeed = maxEndEffectorSpeed;
cfg.motion.maxEndEffectorSpeedLower = maxEndEffectorSpeedLower;
cfg.motion.maxEndEffectorSpeedLift = maxEndEffectorSpeedLift;
cfg.motion.maxEndEffectorSpeedMove = maxEndEffectorSpeedMove;
cfg.motion.maxQ5CommandStep = maxQ5CommandStep;
cfg.motion.q5ExtraSettleTime = q5ExtraSettleTime;
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
arm.precomputeTask1IK = @precomputeTask1IK;
arm.executeCachedTask1 = @executeCachedTask1;
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
arm.frame1YCompensationTransform = @frame1YCompensationTransform;
arm.ikXYZHoldFrame5Angle = @ikXYZHoldFrame5Angle;
arm.ikXYZHoldFrame5Axis = @ikXYZHoldFrame5Axis;
arm.frame5GroundAngle = @frame5GroundAngle;
arm.absoluteFrame5Angle = @frame5GroundAngle;

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
        moveJoints(bot, qStart, q1, 'lift');

        q2 = ikXYZ(safeTargetXYZ, q1);
        moveJoints(bot, q1, q2, 'move');

        q3 = ikXYZ([x, y, z], q2);
        moveJoints(bot, q2, q3, 'lower');
    end

    function pickAndPlaceXY(bot, startXY, finalXY, qStart, q5Mode)
        if nargin < 4 || isempty(qStart)
            qStart = readCurrentQ(bot);
        end
        if nargin < 5
            q5Mode = "default";
        end

        startXY = row2(startXY);
        finalXY = row2(finalXY);

        pickXYZ = [startXY, cfg.motion.gripZ];
        placeXYZ = [finalXY, cfg.motion.gripZ];
        pickAboveXYZ = [startXY, cfg.motion.moveZ];
        placeAboveXYZ = [finalXY, cfg.motion.moveZ];

        plan = planPickAndPlaceXY(startXY, finalXY, qStart, q5Mode);
        qPick = plan.qPick;
        qPlace = plan.qPlace;
        pickAboveXYZ = plan.pickAboveXYZ;
        placeAboveXYZ = plan.placeAboveXYZ;
        assertPosePosition('pick', qPick, pickXYZ, cfg.pickPlaceTolerance);
        assertPosePosition('place', qPlace, placeXYZ, cfg.pickPlaceTolerance);

        qAbovePick = matchApproachWrist( ...
            withGripperAngle(ikXYZ(pickAboveXYZ, qStart), cfg.gripper.openAngle), ...
            qPick);
        moveJoints(bot, qStart, qAbovePick, 'move');

        setGripper(bot, cfg.gripper.openAngle);
        qPick = withGripperAngle(qPick, cfg.gripper.openAngle);
        moveJoints(bot, qAbovePick, qPick, 'lower');
        setGripper(bot, cfg.gripper.closeAngle);
        qPick = withGripperAngle(qPick, cfg.gripper.closeAngle);

        qLift = matchApproachWrist( ...
            withGripperAngle(ikXYZ(pickAboveXYZ, qPick), cfg.gripper.closeAngle), ...
            qPick);
        moveJoints(bot, qPick, qLift, 'lift');

        qAbovePlace = matchApproachWrist( ...
            withGripperAngle(ikXYZ(placeAboveXYZ, qLift), cfg.gripper.closeAngle), ...
            qPlace);
        moveJoints(bot, qLift, qAbovePlace, 'move');

        qPlace = withGripperAngle(qPlace, cfg.gripper.closeAngle);
        moveJoints(bot, qAbovePlace, qPlace, 'lower');
        setGripper(bot, cfg.gripper.openAngle);
        qPlace = withGripperAngle(qPlace, cfg.gripper.openAngle);

        qRetreat = matchApproachWrist( ...
            withGripperAngle(ikXYZ(placeAboveXYZ, qPlace), cfg.gripper.openAngle), ...
            qPlace);
        moveJoints(bot, qPlace, qRetreat, 'lift');
    end

    function plan = planPickAndPlaceXY(startXY, finalXY, qStart, q5Mode)
        if nargin < 3 || isempty(qStart)
            qStart = cfg.qBias;
        end
        if nargin < 4
            q5Mode = "default";
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
            chooseSharedFrame5GroundAngle(pickXYZ, placeXYZ, qPickFree, qPlaceFree, qStart(5), q5Mode);

        plan = struct();
        plan.pickXYZ = pickXYZ;
        plan.placeXYZ = placeXYZ;
        plan.pickAboveXYZ = pickAboveXYZ;
        plan.placeAboveXYZ = placeAboveXYZ;
        plan.qPickFree = qPickFree;
        plan.qPlaceFree = qPlaceFree;
        plan.qPick = qPick;
        plan.qPlace = qPlace;
        plan.qAbovePick = matchApproachWrist(ikXYZ(pickAboveXYZ, qStart), qPick);
        plan.qAbovePlace = matchApproachWrist(ikXYZ(placeAboveXYZ, plan.qAbovePick), qPlace);
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

    function task1Plan = precomputeTask1IK()
        moves = task1Moves();
        cfg.capture.nextIndex = 1;

        qHomeCurrent = homeConfiguration(cfg.qHome);
        planMoves = repmat(emptyCachedTask1Move(), 1, numel(moves));

        fprintf('Precomputing Task 1 IK for %d robot moves...\n', numel(moves));
        for moveIndex = 1:numel(moves)
            move = moves(moveIndex);
            fprintf('Precompute %02d/%02d: %s -> %s\n', ...
                moveIndex, numel(moves), move.from, move.to);

            startXY = squareToXY(move.from);
            if strcmpi(move.to, 'captured pool')
                finalXY = nextCapturedPoolXY();
            else
                finalXY = squareToXY(move.to);
            end

            movePlan = planPickAndPlaceXY(startXY, finalXY, qHomeCurrent);
            qRetreat = matchApproachWrist(ikXYZ(movePlan.placeAboveXYZ, movePlan.qPlace), ...
                movePlan.qPlace);
            qHomeAfter = homeConfiguration(qRetreat);

            planMoves(moveIndex).index = moveIndex;
            planMoves(moveIndex).from = move.from;
            planMoves(moveIndex).to = move.to;
            planMoves(moveIndex).waitAfter = move.waitAfter;
            planMoves(moveIndex).startXY = startXY;
            planMoves(moveIndex).finalXY = finalXY;
            planMoves(moveIndex).qHomeBefore = qHomeCurrent;
            planMoves(moveIndex).qAbovePick = movePlan.qAbovePick;
            planMoves(moveIndex).qPick = movePlan.qPick;
            planMoves(moveIndex).qLift = movePlan.qAbovePick;
            planMoves(moveIndex).qAbovePlace = movePlan.qAbovePlace;
            planMoves(moveIndex).qPlace = movePlan.qPlace;
            planMoves(moveIndex).qRetreat = qRetreat;
            planMoves(moveIndex).qHomeAfter = qHomeAfter;
            planMoves(moveIndex).frame5AngleHold = movePlan.frame5AngleHold;
            planMoves(moveIndex).pickErr = posePositionError(movePlan.qPick, movePlan.pickXYZ);
            planMoves(moveIndex).placeErr = posePositionError(movePlan.qPlace, movePlan.placeXYZ);

            qHomeCurrent = qHomeAfter;
        end

        task1Plan = struct();
        task1Plan.generatedAt = string(datetime('now'));
        task1Plan.config = cfg;
        task1Plan.moves = planMoves;
        task1Plan.numMoves = numel(planMoves);
        fprintf('Task 1 IK precompute complete.\n');
    end

    function executeCachedTask1(bot, task1Plan)
        if isfield(task1Plan, 'task1Plan')
            task1Plan = task1Plan.task1Plan;
        end
        if ~isfield(task1Plan, 'moves')
            error('Cached Task 1 plan must contain a moves field.');
        end

        if cfg.task1.autoApplySavedOffset
            applySavedOffset(bot);
        end

        reply = input('Type s to start cached Task 1: ', 's');
        testMode = strcmpi(strtrim(reply), 't');
        if ~strcmpi(strtrim(reply), 's') && ~testMode
            fprintf('Cancelled. No motion command was sent.\n');
            return;
        end

        qCurrent = readCurrentQ(bot);
        qHomeStart = rowN(task1Plan.moves(1).qHomeBefore, cfg.motor.numJoints);
        moveJoints(bot, qCurrent, qHomeStart, 'move');

        for moveIndex = 1:numel(task1Plan.moves)
            cachedMove = task1Plan.moves(moveIndex);
            fprintf('Cached Task 1 move %02d/%02d: %s -> %s\n', ...
                moveIndex, numel(task1Plan.moves), cachedMove.from, cachedMove.to);
            executeCachedPickAndPlace(bot, cachedMove);

            qAfterMove = readCurrentQ(bot);
            moveJoints(bot, qAfterMove, rowN(cachedMove.qHomeAfter, cfg.motor.numJoints), 'move');

            if cachedMove.waitAfter
                if testMode
                    fprintf('Returned home. Test mode: continuing without waiting for r.\n');
                else
                    reply = input('Returned home. Type r after opponent move: ', 's');
                    while ~strcmpi(strtrim(reply), 'r')
                        reply = input('Waiting for r: ', 's');
                    end
                end
            end
        end

        fprintf('Cached Task 1 complete.\n');
    end

    function executeCachedPickAndPlace(bot, cachedMove)
        qCurrent = readCurrentQ(bot);
        qAbovePick = withGripperAngle(rowN(cachedMove.qAbovePick, cfg.motor.numJoints), ...
            cfg.gripper.openAngle);
        qPickOpen = withGripperAngle(rowN(cachedMove.qPick, cfg.motor.numJoints), ...
            cfg.gripper.openAngle);
        qPickClosed = withGripperAngle(rowN(cachedMove.qPick, cfg.motor.numJoints), ...
            cfg.gripper.closeAngle);
        qLiftClosed = withGripperAngle(rowN(cachedMove.qLift, cfg.motor.numJoints), ...
            cfg.gripper.closeAngle);
        qAbovePlaceClosed = withGripperAngle(rowN(cachedMove.qAbovePlace, cfg.motor.numJoints), ...
            cfg.gripper.closeAngle);
        qPlaceClosed = withGripperAngle(rowN(cachedMove.qPlace, cfg.motor.numJoints), ...
            cfg.gripper.closeAngle);
        qPlaceOpen = withGripperAngle(rowN(cachedMove.qPlace, cfg.motor.numJoints), ...
            cfg.gripper.openAngle);
        qRetreatOpen = withGripperAngle(rowN(cachedMove.qRetreat, cfg.motor.numJoints), ...
            cfg.gripper.openAngle);

        moveJoints(bot, qCurrent, qAbovePick, 'move');
        setGripper(bot, cfg.gripper.openAngle);
        moveJoints(bot, qAbovePick, qPickOpen, 'lower');
        setGripper(bot, cfg.gripper.closeAngle);
        moveJoints(bot, qPickClosed, qLiftClosed, 'lift');
        moveJoints(bot, qLiftClosed, qAbovePlaceClosed, 'move');
        moveJoints(bot, qAbovePlaceClosed, qPlaceClosed, 'lower');
        setGripper(bot, cfg.gripper.openAngle);
        moveJoints(bot, qPlaceOpen, qRetreatOpen, 'lift');
    end

    function move = emptyCachedTask1Move()
        move = struct( ...
            'index', [], ...
            'from', "", ...
            'to', "", ...
            'waitAfter', false, ...
            'startXY', [], ...
            'finalXY', [], ...
            'qHomeBefore', [], ...
            'qAbovePick', [], ...
            'qPick', [], ...
            'qLift', [], ...
            'qAbovePlace', [], ...
            'qPlace', [], ...
            'qRetreat', [], ...
            'qHomeAfter', [], ...
            'frame5AngleHold', [], ...
            'pickErr', [], ...
            'placeErr', []);
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

    function moveJoints(bot, qStart, qTarget, segmentType)
        if nargin < 4 || isempty(segmentType)
            segmentType = 'move';
        end

        ensureOffsetApplied(bot);

        qStart = rowN(qStart, cfg.motor.numJoints);
        qTarget = rowN(qTarget, cfg.motor.numJoints);
        segmentSpeed = speedForSegment(segmentType);

        qDelta = qTarget - qStart;
        armJoints = 1:cfg.arm.numKinematicJoints;
        maxDelta = max(abs(qDelta(armJoints)));
        eePathLength = estimateEndEffectorPathLength(qStart, qTarget);
        maxEndEffectorStep = max(segmentSpeed * cfg.motion.stepPause, eps);
        eeLimitedSteps = ceil(eePathLength / maxEndEffectorStep);
        q5LimitedSteps = ceil(abs(qDelta(5)) / max(cfg.motion.maxQ5CommandStep, eps));
        effectiveSteps = max([cfg.motion.numSteps, eeLimitedSteps, q5LimitedSteps]);
        estimatedDuration = effectiveSteps * cfg.motion.stepPause;

        fprintf(['moveJoints: steps=%d, stepPause=%.3f s, estimatedDuration=%.2f s, ' ...
            'segment=%s, eePath=%.4f m, maxEeSpeed=%.3f m/s, maxDelta=%.2f deg, q5Delta=%.2f deg\n'], ...
            effectiveSteps, cfg.motion.stepPause, estimatedDuration, ...
            char(segmentType), eePathLength, segmentSpeed, ...
            rad2deg(maxDelta), rad2deg(abs(qDelta(5))));

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

        if abs(qDelta(5)) > cfg.motion.maxQ5CommandStep && cfg.motion.q5ExtraSettleTime > 0
            pause(cfg.motion.q5ExtraSettleTime);
        end
    end

    function speed = speedForSegment(segmentType)
        switch lower(char(segmentType))
            case {'lower', 'down', 'descend'}
                speed = cfg.motion.maxEndEffectorSpeedLower;
            case {'lift', 'up', 'raise'}
                speed = cfg.motion.maxEndEffectorSpeedLift;
            case {'move', 'travel', 'horizontal'}
                speed = cfg.motion.maxEndEffectorSpeedMove;
            otherwise
                warning('Unknown segment type "%s"; using default end-effector speed.', ...
                    char(segmentType));
                speed = cfg.motion.maxEndEffectorSpeed;
        end
    end

    function pathLength = estimateEndEffectorPathLength(qStart, qTarget)
        qStart = rowN(qStart, cfg.motor.numJoints);
        qTarget = rowN(qTarget, cfg.motor.numJoints);
        qDelta = qTarget - qStart;

        sampleCount = max(10, ceil(max(abs(qDelta(1:cfg.arm.numKinematicJoints))) / deg2rad(5)));
        pathLength = 0;
        TPrev = fk(qStart);
        xyzPrev = TPrev(1:3, 4)';

        for sampleIndex = 1:sampleCount
            s = sampleIndex / sampleCount;
            qSample = qStart + s * qDelta;
            TSample = fk(qSample);
            xyzSample = TSample(1:3, 4)';
            pathLength = pathLength + norm(xyzSample - xyzPrev);
            xyzPrev = xyzSample;
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

    function T = frame1YCompensationTransform()
        T = rotY(cfg.compensation.frame1YAngle);
    end

    function values = allowedQ5Values(q5Reference, q5Mode)
        if nargin < 2
            q5Mode = "default";
        end

        if ~cfg.q5.restrictToInitialOr180
            values = [];
            return;
        end

        q5Min = cfg.qMin(5);
        q5Max = cfg.qMax(5);
        q5Reference = min(max(q5Reference, q5Min), q5Max);
        if strcmpi(string(q5Mode), "flip")
            rawValues = [q5Reference + cfg.q5.flipAngle, ...
                q5Reference - cfg.q5.flipAngle];
        else
            rawValues = [q5Reference, ...
                q5Reference + cfg.q5.flipAngle, ...
                q5Reference - cfg.q5.flipAngle];
        end

        values = [];
        for valueIndex = 1:numel(rawValues)
            value = rawValues(valueIndex);
            if value < q5Min - 1e-9 || value > q5Max + 1e-9
                continue;
            end
            if isempty(values) || all(abs(values - value) > 1e-6)
                values(end + 1) = value; %#ok<AGROW>
            end
        end

        if isempty(values)
            values = q5Reference;
        end
    end

    function [qPick, qPlace, angleHold, stats] = chooseSharedFrame5GroundAngle( ...
            pickXYZ, placeXYZ, qPickInitial, qPlaceInitial, q5Reference, q5Mode)
        if nargin < 5 || isempty(q5Reference)
            q5Reference = qPickInitial(5);
        end
        if nargin < 6
            q5Mode = "default";
        end

        pickTarget = row3(pickXYZ) - cfg.baseOffsetXYZ - cfg.boardOffsetXYZ;
        placeTarget = row3(placeXYZ) - cfg.baseOffsetXYZ - cfg.boardOffsetXYZ;
        q5AllowedValues = allowedQ5Values(q5Reference, q5Mode);
        placeMustFlipFromPick = strcmpi(string(q5Mode), "placeFlip");
        if placeMustFlipFromPick
            q5AllowedValues = allowedQ5Values(q5Reference, "default");
        end

        freePickAngle = frame5GroundAngle(qPickInitial);
        freePlaceAngle = frame5GroundAngle(qPlaceInitial);
        coarseAngles = cfg.frame5.searchMin:cfg.frame5.searchStep:cfg.frame5.searchMax;
        if isempty(coarseAngles) || coarseAngles(end) < cfg.frame5.searchMax
            coarseAngles = [coarseAngles, cfg.frame5.searchMax];
        end
        candidateAngles = sortAnglesByVertical(clipAngles(unique([ ...
            coarseAngles, ...
            cfg.frame5.preferredGroundAngle, ...
            freePickAngle, ...
            freePlaceAngle, ...
            0, ...
            deg2rad([-15, 15, -30, 30, -45, 45, -60, -75, -85, -90])], 'stable')));

        bestFeasibleCost = inf;
        bestAnyCost = inf;
        bestFeasible = [];
        bestAny = [];

        for angleIndex = 1:numel(candidateAngles)
            candidateAngle = candidateAngles(angleIndex);
            if placeMustFlipFromPick
                pickQ5Candidates = q5AllowedValues;
            else
                pickQ5Candidates = nan;
            end

            for pickQ5Index = 1:numel(pickQ5Candidates)
                if placeMustFlipFromPick
                    pickAllowedQ5 = pickQ5Candidates(pickQ5Index);
                else
                    pickAllowedQ5 = q5AllowedValues;
                end

                [candidatePick, pickErr, pickAngleErr] = ...
                    solveBestFrame5AngleTarget(pickTarget, candidateAngle, ...
                    qPickInitial, pickAllowedQ5);
                if placeMustFlipFromPick
                    placeAllowedQ5 = allowedQ5Values(candidatePick(5), "flip");
                else
                    placeAllowedQ5 = q5AllowedValues;
                end
                [candidatePlace, placeErr, placeAngleErr] = ...
                    solveBestFrame5AngleTarget(placeTarget, candidateAngle, ...
                    qPlaceInitial, placeAllowedQ5);

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
                    'q5AllowedValues', q5AllowedValues, ...
                    'placeAllowedQ5', placeAllowedQ5, ...
                    'feasible', feasible);

                if feasible && feasibleCost < bestFeasibleCost
                    bestFeasibleCost = feasibleCost;
                    bestFeasible = candidate;
                    break;
                end
                if anyCost < bestAnyCost
                    bestAnyCost = anyCost;
                    bestAny = candidate;
                end
            end

            if ~isempty(bestFeasible)
                break;
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
        fprintf(['Selected shared frame5 axis %d ground angle: %.4f rad / %.2f deg ' ...
            '(pick %.4f m, place %.4f m)\n'], ...
            cfg.frame5.axisToHold, angleHold, rad2deg(angleHold), ...
            chosen.pickErr, chosen.placeErr);
        if cfg.q5.restrictToInitialOr180
            fprintf('Allowed q5 angles: %s deg; selected pick/place q5: %.2f / %.2f deg\n', ...
                mat2str(rad2deg(q5AllowedValues), 3), ...
                rad2deg(qPick(5)), rad2deg(qPlace(5)));
        end
    end

    function angles = clipAngles(angles)
        angles = angles(angles >= cfg.frame5.searchMin & angles <= cfg.frame5.searchMax);
        angles = unique(angles, 'stable');
    end

    function angles = sortAnglesByVertical(angles)
        [~, order] = sort(abs(angles - cfg.frame5.preferredGroundAngle), 'ascend');
        angles = angles(order);
    end

    function q = ikXYZHoldFrame5Angle(targetXYZ, frame5AngleHold, qInitial)
        if nargin < 3 || isempty(qInitial)
            qInitial = cfg.qBias;
        end

        targetXYZ = row3(targetXYZ) - cfg.baseOffsetXYZ - cfg.boardOffsetXYZ;
        [q, err, angleErr] = solveBestFrame5AngleTarget( ...
            targetXYZ, frame5AngleHold, qInitial, []);
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
            targetXYZ, frame5AngleHold, qInitial, q5AllowedValues)
        if nargin < 4
            q5AllowedValues = [];
        end

        seedList = ikSeedList(qInitial);
        qBest = seedList(1, :);
        bestCost = inf;
        errBest = inf;
        angleErrBest = inf;

        if isempty(q5AllowedValues)
            q5AllowedValues = nan;
        end

        for q5Index = 1:numel(q5AllowedValues)
            fixedQ5 = q5AllowedValues(q5Index);
            for seedIndex = 1:size(seedList, 1)
                seed = seedList(seedIndex, :);
                if ~isnan(fixedQ5)
                    seed(5) = fixedQ5;
                end

                qCandidate = solveIkXYZHoldFrame5AngleFromSeed( ...
                    targetXYZ, frame5AngleHold, seed, fixedQ5);
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
    end

    function q = solveIkXYZHoldFrame5AngleFromSeed(targetXYZ, frame5AngleHold, qSeed, fixedQ5)
        if nargin < 4
            fixedQ5 = nan;
        end

        q = clamp(rowN(qSeed, cfg.motor.numJoints), cfg.qMin, cfg.qMax);
        if ~isnan(fixedQ5)
            q(5) = fixedQ5;
        end

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
            if ~isnan(fixedQ5)
                J(:, 5) = 0;
            end
            lambda = cfg.ikDamping;
            dq = (J' / (J * J' + lambda^2 * eye(4))) * errorTask';
            dq = cfg.ikStepGain * dq';
            dq = clamp(dq, -cfg.ikMaxJointStep, cfg.ikMaxJointStep);
            if ~isnan(fixedQ5)
                dq(5) = 0;
            end

            q(1:cfg.arm.numKinematicJoints) = ...
                q(1:cfg.arm.numKinematicJoints) + ...
                dq(1:cfg.arm.numKinematicJoints);
            q = clamp(q, cfg.qMin, cfg.qMax);
            if ~isnan(fixedQ5)
                q(5) = fixedQ5;
            end
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
            if i == 1
                T = T * frame1YCompensationTransform();
            end
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
            if frameIndex == 1
                T = T * frame1YCompensationTransform();
            end
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
        frame3Z = jointXYZ(4, 3); % frame 3 origin / elbow
        frame4Z = jointXYZ(5, 3); % frame 4 origin / wrist
        frame5Z = jointXYZ(6, 3); % frame 5 origin / gripper centre

        frame3Violation = max(0, frame5Z + cfg.ik.elbowClearance - frame3Z);
        if isfield(cfg.ik, 'requireFrame4AboveFrame5') && cfg.ik.requireFrame4AboveFrame5
            if isfield(cfg.ik, 'frame4Clearance')
                frame4Clearance = cfg.ik.frame4Clearance;
            else
                frame4Clearance = 0;
            end
            frame4Violation = max(0, frame5Z + frame4Clearance - frame4Z);
        else
            frame4Violation = 0;
        end

        violation = hypot(frame3Violation, frame4Violation);
    end

    function warnIfElbowDown(q)
        violation = elbowViolation(q);
        if violation > 0
            jointXYZ = jointPositions(q);
            warning(['IK pose violates height rule: frame3 z %.4f m, ' ...
                'frame4 z %.4f m, frame5 z %.4f m. Required clearances: ' ...
                'frame3-frame5 %.4f m, frame4-frame5 %.4f m.'], ...
                jointXYZ(4, 3), jointXYZ(5, 3), jointXYZ(6, 3), ...
                cfg.ik.elbowClearance, getFrame4Clearance());
        end
    end

    function clearance = getFrame4Clearance()
        if isfield(cfg.ik, 'frame4Clearance')
            clearance = cfg.ik.frame4Clearance;
        else
            clearance = 0;
        end
    end

    function assertPosePosition(label, q, targetXYZ, tolerance)
        err = posePositionError(q, targetXYZ);

        fprintf('%s pose position error: %.4f m\n', label, err);

        if err > tolerance
            error('%s pose IK is not accurate enough: %.4f m > %.4f m.', ...
                label, err, tolerance);
        end
    end

    function err = posePositionError(q, targetXYZ)
        T = fk(q);
        actualXYZ = T(1:3, 4)';
        targetXYZ = row3(targetXYZ) - cfg.baseOffsetXYZ - cfg.boardOffsetXYZ;
        err = norm(actualXYZ - targetXYZ);
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

    function q = matchApproachWrist(q, qTarget)
        q = rowN(q, cfg.motor.numJoints);
        qTarget = rowN(qTarget, cfg.motor.numJoints);
        q(5) = qTarget(5);
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

function T = rotY(theta)
T = [
     cos(theta), 0, sin(theta), 0;
              0, 1,          0, 0;
    -sin(theta), 0, cos(theta), 0;
              0, 0,          0, 1
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
