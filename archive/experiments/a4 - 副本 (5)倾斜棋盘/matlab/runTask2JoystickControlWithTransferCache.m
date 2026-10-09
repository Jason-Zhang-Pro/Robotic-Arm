function runTask2JoystickControlWithTransferCache()
%% runTask2JoystickControlWithTransferCache.m
% Cached joystick control for Task 2 robot-assisted chess moves.
%
% Flow:
%   1. Move robot to qHome.
%   2. First joystick press moves to D1 above the board with gripper open.
%   3. Push joystick freely to move above the piece to pick.
%   4. Next joystick press descends, closes gripper, and lifts.
%   5. Push joystick to move the held piece subject to pieceType.
%   6. Next joystick press places the piece and lifts.

scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptDir);
cd(projectRoot);

%% User variables
port = "COM5";
joystickCacheFile = fullfile(scriptDir, "task2_joystick_ik_cache.mat");
transferCacheFile = fullfile(scriptDir, "task2_transfer_ik_cache.mat");
startSquare = "D1";
manualPieceType = "rook"; % Fallback if auto-detected piece is not supported.
samplePause = 0.08;
moveThreshold = 0.50;
deadband = 0.12;
adcMaxRaw = 687;
requireConfirmation = true;
useParallelPlacementMap = true;
debugJoystick = true;
debugPrintEvery = 5;
swapJoystickXY = true;
invertJoystickX = true;
invertJoystickY = true;

%% Setup
setup_a4
cleanupSerialConnections()
load("UoM-RS-Var_Def.mat", "MAX_ID", "BAUDRATE")
clear chessMoveModule
chessMoveModule

if exist(joystickCacheFile, "file") ~= 2
    error("Task 2 joystick IK cache not found. Run precomputeTask2JoystickIK first: %s", joystickCacheFile);
end

data = load(joystickCacheFile, "task2JoyPlan");
task2JoyPlan = data.task2JoyPlan;

if exist(transferCacheFile, "file") ~= 2
    error("Task 2 transfer IK cache not found. Run precomputeTask2TransferIK first: %s", transferCacheFile);
end

transferData = load(transferCacheFile, "task2TransferPlan");
task2TransferPlan = transferData.task2TransferPlan;
if ~isfield(task2TransferPlan, "fromCapturePoolTransferTable")
    error("Transfer cache does not contain capture pool -> board plans. Re-run precomputeTask2TransferIK.");
end

fprintf("Loaded Task 2 joystick cache:\n%s\n", joystickCacheFile);
fprintf("Loaded Task 2 transfer cache:\n%s\n", transferCacheFile);

if requireConfirmation
    fprintf("This script will MOVE the robot. First it will return to qHome.\n");
    reply = input('Type GO to start Task 2 joystick control: ', 's');
    if ~strcmp(reply, 'GO')
        fprintf("Cancelled. No motion command was sent.\n");
        return;
    end
end

%% Connect
bot = robot(port, MAX_ID, BAUDRATE);
bot.ConnectToArduino();
arm.applySavedOffset(bot);

bot.getID();
fprintf("Number of connected motors: %d\n", bot.num_ID);
fprintf("Connected motor IDs:\n");
disp(bot.connected_ID);

requiredArmIDs = 1:5;
connectedIDs = bot.connected_ID(1:bot.num_ID);
missingArmIDs = setdiff(requiredArmIDs, connectedIDs);
if ~isempty(missingArmIDs)
    error("Cannot run Task 2 joystick control. Missing required arm motor IDs: %s", ...
        mat2str(missingArmIDs));
end

if ~ismember(arm.config.gripper.motorIndex, connectedIDs)
    warning("Gripper motor ID %d was not detected. Gripper commands may not work.", ...
        arm.config.gripper.motorIndex);
end

%% Move to initial position
qCurrent = arm.readCurrentQ(bot);
qHome = withGripperAngle(arm.config.qHome, arm.config.gripper.openAngle, arm.config);
fprintf("Moving to qHome initial position...\n");
arm.moveJoints(bot, qCurrent, qHome, "move");
qCurrent = qHome;

centreRaw = readJoystickRaw(bot);
fprintf("Joystick centre sample [raw_sw raw_x raw_y]:\n");
disp(centreRaw);

[fileIndex, rankIndex] = squareToIndex(startSquare);
currentPose = task2JoyPlan.squarePoses(rankIndex, fileIndex);
isAtBoard = false;
isHolding = false;
activePieceType = "";
pickupPose = [];
sameAnglePlaceMap = [];
lastButton = false;
lastMoveTime = tic;
sampleIndex = 0;
lastBishopFileStep = 1;
lastBishopRankStep = 1;

fprintf("Ready. Press joystick to move to %s above the board and open gripper.\n", startSquare);
fprintf("Task 2 piece constraint will be detected from the initial chess setup.\n");

while true
    sampleIndex = sampleIndex + 1;
    raw = readJoystickRaw(bot);
    joy = normaliseJoystick(raw, centreRaw, deadband, adcMaxRaw, ...
        swapJoystickXY, invertJoystickX, invertJoystickY);
    buttonPressed = joy(3) == 1;
    buttonEdge = buttonPressed && ~lastButton;
    lastButton = buttonPressed;

    if debugJoystick && (buttonPressed || buttonEdge || mod(sampleIndex, debugPrintEvery) == 0)
        fprintf("joy raw=[%d %d %d] norm=[%+.2f %+.2f] button=%d edge=%d atBoard=%d holding=%d\n", ...
            raw(1), raw(2), raw(3), joy(1), joy(2), ...
            buttonPressed, buttonEdge, isAtBoard, isHolding);
    end

    if buttonEdge
        if ~isAtBoard
            currentPose = task2JoyPlan.squarePoses(rankIndex, fileIndex);
            fprintf("Button: moving to %s above board with gripper open.\n", currentPose.square);
            arm.moveJoints(bot, qCurrent, currentPose.qAboveOpen, "move");
            qCurrent = currentPose.qAboveOpen;
            isAtBoard = true;
            isHolding = false;
            lastMoveTime = tic;
            fprintf("Move above the piece with joystick. Press again to pick.\n");
        elseif ~isHolding
            fprintf("Button: picking at %s...\n", currentPose.square);
            arm.moveJoints(bot, qCurrent, currentPose.qGripOpen, "lower");
            arm.moveJoints(bot, currentPose.qGripOpen, currentPose.qGripClosed, "move");
            pause(arm.config.gripper.pause);
            arm.moveJoints(bot, currentPose.qGripClosed, currentPose.qAboveClosed, "lift");
            qCurrent = currentPose.qAboveClosed;
            isHolding = true;
            lastMoveTime = tic;
            pickupPose = currentPose;
            pickupFrame5Angle = arm.frame5GroundAngle(currentPose.qGripClosed);
            fprintf("Computing same-frame5 placement map from pickup angle %.2f deg...\n", ...
                rad2deg(pickupFrame5Angle));
            mapTimer = tic;
            sameAnglePlaceMap = computeSameFrame5PlacementMap( ...
                arm, task2JoyPlan, pickupFrame5Angle, currentPose.qGripClosed, ...
                projectRoot, useParallelPlacementMap);
            mapElapsed = toc(mapTimer);
            fprintf("Direct same-angle place squares: %d/64. Compute time: %.2f s\n", ...
                countPlaceableSquares(sameAnglePlaceMap), mapElapsed);
            activePieceType = initialSetupPieceType(currentPose.square);
            if ~isSupportedTask2Piece(activePieceType)
                warning("Initial square %s contains %s, which is not supported yet. Falling back to %s.", ...
                    currentPose.square, activePieceType, manualPieceType);
                activePieceType = manualPieceType;
            end
            fprintf("Pick complete at %s. Move with joystick, constrained as %s. Press again to place.\n", ...
                currentPose.square, activePieceType);
        else
            fprintf("Button: placing at %s...\n", currentPose.square);
            placePose = sameAnglePlaceMap(rankIndex, fileIndex);
            if placePose.canPlace
                arm.moveJoints(bot, qCurrent, placePose.qGripClosed, "lower");
                arm.moveJoints(bot, placePose.qGripClosed, placePose.qGripOpen, "move");
                pause(arm.config.gripper.pause);
                arm.moveJoints(bot, placePose.qGripOpen, placePose.qAboveOpen, "lift");
                qCurrent = placePose.qAboveOpen;
                fprintf("Direct same-angle place complete at %s.\n", currentPose.square);
            else
                fprintf("Cannot directly place at %s with pickup angle. Returning to %s, regripping, then placing target.\n", ...
                    currentPose.square, pickupPose.square);
                [qCurrent, currentPose] = placeViaCapturePoolTransferCache( ...
                    arm, bot, qCurrent, task2JoyPlan, task2TransferPlan, sameAnglePlaceMap, ...
                    pickupPose, rankIndex, fileIndex);
            end
            isHolding = false;
            activePieceType = "";
            sameAnglePlaceMap = [];
            pickupPose = [];
            lastMoveTime = tic;
            fprintf("Place sequence complete. Press Ctrl+C to stop, or move empty gripper and pick again.\n");
        end
    end

    if isAtBoard && toc(lastMoveTime) > 0.20
        if isHolding
            [dFile, dRank, lastBishopFileStep, lastBishopRankStep] = ...
                constrainedJoystickStep(joy, moveThreshold, activePieceType, ...
                lastBishopFileStep, lastBishopRankStep);
            moveLabel = activePieceType;
        else
            [dFile, dRank] = freeJoystickStep(joy, moveThreshold);
            moveLabel = "free";
        end

        if dFile ~= 0 || dRank ~= 0
            newFileIndex = fileIndex + dFile;
            newRankIndex = rankIndex + dRank;

            if newFileIndex < 1 || newFileIndex > 8 || ...
                    newRankIndex < 1 || newRankIndex > 8
                fprintf("Joystick %s move blocked at board edge.\n", moveLabel);
                lastMoveTime = tic;
            elseif newFileIndex ~= fileIndex || newRankIndex ~= rankIndex
                fileIndex = newFileIndex;
                rankIndex = newRankIndex;
                currentPose = task2JoyPlan.squarePoses(rankIndex, fileIndex);

                if isHolding
                    placePose = sameAnglePlaceMap(rankIndex, fileIndex);
                    if placePose.canMoveAbove
                        qTarget = placePose.qAboveClosed;
                    else
                        qTarget = currentPose.qAboveClosed;
                        fprintf("Warning: %s has no same-angle above pose; using fallback above pose.\n", ...
                            currentPose.square);
                    end
                else
                    qTarget = currentPose.qAboveOpen;
                end

                fprintf("Joystick %s move: moving to %s above board.\n", ...
                    moveLabel, currentPose.square);
                arm.moveJoints(bot, qCurrent, qTarget, "move");
                qCurrent = qTarget;
                lastMoveTime = tic;
            end
        end
    end

    pause(samplePause);
end
end

function raw = readJoystickRaw(bot)
writeline(bot.serial_obj, "JOY");
[raw, err] = readSerial(bot.serial_obj, 3, false);
if err
    pause(0.02);
    flush(bot.serial_obj);
    writeline(bot.serial_obj, "JOY");
    [raw, err] = readSerial(bot.serial_obj, 3, false);
    if err
        warning("Joystick serial read reported an error.");
    end
end
raw = reshape(raw, 1, []);
end

function joy = normaliseJoystick(raw, centreRaw, deadband, adcMaxRaw, ...
    swapJoystickXY, invertJoystickX, invertJoystickY)
joy = zeros(1, 3);

for axisIndex = 1:2
    rawIndex = axisIndex;
    delta = double(raw(rawIndex) - centreRaw(rawIndex));
    if delta >= 0
        scale = max(1, double(adcMaxRaw) - double(centreRaw(rawIndex)));
    else
        scale = max(1, double(centreRaw(rawIndex)));
    end

    value = delta / scale;
    value = max(-1, min(1, value));
    if abs(value) < deadband
        value = 0;
    end
    joy(axisIndex) = value;
end

if swapJoystickXY
    joy(1:2) = joy([2 1]);
end
if invertJoystickX
    joy(1) = -joy(1);
end
if invertJoystickY
    joy(2) = -joy(2);
end

joy(3) = raw(3) == 1;
end

function [dFile, dRank] = freeJoystickStep(joy, threshold)
% Free eight-direction grid step before the piece is picked up.
dFile = axisStep(joy(1), threshold);
dRank = axisStep(joy(2), threshold);
end

function [dFile, dRank, lastBishopFileStep, lastBishopRankStep] = ...
    constrainedJoystickStep(joy, threshold, pieceType, ...
    lastBishopFileStep, lastBishopRankStep)
pieceType = lower(string(pieceType));
rawFileStep = axisStep(joy(1), threshold);
rawRankStep = axisStep(joy(2), threshold);
dFile = 0;
dRank = 0;

switch pieceType
    case "rook"
        if rawFileStep == 0 && rawRankStep == 0
            return;
        elseif abs(joy(1)) >= abs(joy(2))
            dFile = rawFileStep;
        else
            dRank = rawRankStep;
        end

    case "bishop"
        if rawFileStep == 0 && rawRankStep == 0
            return;
        end
        if rawFileStep ~= 0
            lastBishopFileStep = rawFileStep;
        end
        if rawRankStep ~= 0
            lastBishopRankStep = rawRankStep;
        end
        dFile = lastBishopFileStep;
        dRank = lastBishopRankStep;

    case "knight"
        if rawFileStep == 0 && rawRankStep == 0
            return;
        end
        if rawFileStep == 0
            rawFileStep = signOrOne(joy(1));
        end
        if rawRankStep == 0
            rawRankStep = signOrOne(joy(2));
        end
        if abs(joy(1)) >= abs(joy(2))
            dFile = 2 * rawFileStep;
            dRank = rawRankStep;
        else
            dFile = rawFileStep;
            dRank = 2 * rawRankStep;
        end

    otherwise
        error("Unknown pieceType %s. Use rook, bishop, or knight.", pieceType);
end
end

function step = axisStep(value, threshold)
if value >= threshold
    step = 1;
elseif value <= -threshold
    step = -1;
else
    step = 0;
end
end

function step = signOrOne(value)
step = sign(value);
if step == 0
    step = 1;
end
end

function [fileIndex, rankIndex] = squareToIndex(squareName)
squareName = upper(char(squareName));
fileIndex = double(squareName(1)) - double('A') + 1;
rankIndex = str2double(squareName(2:end));

if fileIndex < 1 || fileIndex > 8 || rankIndex < 1 || rankIndex > 8
    error("Invalid square name: %s", string(squareName));
end
end

function placeMap = computeSameFrame5PlacementMap(arm, task2JoyPlan, frame5Angle, qSeed, ...
    projectRoot, useParallelPlacementMap)
placeMap = repmat(emptySameAnglePose(), 8, 8);

oldWarningState = warning;
warning("off", "all");

canUseParallel = useParallelPlacementMap && ...
    license("test", "Distrib_Computing_Toolbox") && ...
    exist("parpool", "file") == 2;

if canUseParallel
    pool = gcp("nocreate");
    if ~isempty(pool) && contains(class(pool), "ThreadPool")
        delete(pool);
        pool = [];
    end
    if isempty(pool)
        parpool("local");
    end

    rankRows = cell(1, 8);
    parfor rankIndex = 1:8
        rankRows{rankIndex} = computeSameFrame5PlacementRank( ...
            projectRoot, task2JoyPlan, frame5Angle, qSeed, rankIndex);
    end

    for rankIndex = 1:8
        placeMap(rankIndex, :) = rankRows{rankIndex};
    end
else
    for rankIndex = 1:8
        for fileIndex = 1:8
            basePose = task2JoyPlan.squarePoses(rankIndex, fileIndex);
            placeMap(rankIndex, fileIndex) = ...
                computeSameFrame5SquarePose(arm, basePose, frame5Angle, qSeed);
        end
    end
end

warning(oldWarningState);
end

function rankRow = computeSameFrame5PlacementRank(projectRoot, task2JoyPlan, ...
    frame5Angle, qSeed, rankIndex)
addpath(projectRoot);
addpath(fullfile(projectRoot, "external", "UoM-RS-Starter_Code"));
addpath(fullfile(projectRoot, "external", "UoM_RS_MATLAB_LIBRARIES"));
addpath(genpath(fullfile(projectRoot, "matlab")));
clear chessMoveModule
chessMoveModule

rankRow = repmat(emptySameAnglePose(), 1, 8);
warning("off", "all");

for fileIndex = 1:8
    basePose = task2JoyPlan.squarePoses(rankIndex, fileIndex);
    rankRow(fileIndex) = computeSameFrame5SquarePose( ...
        arm, basePose, frame5Angle, qSeed);
end
end

function pose = computeSameFrame5SquarePose(arm, basePose, frame5Angle, qSeed)
cfg = arm.config;
q5Seeds = [qSeed(5), qSeed(5) + pi];

[qGrip, gripErr, gripAngleErr] = bestFrame5Pose( ...
    arm, basePose.gripXYZ, frame5Angle, qSeed, q5Seeds);
[qAbove, aboveErr, aboveAngleErr] = bestFrame5Pose( ...
    arm, basePose.aboveXYZ, frame5Angle, qGrip, [qGrip(5), qGrip(5) + pi]);

pose = emptySameAnglePose();
pose.square = basePose.square;
pose.fileIndex = basePose.fileIndex;
pose.rankIndex = basePose.rankIndex;
pose.frame5Angle = frame5Angle;
pose.qAbove = qAbove;
pose.qGrip = qGrip;
pose.qAboveOpen = withGripperAngle(qAbove, cfg.gripper.openAngle, cfg);
pose.qGripOpen = withGripperAngle(qGrip, cfg.gripper.openAngle, cfg);
pose.qGripClosed = withGripperAngle(qGrip, cfg.gripper.closeAngle, cfg);
pose.qAboveClosed = withGripperAngle(qAbove, cfg.gripper.closeAngle, cfg);
pose.aboveErr = aboveErr;
pose.gripErr = gripErr;
pose.aboveAngleErr = aboveAngleErr;
pose.gripAngleErr = gripAngleErr;
pose.canMoveAbove = aboveErr <= cfg.pickPlaceTolerance && ...
    aboveAngleErr <= cfg.frame5.angleTolerance;
pose.canPlace = pose.canMoveAbove && ...
    gripErr <= cfg.pickPlaceTolerance && ...
    gripAngleErr <= cfg.frame5.angleTolerance;
end

function [qBest, errBest, angleErrBest] = bestFrame5Pose( ...
    arm, targetXYZ, frame5Angle, qSeed, q5Seeds)
qBest = [];
errBest = inf;
angleErrBest = inf;
bestCost = inf;

for seedIndex = 1:numel(q5Seeds)
    qTrySeed = qSeed;
    qTrySeed(5) = q5Seeds(seedIndex);
    qCandidate = arm.ikXYZHoldFrame5Angle(targetXYZ, frame5Angle, qTrySeed);

    err = posePositionError(arm, qCandidate, targetXYZ);
    angleErr = abs(frame5Angle - arm.frame5GroundAngle(qCandidate));
    cost = err + angleErr + 0.001 * norm(qCandidate - qTrySeed);

    if cost < bestCost
        bestCost = cost;
        qBest = qCandidate;
        errBest = err;
        angleErrBest = angleErr;
    end
end
end

function [qCurrent, currentPose] = placeViaCapturePoolTransferCache( ...
    arm, bot, qCurrent, task2JoyPlan, task2TransferPlan, sameAnglePlaceMap, ...
    pickupPose, targetRankIndex, targetFileIndex)
sourceRankIndex = pickupPose.rankIndex;
sourceFileIndex = pickupPose.fileIndex;
sourceReturnPose = sameAnglePlaceMap(sourceRankIndex, sourceFileIndex);
targetBasePose = task2JoyPlan.squarePoses(targetRankIndex, targetFileIndex);
poolIndex = chooseCapturePoolIndex(task2TransferPlan, sourceRankIndex, sourceFileIndex);
poolXY = task2TransferPlan.capturePoolXYList(poolIndex, :);

if ~sourceReturnPose.canPlace
    warning("Pickup square %s is not marked placeable with original pickup angle. Using cached source pose.", ...
        pickupPose.square);
    sourceReturnPose = cachedAsSameAnglePose(pickupPose, arm.config);
end

arm.moveJoints(bot, qCurrent, sourceReturnPose.qAboveClosed, "move");
arm.moveJoints(bot, sourceReturnPose.qAboveClosed, sourceReturnPose.qGripClosed, "lower");
arm.moveJoints(bot, sourceReturnPose.qGripClosed, sourceReturnPose.qGripOpen, "move");
pause(arm.config.gripper.pause);
arm.moveJoints(bot, sourceReturnPose.qGripOpen, sourceReturnPose.qAboveOpen, "lift");
qCurrent = sourceReturnPose.qAboveOpen;

captureTransfer = task2TransferPlan.capturePoolTransferTable( ...
    sourceRankIndex, sourceFileIndex, poolIndex);
if captureTransfer.isFeasible
    fprintf("Using cached transfer %s -> capture pool %d.\n", ...
        pickupPose.square, poolIndex);
    qCurrent = executeCachedTransfer(bot, arm, captureTransfer, qCurrent);
else
    warning("Cached transfer %s -> capture pool %d is not feasible. Using move module fallback.", ...
        pickupPose.square, poolIndex);
    arm.pickAndPlaceXY(bot, pickupPose.xy, poolXY, qCurrent);
    qCurrent = arm.readCurrentQ(bot);
end

fprintf("Regripping from capture pool %d and placing at %s with move module.\n", ...
    poolIndex, targetBasePose.square);
fromPoolTransfer = task2TransferPlan.fromCapturePoolTransferTable( ...
    poolIndex, targetRankIndex, targetFileIndex);
if fromPoolTransfer.isFeasible
    fprintf("Using cached transfer capture pool %d -> %s.\n", ...
        poolIndex, targetBasePose.square);
    qCurrent = executeCachedTransfer(bot, arm, fromPoolTransfer, qCurrent);
else
    warning("Cached transfer capture pool %d -> %s is not feasible. Using move module fallback.", ...
        poolIndex, targetBasePose.square);
    arm.pickAndPlaceXY(bot, poolXY, targetBasePose.xy, qCurrent);
    qCurrent = arm.readCurrentQ(bot);
end
currentPose = targetBasePose;
end

function poolIndex = chooseCapturePoolIndex(task2TransferPlan, fromRankIndex, fromFileIndex)
poolIndex = 1;
if ~isfield(task2TransferPlan, "capturePoolTransferTable")
    error("task2TransferPlan does not contain capturePoolTransferTable. Re-run precomputeTask2TransferIK.");
end

for candidateIndex = 1:size(task2TransferPlan.capturePoolTransferTable, 3)
    candidate = task2TransferPlan.capturePoolTransferTable( ...
        fromRankIndex, fromFileIndex, candidateIndex);
    if candidate.isFeasible
        poolIndex = candidateIndex;
        return;
    end
end
end

function qCurrent = executeCachedTransfer(bot, arm, transfer, qCurrent)
arm.moveJoints(bot, qCurrent, transfer.qAbovePick, "move");
arm.moveJoints(bot, transfer.qAbovePick, transfer.qPickOpen, "lower");
arm.moveJoints(bot, transfer.qPickOpen, transfer.qPickClosed, "move");
pause(arm.config.gripper.pause);
arm.moveJoints(bot, transfer.qPickClosed, transfer.qLiftClosed, "lift");
arm.moveJoints(bot, transfer.qLiftClosed, transfer.qAbovePlaceClosed, "move");
arm.moveJoints(bot, transfer.qAbovePlaceClosed, transfer.qPlaceClosed, "lower");
arm.moveJoints(bot, transfer.qPlaceClosed, transfer.qPlaceOpen, "move");
pause(arm.config.gripper.pause);
arm.moveJoints(bot, transfer.qPlaceOpen, transfer.qRetreatOpen, "lift");
qCurrent = transfer.qRetreatOpen;
end

function pose = cachedAsSameAnglePose(basePose, cfg)
pose = emptySameAnglePose();
pose.square = basePose.square;
pose.fileIndex = basePose.fileIndex;
pose.rankIndex = basePose.rankIndex;
pose.qAbove = basePose.qAbove;
pose.qGrip = basePose.qGrip;
pose.qAboveOpen = withGripperAngle(basePose.qAbove, cfg.gripper.openAngle, cfg);
pose.qGripOpen = withGripperAngle(basePose.qGrip, cfg.gripper.openAngle, cfg);
pose.qGripClosed = withGripperAngle(basePose.qGrip, cfg.gripper.closeAngle, cfg);
pose.qAboveClosed = withGripperAngle(basePose.qAbove, cfg.gripper.closeAngle, cfg);
pose.canMoveAbove = true;
pose.canPlace = true;
end

function pose = emptySameAnglePose()
pose = struct( ...
    "square", "", ...
    "fileIndex", [], ...
    "rankIndex", [], ...
    "frame5Angle", [], ...
    "qAbove", [], ...
    "qGrip", [], ...
    "qAboveOpen", [], ...
    "qGripOpen", [], ...
    "qGripClosed", [], ...
    "qAboveClosed", [], ...
    "aboveErr", inf, ...
    "gripErr", inf, ...
    "aboveAngleErr", inf, ...
    "gripAngleErr", inf, ...
    "canMoveAbove", false, ...
    "canPlace", false);
end

function count = countPlaceableSquares(placeMap)
count = 0;
for rankIndex = 1:size(placeMap, 1)
    for fileIndex = 1:size(placeMap, 2)
        count = count + double(placeMap(rankIndex, fileIndex).canPlace);
    end
end
end

function err = posePositionError(arm, q, targetXYZ)
T = arm.fk(q);
actualXYZ = T(1:3, 4)';
if isfield(arm, "motionTargetXYZ")
    targetXYZ = arm.motionTargetXYZ(targetXYZ);
end
err = norm(actualXYZ - targetXYZ);
end

function pieceType = initialSetupPieceType(squareName)
% Initial chess setup, mirrored for white and black.
squareName = upper(char(squareName));
fileChar = squareName(1);
rankIndex = str2double(squareName(2:end));

switch rankIndex
    case {1, 8}
        switch fileChar
            case {'A', 'H'}
                pieceType = "rook";
            case {'B', 'G'}
                pieceType = "knight";
            case {'C', 'F'}
                pieceType = "bishop";
            case 'D'
                pieceType = "queen";
            case 'E'
                pieceType = "king";
            otherwise
                pieceType = "empty";
        end
    case {2, 7}
        pieceType = "pawn";
    otherwise
        pieceType = "empty";
end
end

function tf = isSupportedTask2Piece(pieceType)
tf = any(lower(string(pieceType)) == ["rook", "bishop", "knight"]);
end

function q = withGripperAngle(q, gripperAngle, cfg)
if cfg.gripper.motorIndex <= numel(q)
    q(cfg.gripper.motorIndex) = gripperAngle;
end
end
