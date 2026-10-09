function runTask2JoystickControl()
%% runTask2JoystickControl.m
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
port = "COM4";
cacheFile = fullfile(scriptDir, "task2_joystick_ik_cache.mat");
transferCacheFile = fullfile(scriptDir, "task2_transfer_ik_cache.mat");
startSquare = "D1";
manualPieceType = "rook"; % Fallback if auto-detected piece is not supported.
samplePause = 0.08;
moveThreshold = 0.50;
deadband = 0.12;
adcMaxRaw = 687;
requireConfirmation = true;
samePoseAngleTolerance = deg2rad(2);
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

if exist(cacheFile, "file") ~= 2
    error("Task 2 joystick IK cache not found. Run precomputeTask2JoystickIK first: %s", cacheFile);
end
if exist(transferCacheFile, "file") ~= 2
    error("Task 2 transfer IK cache not found. Run precomputeTask2TransferIK first: %s", transferCacheFile);
end

data = load(cacheFile, "task2JoyPlan");
task2JoyPlan = data.task2JoyPlan;
data = load(transferCacheFile, "task2TransferPlan");
task2TransferPlan = data.task2TransferPlan;
validateTask2Caches(task2JoyPlan, task2TransferPlan);

fprintf("Loaded Task 2 joystick cache:\n%s\n", cacheFile);
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
            directPlaceCount = countDirectTransferTargets( ...
                task2TransferPlan, pickupPose, samePoseAngleTolerance);
            fprintf("Cached direct same-angle place squares: %d/64.\n", directPlaceCount);
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
            targetTransfer = boardTransfer(task2TransferPlan, ...
                pickupPose.rankIndex, pickupPose.fileIndex, rankIndex, fileIndex);
            if canUseDirectTransfer(targetTransfer, pickupPose, samePoseAngleTolerance)
                arm.moveJoints(bot, qCurrent, targetTransfer.qPlaceClosed, "lower");
                arm.moveJoints(bot, targetTransfer.qPlaceClosed, targetTransfer.qPlaceOpen, "move");
                pause(arm.config.gripper.pause);
                arm.moveJoints(bot, targetTransfer.qPlaceOpen, targetTransfer.qRetreatOpen, "lift");
                qCurrent = targetTransfer.qRetreatOpen;
                fprintf("Direct same-angle place complete at %s.\n", currentPose.square);
            else
                fprintf("No cached same-angle place for %s. Returning to %s, regripping, then placing target.\n", ...
                    currentPose.square, pickupPose.square);
                [qCurrent, currentPose] = returnRegripAndPlaceTargetCached( ...
                    arm, bot, qCurrent, task2JoyPlan, task2TransferPlan, ...
                    pickupPose, rankIndex, fileIndex);
            end
            isHolding = false;
            activePieceType = "";
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
                    targetTransfer = boardTransfer(task2TransferPlan, ...
                        pickupPose.rankIndex, pickupPose.fileIndex, rankIndex, fileIndex);
                    if canUseDirectTransfer(targetTransfer, pickupPose, samePoseAngleTolerance)
                        qTarget = targetTransfer.qAbovePlaceClosed;
                        fprintf("Joystick %s move: moving to %s above board.\n", ...
                            moveLabel, currentPose.square);
                        arm.moveJoints(bot, qCurrent, qTarget, "move");
                        qCurrent = qTarget;
                    else
                        fprintf("Joystick %s selection: %s has no cached same-angle above pose; arm will stay put until placement.\n", ...
                            moveLabel, currentPose.square);
                    end
                else
                    qTarget = currentPose.qAboveOpen;
                    fprintf("Joystick %s move: moving to %s above board.\n", ...
                        moveLabel, currentPose.square);
                    arm.moveJoints(bot, qCurrent, qTarget, "move");
                    qCurrent = qTarget;
                end
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

function validateTask2Caches(task2JoyPlan, task2TransferPlan)
if ~isfield(task2JoyPlan, "squarePoses")
    error("Task 2 joystick cache is missing squarePoses. Run precomputeTask2JoystickIK.");
end
samplePose = task2JoyPlan.squarePoses(1, 1);
if ~isfield(samplePose, "frame5AngleGrip")
    error("Task 2 joystick cache is missing frame5AngleGrip. Run precomputeTask2JoystickIK again.");
end
if ~isfield(task2TransferPlan, "transferTable")
    error("Task 2 transfer cache is missing transferTable. Run precomputeTask2TransferIK.");
end
sampleTransfer = task2TransferPlan.transferTable(1, 1, 1, 1);
requiredFields = ["qAbovePick", "qPickOpen", "qPickClosed", ...
    "qLiftClosed", "qAbovePlaceClosed", "qPlaceClosed", ...
    "qPlaceOpen", "qRetreatOpen", "frame5AnglePick", ...
    "frame5AnglePlace", "isFeasible"];
for fieldIndex = 1:numel(requiredFields)
    if ~isfield(sampleTransfer, requiredFields(fieldIndex))
        error("Task 2 transfer cache is missing %s. Run precomputeTask2TransferIK again.", ...
            char(requiredFields(fieldIndex)));
    end
end
end

function transfer = boardTransfer(task2TransferPlan, fromRankIndex, fromFileIndex, ...
    toRankIndex, toFileIndex)
transfer = task2TransferPlan.transferTable(fromRankIndex, fromFileIndex, ...
    toRankIndex, toFileIndex);
if isempty(transfer.index)
    error("No cached transfer for %s -> %s.", transfer.fromSquare, transfer.toSquare);
end
end

function tf = canUseDirectTransfer(transfer, pickupPose, angleTolerance)
tf = transfer.isFeasible && ...
    ~isempty(transfer.qAbovePlaceClosed) && ...
    ~isempty(transfer.qPlaceClosed) && ...
    abs(angleDifference(transfer.frame5AnglePick, pickupPose.frame5AngleGrip)) <= angleTolerance && ...
    abs(angleDifference(transfer.frame5AnglePlace, pickupPose.frame5AngleGrip)) <= angleTolerance;
end

function count = countDirectTransferTargets(task2TransferPlan, pickupPose, angleTolerance)
count = 0;
for rankIndex = 1:8
    for fileIndex = 1:8
        transfer = boardTransfer(task2TransferPlan, ...
            pickupPose.rankIndex, pickupPose.fileIndex, rankIndex, fileIndex);
        count = count + double(canUseDirectTransfer(transfer, pickupPose, angleTolerance));
    end
end
end

function diff = angleDifference(a, b)
diff = atan2(sin(a - b), cos(a - b));
end

function [qCurrent, currentPose] = returnRegripAndPlaceTargetCached( ...
    arm, bot, qCurrent, task2JoyPlan, task2TransferPlan, ...
    pickupPose, targetRankIndex, targetFileIndex)
targetTransfer = boardTransfer(task2TransferPlan, ...
    pickupPose.rankIndex, pickupPose.fileIndex, targetRankIndex, targetFileIndex);
if ~targetTransfer.isFeasible
    error("Cached transfer %s -> %s is not feasible: %s", ...
        targetTransfer.fromSquare, targetTransfer.toSquare, targetTransfer.errorMessage);
end

arm.moveJoints(bot, qCurrent, pickupPose.qAboveClosed, "move");
arm.moveJoints(bot, pickupPose.qAboveClosed, pickupPose.qGripClosed, "lower");
arm.moveJoints(bot, pickupPose.qGripClosed, pickupPose.qGripOpen, "move");
pause(arm.config.gripper.pause);
arm.moveJoints(bot, pickupPose.qGripOpen, pickupPose.qAboveOpen, "lift");
qCurrent = pickupPose.qAboveOpen;

fprintf("Executing cached transfer %s -> %s after regrip.\n", ...
    targetTransfer.fromSquare, targetTransfer.toSquare);
qCurrent = executeCachedTransfer(arm, bot, qCurrent, targetTransfer);
currentPose = task2JoyPlan.squarePoses(targetRankIndex, targetFileIndex);
end

function qCurrent = executeCachedTransfer(arm, bot, qCurrent, transfer)
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
