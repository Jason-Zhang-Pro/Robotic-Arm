function runTask2JoystickControl()
%% runTask2JoystickControl.m
% Cached joystick control for Task 2 robot-assisted chess moves.
%
% Flow:
%   1. Move robot to qHome.
%   2. First joystick press moves to B1 above the board with gripper open.
%   3. Push joystick freely to move above the piece to pick.
%   4. Next joystick press descends, closes gripper, and lifts.
%   5. Push joystick to move the held piece subject to pieceType.
%   6. Next joystick press places the piece and lifts.

scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptDir);
cd(projectRoot);

%% User variables
port = "COM5";
cacheFile = fullfile(scriptDir, "task2_joystick_ik_cache.mat");
transferCacheFile = fullfile(scriptDir, "task2_transfer_ik_cache.mat");
startSquare = "B1";
manualPieceType = "rook"; % Fallback if auto-detected piece is not supported.
samplePause = 0.08;
moveThreshold = 0.80;
deadband = 0.12;
adcMaxRaw = 1023; % 5 V joystick ADC range: 0 to 1023.
buttonThreshold = 10;
buttonConfirmSamples = 2;
centreRawXY = [518, 499];
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
task2JoyPlan = ensureTask2JoyPlanFrame5Angles(arm, task2JoyPlan);
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
fprintf("Joystick sample [raw_x raw_y raw_sw]:\n");
disp(centreRaw);
fprintf("Using fixed joystick centre [raw_x raw_y] = [%d %d], button threshold raw_sw < %d.\n", ...
    centreRawXY(1), centreRawXY(2), buttonThreshold);

[fileIndex, rankIndex] = squareToIndex(startSquare);
currentPose = task2JoyPlan.squarePoses(rankIndex, fileIndex);
isAtBoard = false;
isHolding = false;
activePieceType = "";
pickupPose = [];
buttonDownCount = 0;
buttonLatched = false;
lastMoveTime = tic;
sampleIndex = 0;
lastBishopFileStep = 1;
lastBishopRankStep = 1;

fprintf("Ready. Press joystick to move to %s above the board and open gripper.\n", startSquare);
fprintf("Task 2 piece constraint will be detected from the initial chess setup.\n");

while true
    sampleIndex = sampleIndex + 1;
    raw = readJoystickRaw(bot);
    joy = normaliseJoystick(raw, centreRawXY, deadband, adcMaxRaw, ...
        buttonThreshold, swapJoystickXY, invertJoystickX, invertJoystickY);
    buttonRawPressed = joy(3) == 1;
    if buttonRawPressed
        buttonDownCount = buttonDownCount + 1;
    else
        buttonDownCount = 0;
        buttonLatched = false;
    end
    buttonPressed = buttonDownCount >= buttonConfirmSamples;
    buttonEdge = buttonPressed && ~buttonLatched;
    if buttonEdge
        buttonLatched = true;
    end

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
            arm.moveJoints(bot, qCurrent, currentPose.qAboveOpen, "move");
            qCurrent = currentPose.qAboveOpen;
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
            if rankIndex == pickupPose.rankIndex && fileIndex == pickupPose.fileIndex
                fprintf("Placing back on pickup square %s directly.\n", pickupPose.square);
                arm.moveJoints(bot, qCurrent, pickupPose.qAboveClosed, "move");
                arm.moveJoints(bot, pickupPose.qAboveClosed, pickupPose.qGripClosed, "lower");
                arm.moveJoints(bot, pickupPose.qGripClosed, pickupPose.qGripOpen, "move");
                pause(arm.config.gripper.pause);
                arm.moveJoints(bot, pickupPose.qGripOpen, pickupPose.qAboveOpen, "lift");
                qCurrent = pickupPose.qAboveOpen;
                currentPose = pickupPose;
            else
                targetTransfer = boardTransfer(task2TransferPlan, ...
                    pickupPose.rankIndex, pickupPose.fileIndex, rankIndex, fileIndex);
                if canUseDirectTransfer(targetTransfer, pickupPose, samePoseAngleTolerance)
                    arm.moveJoints(bot, qCurrent, targetTransfer.qAbovePlaceClosed, "move");
                    qCurrent = targetTransfer.qAbovePlaceClosed;
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
                        qTarget = currentPose.qAboveClosed;
                        fprintf("Joystick %s selection: %s has no cached same-angle above pose; moving to square-above closed pose.\n", ...
                            moveLabel, currentPose.square);
                        arm.moveJoints(bot, qCurrent, qTarget, "move");
                        qCurrent = qTarget;
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

function joy = normaliseJoystick(raw, centreRawXY, deadband, adcMaxRaw, ...
    buttonThreshold, swapJoystickXY, invertJoystickX, invertJoystickY)
joy = zeros(1, 3);

for axisIndex = 1:2
    rawIndex = axisIndex;
    centre = centreRawXY(axisIndex);
    delta = double(raw(rawIndex) - centre);
    if delta >= 0
        scale = max(1, double(adcMaxRaw) - double(centre));
    else
        scale = max(1, double(centre));
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

joy(3) = raw(3) < buttonThreshold;
end

function [dFile, dRank] = freeJoystickStep(joy, threshold)
% Free eight-direction grid step before the piece is picked up.
[dFile, dRank] = joystickEightDirectionStep(joy, threshold);
end

function [dFile, dRank, lastBishopFileStep, lastBishopRankStep] = ...
    constrainedJoystickStep(joy, threshold, pieceType, ...
    lastBishopFileStep, lastBishopRankStep)
pieceType = lower(string(pieceType));
[dirFile, dirRank, sector] = joystickEightDirectionStep(joy, threshold);
dFile = 0;
dRank = 0;

switch pieceType
    case "rook"
        if sector == ""
            return;
        end
        if dirRank == 0
            dFile = dirFile;
        elseif dirFile == 0
            dRank = dirRank;
        end

    case "bishop"
        if sector == ""
            return;
        end
        if dirFile ~= 0 && dirRank ~= 0
            lastBishopFileStep = dirFile;
            lastBishopRankStep = dirRank;
            dFile = lastBishopFileStep;
            dRank = lastBishopRankStep;
        end

    case "knight"
        [dFile, dRank] = joystickKnightStep(joy, threshold);

    otherwise
        error("Unknown pieceType %s. Use rook, bishop, or knight.", pieceType);
end
end

function [dFile, dRank, sector] = joystickKnightStep(joy, threshold)
fileValue = joy(1);
rankValue = joy(2);
if hypot(fileValue, rankValue) < threshold
    dFile = 0;
    dRank = 0;
    sector = "";
    return;
end

angle = mod(atan2(rankValue, fileValue), 2 * pi);
sectorIndex = floor(angle / (pi / 4));

switch sectorIndex
    case 0
        dFile = 2; dRank = 1; sector = "EN";
    case 1
        dFile = 1; dRank = 2; sector = "NE";
    case 2
        dFile = -1; dRank = 2; sector = "NW";
    case 3
        dFile = -2; dRank = 1; sector = "WN";
    case 4
        dFile = -2; dRank = -1; sector = "WS";
    case 5
        dFile = -1; dRank = -2; sector = "SW";
    case 6
        dFile = 1; dRank = -2; sector = "SE";
    case 7
        dFile = 2; dRank = -1; sector = "ES";
    otherwise
        dFile = 0; dRank = 0; sector = "";
end
end

function [dFile, dRank, sector] = joystickEightDirectionStep(joy, threshold)
fileValue = joy(1);
rankValue = joy(2);
if hypot(fileValue, rankValue) < threshold
    dFile = 0;
    dRank = 0;
    sector = "";
    return;
end

angle = atan2(rankValue, fileValue);
sectorIndex = mod(round(angle / (pi / 4)), 8);

switch sectorIndex
    case 0
        dFile = 1; dRank = 0; sector = "E";
    case 1
        dFile = 1; dRank = 1; sector = "NE";
    case 2
        dFile = 0; dRank = 1; sector = "N";
    case 3
        dFile = -1; dRank = 1; sector = "NW";
    case 4
        dFile = -1; dRank = 0; sector = "W";
    case 5
        dFile = -1; dRank = -1; sector = "SW";
    case 6
        dFile = 0; dRank = -1; sector = "S";
    case 7
        dFile = 1; dRank = -1; sector = "SE";
    otherwise
        dFile = 0; dRank = 0; sector = "";
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

function task2JoyPlan = ensureTask2JoyPlanFrame5Angles(arm, task2JoyPlan)
if ~isfield(task2JoyPlan, "squarePoses")
    return;
end

samplePose = task2JoyPlan.squarePoses(1, 1);
if isfield(samplePose, "frame5AngleGrip")
    return;
end

warning("Task 2 joystick cache is missing frame5AngleGrip. Adding it in memory from cached qGrip poses.");
for rankIndex = 1:size(task2JoyPlan.squarePoses, 1)
    for fileIndex = 1:size(task2JoyPlan.squarePoses, 2)
        task2JoyPlan.squarePoses(rankIndex, fileIndex).frame5AngleGrip = ...
            arm.frame5GroundAngle(task2JoyPlan.squarePoses(rankIndex, fileIndex).qGrip);
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
