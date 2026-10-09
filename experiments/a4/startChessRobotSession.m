function startChessRobotSession()
%% startChessRobotSession.m
% Persistent MATLAB control session for the chess robot.
%
% Command file protocol:
%   matlab/robot_command.txt
%
% Commands:
%   MOVE,<id>,<fromSquare>,<toSquare>
%   CAPTURE,<id>,<fromSquare>,<targetSquare>
%   HOME,<id>
%   STOP,<id>
%
% Status file:
%   matlab/robot_status.txt

projectRoot = fileparts(mfilename("fullpath"));
cd(projectRoot);

commandFile = fullfile(projectRoot, "matlab", "robot_command.txt");
statusFile = fullfile(projectRoot, "matlab", "robot_status.txt");
logFile = fullfile(projectRoot, "matlab", "robot_session.log");

if exist(commandFile, "file") == 2
    delete(commandFile);
end

diary(logFile);
diary on;
cleanupDiary = onCleanup(@() diary("off")); %#ok<NASGU>

fprintf("\n=== Chess robot persistent session starting: %s ===\n", string(datetime("now")));
writeStatus(statusFile, "STARTING", "initialising MATLAB robot session");

%% Setup
setup_a4
cleanupSerialConnections()
load("UoM-RS-Var_Def.mat", "MAX_ID", "BAUDRATE")
clear chessMoveModule
chessMoveModule
task2TransferPlan = loadTask2TransferPlan(projectRoot);

port = "COM5";
requiredArmIDs = 1:5;

bot = robot(port, MAX_ID, BAUDRATE);
bot.ConnectToArduino();
arm.applySavedOffset(bot);

bot.getID();
fprintf("Number of connected motors: %d\n", bot.num_ID);
fprintf("Connected motor IDs:\n");
disp(bot.connected_ID);

connectedIDs = bot.connected_ID(1:bot.num_ID);
missingArmIDs = setdiff(requiredArmIDs, connectedIDs);
if ~isempty(missingArmIDs)
    error("Cannot start persistent session. Missing required arm motor IDs: %s", ...
        mat2str(missingArmIDs));
end

if ~ismember(arm.config.gripper.motorIndex, connectedIDs)
    warning("Gripper motor ID %d was not detected. Gripper commands may not work.", ...
        arm.config.gripper.motorIndex);
end

qHome = homeWithOpenGripper(arm);
moveHome(arm, bot, qHome);

lastCommandId = "";
nextCaptureIndex = 1;
writeStatus(statusFile, "IDLE", "ready");
fprintf("Persistent session ready. Waiting for commands in:\n%s\n", commandFile);

while true
    pause(0.10);

    if exist(commandFile, "file") ~= 2
        continue;
    end

    commandText = strtrim(string(fileread(commandFile)));
    if commandText == ""
        continue;
    end

    [commandName, commandId, args] = parseCommand(commandText);
    if commandId == "" || commandId == lastCommandId
        continue;
    end
    lastCommandId = commandId;

    fprintf("\nCommand %s received: %s\n", commandId, commandText);

    try
        switch upper(commandName)
            case "MOVE"
                if numel(args) < 2
                    error("MOVE command requires fromSquare and toSquare.");
                end
                fromSquare = upper(args(1));
                toSquare = upper(args(2));
                writeStatus(statusFile, "WORKING", "MOVE " + fromSquare + " " + toSquare);
                executeOneMove(arm, bot, qHome, fromSquare, toSquare, task2TransferPlan);
                writeStatus(statusFile, "DONE", "MOVE " + fromSquare + " " + toSquare);

            case "CAPTURE"
                if numel(args) < 2
                    error("CAPTURE command requires fromSquare and targetSquare.");
                end
                fromSquare = upper(args(1));
                targetSquare = upper(args(2));
                writeStatus(statusFile, "WORKING", ...
                    "CAPTURE " + fromSquare + " " + targetSquare);
                [nextCaptureIndex, poolLabel] = executeCaptureMove( ...
                    arm, bot, qHome, fromSquare, targetSquare, ...
                    nextCaptureIndex, task2TransferPlan);
                writeStatus(statusFile, "DONE", ...
                    "CAPTURE " + fromSquare + " " + targetSquare + " to " + poolLabel);

            case "HOME"
                writeStatus(statusFile, "WORKING", "HOME");
                moveHome(arm, bot, qHome);
                writeStatus(statusFile, "DONE", "HOME");

            case "STOP"
                writeStatus(statusFile, "STOPPED", "session stopped by command");
                fprintf("STOP command received. Exiting persistent session.\n");
                break;

            otherwise
                error("Unknown command: %s", commandName);
        end
    catch ME
        writeStatus(statusFile, "ERROR", ME.message);
        fprintf("Command %s failed:\n%s\n", commandId, getReport(ME, "extended", "hyperlinks", "off"));
        try
            moveHome(arm, bot, qHome);
        catch homeError
            fprintf("Home recovery failed:\n%s\n", getReport(homeError, "basic", "hyperlinks", "off"));
        end
    end
end

fprintf("=== Chess robot persistent session ended: %s ===\n", string(datetime("now")));
end

function task2TransferPlan = loadTask2TransferPlan(projectRoot)
cacheFile = fullfile(projectRoot, "matlab", "task2_transfer_ik_cache.mat");
task2TransferPlan = [];

if exist(cacheFile, "file") ~= 2
    warning("Task 2 transfer IK cache not found. Realtime IK fallback will be used: %s", ...
        cacheFile);
    return;
end

data = load(cacheFile, "task2TransferPlan");
if ~isfield(data, "task2TransferPlan")
    warning("Task 2 transfer cache missing task2TransferPlan. Realtime IK fallback will be used.");
    return;
end

task2TransferPlan = data.task2TransferPlan;
fprintf("Loaded Task 2 transfer IK cache:\n%s\n", cacheFile);
fprintf("Cached feasible transfers: %d/%d\n", ...
    task2TransferPlan.feasibleCount, task2TransferPlan.numTransfers);
end

function [nextCaptureIndex, poolLabel] = executeCaptureMove( ...
        arm, bot, qHome, fromSquare, targetSquare, nextCaptureIndex, task2TransferPlan)
poolXYList = arm.config.capture.poolXYList;
if nextCaptureIndex > size(poolXYList, 1)
    error("No captured pool positions left.");
end

capturedXY = poolXYList(nextCaptureIndex, :);
poolLabel = "capture-pool-" + string(nextCaptureIndex);

fprintf("Capture move: %s captures on %s, captured piece -> %s\n", ...
    fromSquare, targetSquare, poolLabel);

targetXY = arm.squareToXY(targetSquare);
moveHome(arm, bot, qHome);

fprintf("Removing captured piece from %s to %s...\n", targetSquare, poolLabel);
captureTransfer = cachedCaptureTransfer(task2TransferPlan, targetSquare, nextCaptureIndex);
if canUseCachedTransfer(captureTransfer)
    executeCachedTransfer(arm, bot, qHome, captureTransfer);
else
    fprintf("No feasible cached capture transfer for %s -> %s; using realtime IK.\n", ...
        targetSquare, poolLabel);
    arm.pickAndPlaceXY(bot, targetXY, capturedXY, qHome);
    moveHome(arm, bot, qHome);
end

fprintf("Moving attacker %s -> %s...\n", fromSquare, targetSquare);
executeOneMove(arm, bot, qHome, fromSquare, targetSquare, task2TransferPlan);

nextCaptureIndex = nextCaptureIndex + 1;
end

function executeOneMove(arm, bot, qHome, fromSquare, toSquare, task2TransferPlan)
startXY = arm.squareToXY(fromSquare);
finalXY = arm.squareToXY(toSquare);

fprintf("Selected white move: %s -> %s\n", fromSquare, toSquare);
fprintf("Start XY:  [%.4f %.4f]\n", startXY(1), startXY(2));
fprintf("Target XY: [%.4f %.4f]\n", finalXY(1), finalXY(2));

moveHome(arm, bot, qHome);
fprintf("Executing selected white move %s -> %s...\n", fromSquare, toSquare);
boardTransfer = cachedBoardTransfer(task2TransferPlan, fromSquare, toSquare);
if canUseCachedTransfer(boardTransfer)
    executeCachedTransfer(arm, bot, qHome, boardTransfer);
else
    fprintf("No feasible cached transfer for %s -> %s; using realtime IK.\n", ...
        fromSquare, toSquare);
    arm.pickAndPlaceXY(bot, startXY, finalXY, qHome);
    moveHome(arm, bot, qHome);
end
fprintf("Selected white move complete. Robot returned home.\n");
end

function transfer = cachedBoardTransfer(task2TransferPlan, fromSquare, toSquare)
transfer = [];
if isempty(task2TransferPlan) || ~isfield(task2TransferPlan, "transferTable")
    return;
end

[fromFile, fromRank] = squareToIndex(fromSquare);
[toFile, toRank] = squareToIndex(toSquare);
transfer = task2TransferPlan.transferTable(fromRank, fromFile, toRank, toFile);
end

function transfer = cachedCaptureTransfer(task2TransferPlan, fromSquare, captureIndex)
transfer = [];
if isempty(task2TransferPlan) || ~isfield(task2TransferPlan, "captureTransferTable")
    return;
end

[fromFile, fromRank] = squareToIndex(fromSquare);
if captureIndex > size(task2TransferPlan.captureTransferTable, 3)
    return;
end
transfer = task2TransferPlan.captureTransferTable(fromRank, fromFile, captureIndex);
end

function tf = canUseCachedTransfer(transfer)
tf = ~isempty(transfer) && ...
    isfield(transfer, "isFeasible") && transfer.isFeasible && ...
    isfield(transfer, "qAbovePick") && ~isempty(transfer.qAbovePick) && ...
    isfield(transfer, "qRetreatOpen") && ~isempty(transfer.qRetreatOpen);
end

function executeCachedTransfer(arm, bot, qHome, transfer)
fprintf("Executing cached Task 2 transfer %s -> %s.\n", ...
    transfer.fromSquare, transfer.toSquare);

qCurrent = arm.readCurrentQ(bot);
arm.moveToPickAboveForPick(bot, qCurrent, transfer.qAbovePick);
arm.moveJoints(bot, transfer.qAbovePick, transfer.qPickOpen, "lower");
arm.moveJoints(bot, transfer.qPickOpen, transfer.qPickClosed, "move");
pause(arm.config.gripper.pause);
arm.moveJoints(bot, transfer.qPickClosed, transfer.qLiftClosed, "lift");
arm.moveJoints(bot, transfer.qLiftClosed, transfer.qAbovePlaceClosed, "move");
arm.moveJoints(bot, transfer.qAbovePlaceClosed, transfer.qPlaceClosed, "lower");
arm.prepareGripperForRelease(bot);
qPlaceRelease = withGripperReleaseAngle(arm, transfer.qPlaceOpen);
arm.moveJoints(bot, qPlaceRelease, transfer.qRetreatOpen, "lift");
moveHome(arm, bot, qHome);
end

function q = withGripperReleaseAngle(arm, q)
releaseAngle = arm.config.gripper.openAngle + ...
    arm.config.gripper.prePickExtraOpenAngle;
q = qWithGripperAngle(q, releaseAngle, arm.config);
end

function q = qWithGripperAngle(q, gripperAngle, cfg)
if cfg.gripper.motorIndex <= numel(q)
    q(cfg.gripper.motorIndex) = gripperAngle;
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

function moveHome(arm, bot, qHome)
qCurrent = arm.readCurrentQ(bot);
fprintf("Moving to home...\n");
arm.moveJoints(bot, qCurrent, qHome, "move");
end

function qHome = homeWithOpenGripper(arm)
qHome = arm.config.qHome;
if arm.config.gripper.motorIndex <= numel(qHome)
    qHome(arm.config.gripper.motorIndex) = arm.config.gripper.openAngle;
end
end

function [commandName, commandId, args] = parseCommand(commandText)
parts = strip(split(commandText, ","));
commandName = "";
commandId = "";
args = strings(0, 1);

if numel(parts) >= 1
    commandName = upper(parts(1));
end
if numel(parts) >= 2
    commandId = parts(2);
end
if numel(parts) >= 3
    args = parts(3:end);
end
end

function writeStatus(statusFile, state, message)
timestamp = string(datetime("now", "Format", "yyyy-MM-dd HH:mm:ss"));
text = state + "," + timestamp + "," + string(message);
fid = fopen(statusFile, "w");
if fid < 0
    warning("Could not write status file: %s", statusFile);
    return;
end
cleanupObj = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, "%s\n", text);
end
