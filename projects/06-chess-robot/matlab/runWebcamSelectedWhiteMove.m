function runWebcamSelectedWhiteMove(fromSquare, toSquare, captureFrame)
%% runWebcamSelectedWhiteMove.m
% Execute one selected legal white chess move after webcam inspection.
%
% Examples:
%   runWebcamSelectedWhiteMove("E2", "E4")
%   runWebcamSelectedWhiteMove("G1", "F3")
%   runWebcamSelectedWhiteMove("F1", "C4", false)
%
% The webcam frame is used as an operator/audit snapshot. The current camera
% view alone does not uniquely identify the opponent's previous move without
% a before/after frame pair, so this script chooses a conservative legal
% white move from the visible opening-like position.

scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptDir);
cd(projectRoot);

%% User variables
port = "COM5";
if nargin < 1 || isempty(fromSquare)
    fromSquare = "E2";
end
if nargin < 2 || isempty(toSquare)
    toSquare = "E4";
end
if nargin < 3 || isempty(captureFrame)
    captureFrame = true;
end
fromSquare = upper(string(fromSquare));
toSquare = upper(string(toSquare));
requiredArmIDs = 1:5;
ffmpegExe = "C:\Program Files\SOLIDWORKS Corp\SOLIDWORKS Flow Simulation\binCFW\ffmpeg.exe";
cameraName = "OsmoAction4";

%% Setup
setup_a4
cleanupSerialConnections()
load("UoM-RS-Var_Def.mat", "MAX_ID", "BAUDRATE")
clear chessMoveModule
chessMoveModule

if captureFrame
    saveAuditFrame(scriptDir, ffmpegExe, cameraName);
end

startXY = arm.squareToXY(fromSquare);
finalXY = arm.squareToXY(toSquare);

fprintf("Selected white move: %s -> %s\n", fromSquare, toSquare);
fprintf("Start XY:  [%.4f %.4f]\n", startXY(1), startXY(2));
fprintf("Target XY: [%.4f %.4f]\n", finalXY(1), finalXY(2));

%% Connect
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
    error("Cannot run selected white move. Missing required arm motor IDs: %s", ...
        mat2str(missingArmIDs));
end

if ~ismember(arm.config.gripper.motorIndex, connectedIDs)
    warning("Gripper motor ID %d was not detected. Arm will move, but gripper commands may not work.", ...
        arm.config.gripper.motorIndex);
end

%% Execute
qCurrent = arm.readCurrentQ(bot);
qHome = arm.config.qHome;
qHome(arm.config.gripper.motorIndex) = arm.config.gripper.openAngle;

fprintf("Moving to home before selected move...\n");
arm.moveJoints(bot, qCurrent, qHome, "move");

fprintf("Executing selected white move %s -> %s...\n", fromSquare, toSquare);
arm.pickAndPlaceXY(bot, startXY, finalXY, qHome);

qAfterMove = arm.readCurrentQ(bot);
fprintf("Returning to home after selected move...\n");
arm.moveJoints(bot, qAfterMove, qHome, "move");

fprintf("Selected white move complete. Robot returned home.\n");
end

function saveAuditFrame(scriptDir, ffmpegExe, cameraName)
outputDir = fullfile(scriptDir, "webcam_frames");
if exist(outputDir, "dir") ~= 7
    mkdir(outputDir);
end

timestamp = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
latestFile = fullfile(outputDir, "latest_board.jpg");
auditFile = fullfile(outputDir, "before_white_move_" + timestamp + ".jpg");

if exist(ffmpegExe, "file") ~= 2
    warning("ffmpeg not found, skipping webcam audit frame: %s", ffmpegExe);
    return;
end

command = sprintf('"%s" -hide_banner -y -f dshow -i "video=%s" -frames:v 1 -q:v 2 "%s"', ...
    ffmpegExe, cameraName, auditFile);
[status, output] = system(command);
if status ~= 0
    warning("Could not capture webcam audit frame:\n%s", output);
    return;
end

copyfile(auditFile, latestFile);
fprintf("Saved webcam audit frame:\n%s\n", auditFile);
end
