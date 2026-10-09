function runTask1Cached()
%% runTask1Cached.m
% Run Task 1 from precomputed IK poses.

scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptDir);
cd(projectRoot);

%% User variables
port = "COM5";
cacheFile = fullfile(scriptDir, "task1_ik_cache.mat");
requiredArmIDs = 1:5;

%% Setup
setup_a4
cleanupSerialConnections()
load("UoM-RS-Var_Def.mat")
clear chessMoveModule
chessMoveModule

if exist(cacheFile, "file") ~= 2
    error("Task 1 IK cache not found. Run precomputeTask1IK first: %s", cacheFile);
end

data = load(cacheFile, "task1Plan");
task1Plan = data.task1Plan;
fprintf("Loaded Task 1 IK cache:\n%s\n", cacheFile);
fprintf("Cached moves: %d\n", task1Plan.numMoves);

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
    error("Cannot run cached Task 1. Missing required arm motor IDs: %s", ...
        mat2str(missingArmIDs));
end

if ~ismember(arm.config.gripper.motorIndex, connectedIDs)
    warning("Gripper motor ID %d was not detected. Arm will move, but gripper commands may not work.", ...
        arm.config.gripper.motorIndex);
end

%% Run
arm.executeCachedTask1(bot, task1Plan);
end
