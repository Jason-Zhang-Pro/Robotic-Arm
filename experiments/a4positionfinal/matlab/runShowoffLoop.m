function runShowoffLoop()
%% runShowoffLoop.m
% Run the recorded showoff poses in an infinite loop.
% Stop manually with Ctrl+C in the MATLAB Command Window.

scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptDir);
cd(projectRoot);

%% User variables
port = "COM5";
recordingFile = fullfile(scriptDir, "showoff_pose_recording.mat");
applySavedOffset = true;
requireConfirmation = true;
maxJointStep = deg2rad(5);
pathStepPause = 0.01;
maxEndEffectorSpeed = 0.001; % [] uses arm.config.motion.maxEndEffectorSpeedMove [m/s]

%% Setup
setup_a4
cleanupSerialConnections()
load("UoM-RS-Var_Def.mat", "MAX_ID", "BAUDRATE")
clear chessMoveModule
chessMoveModule

if exist(recordingFile, "file") ~= 2
    error("Showoff recording not found. Run recordShowoffPoses first: %s", recordingFile);
end

data = load(recordingFile, "showoffPlan");
showoffPlan = data.showoffPlan;
recordedQ = showoffPlan.recordedQ;
recordedQ = normaliseRecordedPoses(recordedQ, arm.config);
if isempty(maxEndEffectorSpeed)
    maxEndEffectorSpeed = arm.config.motion.maxEndEffectorSpeedMove;
end

if isempty(recordedQ)
    error("Showoff recording contains no poses. Record at least one posture first.");
end

fprintf("Loaded showoff recording:\n%s\n", recordingFile);
fprintf("Recorded poses: %d\n", size(recordedQ, 1));
fprintf("Showoff EE speed limit: %.3f m/s\n", maxEndEffectorSpeed);

if requireConfirmation
    fprintf("This script will MOVE the robot and loop forever until Ctrl+C.\n");
    reply = input("Type GO to start showoff loop: ", "s");
    if ~strcmp(reply, "GO")
        fprintf("Cancelled. No motion command was sent.\n");
        return;
    end
end

%% Connect
bot = robot(port, MAX_ID, BAUDRATE);
bot.ConnectToArduino();

if applySavedOffset
    arm.applySavedOffset(bot);
end

bot.getID();
connectedIDs = bot.connected_ID(1:bot.num_ID);
fprintf("Connected motor IDs:\n");
disp(connectedIDs);

requiredIDs = 1:arm.config.arm.numKinematicJoints;
missingIDs = setdiff(requiredIDs, connectedIDs);
if ~isempty(missingIDs)
    error("Cannot run showoff loop. Missing required arm motor IDs: %s", ...
        mat2str(missingIDs));
end

if ~ismember(arm.config.gripper.motorIndex, connectedIDs)
    warning("Gripper motor ID %d was not detected. Recorded gripper poses may not run.", ...
        arm.config.gripper.motorIndex);
end

%% Run forever
qCurrent = arm.readCurrentQ(bot);
fprintf("Moving to first showoff pose...\n");
arm.moveJoints(bot, qCurrent, recordedQ(1, :), "move");

trajectoryQ = buildContinuousTrajectory(recordedQ, maxJointStep, ...
    maxEndEffectorSpeed, pathStepPause, arm);
fprintf("Continuous showoff trajectory points per loop: %d\n", size(trajectoryQ, 1));

if ismethod(bot, "setDriveControlMode")
    bot.setDriveControlMode(zeros(1, arm.config.motor.numJoints));
end

loopIndex = 0;
fprintf("Showoff loop running. Press Ctrl+C to stop.\n");
while true
    loopIndex = loopIndex + 1;
    fprintf("Showoff loop %d\n", loopIndex);
    playTrajectory(bot, arm.config, trajectoryQ, pathStepPause);
end
end

function recordedQ = normaliseRecordedPoses(recordedQ, cfg)
recordedQ = rowN(recordedQ, cfg.motor.numJoints);
recordedQ(:, 1:cfg.arm.numKinematicJoints) = ...
    min(max(recordedQ(:, 1:cfg.arm.numKinematicJoints), ...
    cfg.qMin(1:cfg.arm.numKinematicJoints)), ...
    cfg.qMax(1:cfg.arm.numKinematicJoints));
if cfg.gripper.motorIndex <= size(recordedQ, 2)
    recordedQ(:, cfg.gripper.motorIndex) = min(max( ...
        recordedQ(:, cfg.gripper.motorIndex), ...
        cfg.qMin(cfg.gripper.motorIndex)), ...
        cfg.qMax(cfg.gripper.motorIndex));
end
end

function trajectoryQ = buildContinuousTrajectory(recordedQ, maxJointStep, ...
    maxEndEffectorSpeed, pathStepPause, arm)
recordedQ = rowN(recordedQ, size(recordedQ, 2));
numPoses = size(recordedQ, 1);
trajectoryQ = zeros(0, size(recordedQ, 2));

for poseIndex = 1:numPoses
    qStart = recordedQ(poseIndex, :);
    if poseIndex < numPoses
        qTarget = recordedQ(poseIndex + 1, :);
    else
        qTarget = recordedQ(1, :);
    end

    qDelta = qTarget - qStart;
    jointStepCount = ceil(max(abs(qDelta)) / max(maxJointStep, eps));
    eePathLength = estimateEndEffectorPathLength(arm, qStart, qTarget, maxJointStep);
    maxEeStep = max(maxEndEffectorSpeed * pathStepPause, eps);
    eeStepCount = ceil(eePathLength / maxEeStep);
    stepCount = max([1, jointStepCount, eeStepCount]);

    for stepIndex = 1:stepCount
        s = stepIndex / stepCount;
        trajectoryQ(end + 1, :) = qStart + s * qDelta; %#ok<AGROW>
    end
end
end

function pathLength = estimateEndEffectorPathLength(arm, qStart, qTarget, maxJointStep)
qStart = rowN(qStart, arm.config.motor.numJoints);
qTarget = rowN(qTarget, arm.config.motor.numJoints);
qDelta = qTarget - qStart;
sampleCount = max(1, ceil(max(abs(qDelta(1:arm.config.arm.numKinematicJoints))) / ...
    max(maxJointStep, eps)));

pathLength = 0;
previousT = arm.fk(qStart);
previousXYZ = previousT(1:3, 4)';

for sampleIndex = 1:sampleCount
    s = sampleIndex / sampleCount;
    qSample = qStart + s * qDelta;
    currentT = arm.fk(qSample);
    currentXYZ = currentT(1:3, 4)';
    pathLength = pathLength + norm(currentXYZ - previousXYZ);
    previousXYZ = currentXYZ;
end
end

function playTrajectory(bot, cfg, trajectoryQ, stepPause)
for pointIndex = 1:size(trajectoryQ, 1)
    qTarget = trajectoryQ(pointIndex, :);
    bot.setQ(toMotorQ(qTarget, cfg));
    bot.driveMotors();
    pause(stepPause);
end
end

function qMotor = toMotorQ(qModel, cfg)
qMotor = rowN(qModel, cfg.motor.numJoints) .* cfg.jointDirection;
end

function y = rowN(x, n)
if isvector(x)
    y = reshape(x, 1, []);
    if numel(y) < n
        y = [y, zeros(1, n - numel(y))];
    elseif numel(y) > n
        y = y(1:n);
    end
else
    y = x;
    if size(y, 2) < n
        y = [y, zeros(size(y, 1), n - size(y, 2))];
    elseif size(y, 2) > n
        y = y(:, 1:n);
    end
end
end
