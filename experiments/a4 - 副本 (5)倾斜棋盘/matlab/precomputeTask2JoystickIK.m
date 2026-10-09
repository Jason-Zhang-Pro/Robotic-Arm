function precomputeTask2JoystickIK()
%% precomputeTask2JoystickIK.m
% Precompute IK poses used by Task 2 joystick grid control.

scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptDir);
cd(projectRoot);

%% User variables
cacheFile = fullfile(scriptDir, "task2_joystick_ik_cache.mat");
files = 'A':'H';
ranks = 1:8;

%% Setup
setup_a4
cleanupSerialConnections()
load("UoM-RS-Var_Def.mat") %#ok<LOAD>
clear chessMoveModule
chessMoveModule

fprintf("Precomputing Task 2 joystick IK for all board squares...\n");

squarePoses = repmat(emptySquarePose(), 8, 8);
qSeed = arm.config.qHome;

for rankIndex = 1:numel(ranks)
    for fileIndex = 1:numel(files)
        squareName = string(files(fileIndex)) + string(ranks(rankIndex));
        xy = arm.squareToXY(squareName);
        aboveXYZ = [xy, arm.config.motion.moveZ];
        gripXYZ = [xy, arm.config.motion.gripZ];

        fprintf("  %s above/grip IK...\n", squareName);

        frame5AxisPreferred = -arm.boardNormalAtXYZ(gripXYZ);
        qAboveSeed = arm.ikXYZ(aboveXYZ, qSeed);
        qGrip = arm.ikXYZPreferFrame5Axis(gripXYZ, frame5AxisPreferred, qAboveSeed);
        qAbove = arm.ikXYZPreferFrame5Axis(aboveXYZ, frame5AxisPreferred, qGrip);

        qAboveOpen = withGripperAngle(qAbove, arm.config.gripper.openAngle, arm.config);
        qGripOpen = withGripperAngle(qGrip, arm.config.gripper.openAngle, arm.config);
        qGripClosed = withGripperAngle(qGrip, arm.config.gripper.closeAngle, arm.config);
        qAboveClosed = withGripperAngle(qAbove, arm.config.gripper.closeAngle, arm.config);

        pose = emptySquarePose();
        pose.square = squareName;
        pose.fileIndex = fileIndex;
        pose.rankIndex = rankIndex;
        pose.xy = xy;
        pose.aboveXYZ = aboveXYZ;
        pose.gripXYZ = gripXYZ;
        pose.qAbove = qAbove;
        pose.qGrip = qGrip;
        pose.qAboveOpen = qAboveOpen;
        pose.qGripOpen = qGripOpen;
        pose.qGripClosed = qGripClosed;
        pose.qAboveClosed = qAboveClosed;
        pose.aboveErr = posePositionError(arm, qAbove, aboveXYZ);
        pose.gripErr = posePositionError(arm, qGrip, gripXYZ);

        squarePoses(rankIndex, fileIndex) = pose;
        qSeed = qAbove;
    end
end

task2JoyPlan = struct();
task2JoyPlan.generatedAt = string(datetime("now"));
task2JoyPlan.cacheFile = string(cacheFile);
task2JoyPlan.files = string(files);
task2JoyPlan.ranks = ranks;
task2JoyPlan.startSquare = "D1";
task2JoyPlan.config = arm.config;
task2JoyPlan.squarePoses = squarePoses;

save(cacheFile, "task2JoyPlan");

fprintf("Saved Task 2 joystick IK cache:\n%s\n", cacheFile);
end

function pose = emptySquarePose()
pose = struct( ...
    "square", "", ...
    "fileIndex", [], ...
    "rankIndex", [], ...
    "xy", [], ...
    "aboveXYZ", [], ...
    "gripXYZ", [], ...
    "qAbove", [], ...
    "qGrip", [], ...
    "qAboveOpen", [], ...
    "qGripOpen", [], ...
    "qGripClosed", [], ...
    "qAboveClosed", [], ...
    "aboveErr", [], ...
    "gripErr", []);
end

function q = withGripperAngle(q, gripperAngle, cfg)
if cfg.gripper.motorIndex <= numel(q)
    q(cfg.gripper.motorIndex) = gripperAngle;
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
