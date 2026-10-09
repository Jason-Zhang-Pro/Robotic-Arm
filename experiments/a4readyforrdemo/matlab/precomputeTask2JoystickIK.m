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
useParallel = true;

%% Setup
setup_a4
cleanupSerialConnections()
clear chessMoveModule
chessMoveModule

fprintf("Precomputing Task 2 joystick IK for all board squares...\n");

qSeed = arm.config.qHome;
squareInputs = buildSquareInputs(arm, files, ranks, qSeed);
poses = repmat(emptySquarePose(), 1, numel(squareInputs));

canUseParallel = useParallel && ...
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

    fprintf("Using parallel joystick precompute across board squares.\n");
    parfor inputIndex = 1:numel(squareInputs)
        poses(inputIndex) = computeSquarePose(projectRoot, squareInputs(inputIndex));
    end
else
    fprintf("Parallel toolbox not available; using serial joystick precompute.\n");
    for inputIndex = 1:numel(squareInputs)
        poses(inputIndex) = computeSquarePoseLocal(arm, squareInputs(inputIndex));
    end
end

squarePoses = repmat(emptySquarePose(), 8, 8);
for inputIndex = 1:numel(poses)
    pose = poses(inputIndex);
    squarePoses(pose.rankIndex, pose.fileIndex) = pose;
end

task2JoyPlan = struct();
task2JoyPlan.generatedAt = string(datetime("now"));
task2JoyPlan.cacheFile = string(cacheFile);
task2JoyPlan.files = string(files);
task2JoyPlan.ranks = ranks;
task2JoyPlan.startSquare = "B1";
task2JoyPlan.config = arm.config;
task2JoyPlan.squarePoses = squarePoses;

save(cacheFile, "task2JoyPlan");

fprintf("Saved Task 2 joystick IK cache:\n%s\n", cacheFile);
end

function squareInputs = buildSquareInputs(arm, files, ranks, qSeed)
squareInputs = repmat(emptySquareInput(), 1, numel(files) * numel(ranks));
inputIndex = 0;

for rankIndex = 1:numel(ranks)
    for fileIndex = 1:numel(files)
        squareName = string(files(fileIndex)) + string(ranks(rankIndex));
        xy = arm.squareToXY(squareName);

        inputIndex = inputIndex + 1;
        squareInputs(inputIndex).index = inputIndex;
        squareInputs(inputIndex).square = squareName;
        squareInputs(inputIndex).fileIndex = fileIndex;
        squareInputs(inputIndex).rankIndex = rankIndex;
        squareInputs(inputIndex).xy = xy;
        squareInputs(inputIndex).qSeed = qSeed;
    end
end
end

function input = emptySquareInput()
input = struct( ...
    "index", [], ...
    "square", "", ...
    "fileIndex", [], ...
    "rankIndex", [], ...
    "xy", [], ...
    "qSeed", []);
end

function pose = computeSquarePose(projectRoot, input)
addProjectPaths(projectRoot);
clear chessMoveModule
chessMoveModule
pose = computeSquarePoseLocal(arm, input);
end

function pose = computeSquarePoseLocal(arm, input)
xy = input.xy;
aboveXYZ = [xy, arm.config.motion.moveZ];
gripXYZ = [xy, arm.config.motion.gripZ];

fprintf("  %s above/grip IK...\n", input.square);

movePlan = arm.planPickAndPlaceXY(xy, xy, input.qSeed);
qAbove = movePlan.qAbovePick;
qGrip = movePlan.qPick;

qAboveOpen = withGripperAngle(qAbove, arm.config.gripper.openAngle, arm.config);
qGripOpen = withGripperAngle(qGrip, arm.config.gripper.openAngle, arm.config);
qGripClosed = withGripperAngle(qGrip, arm.config.gripper.closeAngle, arm.config);
qAboveClosed = withGripperAngle(qAbove, arm.config.gripper.closeAngle, arm.config);

pose = emptySquarePose();
pose.square = input.square;
pose.fileIndex = input.fileIndex;
pose.rankIndex = input.rankIndex;
pose.xy = xy;
pose.aboveXYZ = aboveXYZ;
pose.gripXYZ = gripXYZ;
pose.qAbove = qAbove;
pose.qGrip = qGrip;
pose.qAboveOpen = qAboveOpen;
pose.qGripOpen = qGripOpen;
pose.qGripClosed = qGripClosed;
pose.qAboveClosed = qAboveClosed;
pose.frame5AngleGrip = arm.frame5GroundAngle(qGrip);
pose.frame5AngleHold = movePlan.frame5AngleHold;
pose.aboveErr = posePositionError(arm, qAbove, aboveXYZ);
pose.gripErr = posePositionError(arm, qGrip, gripXYZ);
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
    "frame5AngleGrip", [], ...
    "frame5AngleHold", [], ...
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
err = norm(actualXYZ - targetXYZ);
end

function addProjectPaths(projectRoot)
projectRoot = char(projectRoot);
projectMatlabDir = fullfile(projectRoot, "matlab");
matlabLibDir = fullfile(projectRoot, "external", "UoM_RS_MATLAB_LIBRARIES");
starterDir = fullfile(projectRoot, "external", "UoM-RS-Starter_Code");

addpath(projectRoot);
if exist(projectMatlabDir, "dir") == 7
    addpath(genpath(projectMatlabDir));
end
if exist(matlabLibDir, "dir") == 7
    addpath(matlabLibDir);
end
if exist(starterDir, "dir") == 7
    addpath(starterDir);
end
cd(projectRoot);
end
