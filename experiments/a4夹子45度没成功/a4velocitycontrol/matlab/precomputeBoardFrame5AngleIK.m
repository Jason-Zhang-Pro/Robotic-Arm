function precomputeBoardFrame5AngleIK()
%% precomputeBoardFrame5AngleIK.m
% Search for a shared frame5 angle that works for pickup/place on the
% whole chessboard, allowing q5 to remain at the seed value or flip by 180 deg.
%
% The saved cache is intended for later pick/place controllers that need the
% picked piece and placed piece to use the same frame5 angle.

scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptDir);
cd(projectRoot);

%% User variables
cacheFile = fullfile(scriptDir, "board_frame5_angle_ik_cache.mat");
files = 'A':'H';
ranks = 1:8;
angleSearchStep = deg2rad(5);
q5FlipAngle = pi;
requireAbovePose = true;
useParallel = true;

%% Setup
setup_a4
cleanupSerialConnections()
load("UoM-RS-Var_Def.mat") %#ok<LOAD>
clear chessMoveModule
chessMoveModule

cfg = arm.config;
candidateAngles = cfg.frame5.searchMin:angleSearchStep:cfg.frame5.searchMax;

fprintf("Searching shared frame5 angles for the whole board...\n");
fprintf("Candidate angles: %.1f deg to %.1f deg, step %.1f deg\n", ...
    rad2deg(candidateAngles(1)), rad2deg(candidateAngles(end)), ...
    rad2deg(angleSearchStep));
fprintf("q5 seed options: seed q5 and seed q5 + 180 deg.\n");

angleResults = repmat(emptyAngleResult(), 1, numel(candidateAngles));

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

    fprintf("Using parallel precompute across %d frame5 angle candidates.\n", ...
        numel(candidateAngles));
    parfor angleIndex = 1:numel(candidateAngles)
        angleResults(angleIndex) = computeAngleResult( ...
            projectRoot, candidateAngles(angleIndex), files, ranks, ...
            q5FlipAngle, requireAbovePose);
    end
else
    fprintf("Parallel toolbox not available; using serial precompute.\n");
    for angleIndex = 1:numel(candidateAngles)
        angleResults(angleIndex) = computeAngleResult( ...
            projectRoot, candidateAngles(angleIndex), files, ranks, ...
            q5FlipAngle, requireAbovePose);
    end
end

bestIndex = chooseBestAngle(angleResults);
bestResult = angleResults(bestIndex);

taskBoardFrame5Plan = struct();
taskBoardFrame5Plan.generatedAt = string(datetime("now"));
taskBoardFrame5Plan.cacheFile = string(cacheFile);
taskBoardFrame5Plan.files = string(files);
taskBoardFrame5Plan.ranks = ranks;
taskBoardFrame5Plan.requireAbovePose = requireAbovePose;
taskBoardFrame5Plan.q5FlipAngle = q5FlipAngle;
taskBoardFrame5Plan.angleSearchStep = angleSearchStep;
taskBoardFrame5Plan.bestFrame5Angle = bestResult.frame5Angle;
taskBoardFrame5Plan.bestResult = bestResult;
taskBoardFrame5Plan.angleResults = angleResults;
taskBoardFrame5Plan.config = cfg;

save(cacheFile, "taskBoardFrame5Plan");

fprintf("\nBest shared frame5 angle: %.1f deg\n", rad2deg(bestResult.frame5Angle));
fprintf("Feasible squares: %d/64\n", bestResult.feasibleCount);
fprintf("Worst grip error: %.4f m\n", bestResult.worstGripErr);
fprintf("Worst above error: %.4f m\n", bestResult.worstAboveErr);
fprintf("Worst frame5 angle error: %.2f deg\n", rad2deg(bestResult.worstAngleErr));
fprintf("Saved board frame5 IK cache:\n%s\n", cacheFile);

printFailedSquares(bestResult.squarePoses);
end

function result = computeAngleResult(projectRoot, frame5Angle, files, ranks, ...
    q5FlipAngle, requireAbovePose)
addpath(projectRoot);
addpath(fullfile(projectRoot, "external", "UoM-RS-Starter_Code"));
addpath(fullfile(projectRoot, "external", "UoM_RS_MATLAB_LIBRARIES"));
addpath(genpath(fullfile(projectRoot, "matlab")));
clear chessMoveModule
chessMoveModule
cfg = arm.config;

warning("off", "all");

squarePoses = repmat(emptySquarePose(), 8, 8);
feasibleCount = 0;
worstGripErr = 0;
worstAboveErr = 0;
worstAngleErr = 0;
totalCost = 0;

for rankIndex = 1:numel(ranks)
    for fileIndex = 1:numel(files)
        squareName = string(files(fileIndex)) + string(ranks(rankIndex));
        xy = arm.squareToXY(squareName);
        aboveXYZ = [xy, cfg.motion.moveZ];
        gripXYZ = [xy, cfg.motion.gripZ];

        qSeed = seedForSquare(cfg, rankIndex, fileIndex);
        q5Seeds = [qSeed(5), qSeed(5) + q5FlipAngle];

        [qGrip, gripErr, gripAngleErr, gripQ5Mode] = ...
            bestFrame5Pose(arm, gripXYZ, frame5Angle, qSeed, q5Seeds);
        [qAbove, aboveErr, aboveAngleErr, aboveQ5Mode] = ...
            bestFrame5Pose(arm, aboveXYZ, frame5Angle, qGrip, ...
            [qGrip(5), qGrip(5) + q5FlipAngle]);

        pose = emptySquarePose();
        pose.square = squareName;
        pose.fileIndex = fileIndex;
        pose.rankIndex = rankIndex;
        pose.xy = xy;
        pose.aboveXYZ = aboveXYZ;
        pose.gripXYZ = gripXYZ;
        pose.frame5Angle = frame5Angle;
        pose.qAbove = qAbove;
        pose.qGrip = qGrip;
        pose.qAboveOpen = withGripperAngle(qAbove, cfg.gripper.openAngle, cfg);
        pose.qGripOpen = withGripperAngle(qGrip, cfg.gripper.openAngle, cfg);
        pose.qGripClosed = withGripperAngle(qGrip, cfg.gripper.closeAngle, cfg);
        pose.qAboveClosed = withGripperAngle(qAbove, cfg.gripper.closeAngle, cfg);
        pose.gripErr = gripErr;
        pose.aboveErr = aboveErr;
        pose.gripAngleErr = gripAngleErr;
        pose.aboveAngleErr = aboveAngleErr;
        pose.gripQ5Mode = gripQ5Mode;
        pose.aboveQ5Mode = aboveQ5Mode;

        pose.isFeasible = gripErr <= cfg.pickPlaceTolerance && ...
            gripAngleErr <= cfg.frame5.angleTolerance;
        if requireAbovePose
            pose.isFeasible = pose.isFeasible && ...
                aboveErr <= cfg.pickPlaceTolerance && ...
                aboveAngleErr <= cfg.frame5.angleTolerance;
        end

        squarePoses(rankIndex, fileIndex) = pose;

        feasibleCount = feasibleCount + double(pose.isFeasible);
        worstGripErr = max(worstGripErr, gripErr);
        worstAboveErr = max(worstAboveErr, aboveErr);
        worstAngleErr = max(worstAngleErr, max(gripAngleErr, aboveAngleErr));
        totalCost = totalCost + gripErr + aboveErr + gripAngleErr + aboveAngleErr;
    end
end

result = emptyAngleResult();
result.frame5Angle = frame5Angle;
result.feasibleCount = feasibleCount;
result.allFeasible = feasibleCount == 64;
result.worstGripErr = worstGripErr;
result.worstAboveErr = worstAboveErr;
result.worstAngleErr = worstAngleErr;
result.totalCost = totalCost;
result.squarePoses = squarePoses;

fprintf("Angle %.1f deg: feasible %d/64, worst grip %.4f m, worst above %.4f m, worst angle %.2f deg\n", ...
    rad2deg(frame5Angle), feasibleCount, worstGripErr, ...
    worstAboveErr, rad2deg(worstAngleErr));
end

function [qBest, errBest, angleErrBest, q5ModeBest] = ...
    bestFrame5Pose(arm, targetXYZ, frame5Angle, qSeed, q5Seeds)
qBest = [];
errBest = inf;
angleErrBest = inf;
q5ModeBest = "";
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
        if seedIndex == 1
            q5ModeBest = "seed";
        else
            q5ModeBest = "seed+180";
        end
    end
end
end

function qSeed = seedForSquare(cfg, rankIndex, fileIndex)
qSeed = cfg.qHome;
qSeed(1) = atan2(rankIndex - 4.5, fileIndex - 4.5);
end

function bestIndex = chooseBestAngle(angleResults)
feasibleCounts = [angleResults.feasibleCount];
worstGripErrs = [angleResults.worstGripErr];
worstAboveErrs = [angleResults.worstAboveErr];
worstAngleErrs = [angleResults.worstAngleErr];
totalCosts = [angleResults.totalCost];

score = -1000 * feasibleCounts + ...
    100 * worstGripErrs + 100 * worstAboveErrs + ...
    worstAngleErrs + 0.01 * totalCosts;
[~, bestIndex] = min(score);
end

function pose = emptySquarePose()
pose = struct( ...
    "square", "", ...
    "fileIndex", [], ...
    "rankIndex", [], ...
    "xy", [], ...
    "aboveXYZ", [], ...
    "gripXYZ", [], ...
    "frame5Angle", [], ...
    "qAbove", [], ...
    "qGrip", [], ...
    "qAboveOpen", [], ...
    "qGripOpen", [], ...
    "qGripClosed", [], ...
    "qAboveClosed", [], ...
    "gripErr", [], ...
    "aboveErr", [], ...
    "gripAngleErr", [], ...
    "aboveAngleErr", [], ...
    "gripQ5Mode", "", ...
    "aboveQ5Mode", "", ...
    "isFeasible", false);
end

function result = emptyAngleResult()
result = struct( ...
    "frame5Angle", [], ...
    "feasibleCount", 0, ...
    "allFeasible", false, ...
    "worstGripErr", inf, ...
    "worstAboveErr", inf, ...
    "worstAngleErr", inf, ...
    "totalCost", inf, ...
    "squarePoses", []);
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

function printFailedSquares(squarePoses)
failed = strings(1, 0);
for rankIndex = 1:size(squarePoses, 1)
    for fileIndex = 1:size(squarePoses, 2)
        pose = squarePoses(rankIndex, fileIndex);
        if ~pose.isFeasible
            failed(end + 1) = pose.square; %#ok<AGROW>
        end
    end
end

if isempty(failed)
    fprintf("All board squares are feasible with the best shared frame5 angle.\n");
else
    fprintf("Failed squares for the best shared frame5 angle:\n");
    disp(failed);
end
end
