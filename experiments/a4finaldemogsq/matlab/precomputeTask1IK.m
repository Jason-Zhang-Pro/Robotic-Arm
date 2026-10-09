function precomputeTask1IK()
%% precomputeTask1IK.m
% Precompute all Task 1 IK poses and save them for fast demonstration.

scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptDir);
cd(projectRoot);

%% User variables
cacheFile = fullfile(scriptDir, "task1_ik_cache.mat");
useParallel = true;

%% Setup
setup_a4
cleanupSerialConnections()
load("UoM-RS-Var_Def.mat") %#ok<LOAD>
clear chessMoveModule
chessMoveModule

moves = arm.task1Moves();
moveInputs = buildTask1MoveInputs(arm, moves);
qHome = arm.config.qHome;
groupRanges = buildTask1MoveGroups(moveInputs);

fprintf("Precomputing Task 1 IK for %d robot moves...\n", numel(moveInputs));
fprintf("Only moves that wait for the opponent will return to qHome.\n");
fprintf("Move groups between home returns: %d\n", size(groupRanges, 1));

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

    fprintf("Using parallel precompute across independent move groups.\n");
    moveInputGroups = cell(1, size(groupRanges, 1));
    for groupIndex = 1:size(groupRanges, 1)
        firstMove = groupRanges(groupIndex, 1);
        lastMove = groupRanges(groupIndex, 2);
        moveInputGroups{groupIndex} = moveInputs(firstMove:lastMove);
    end

    cachedMoveGroups = cell(1, numel(moveInputGroups));
    parfor groupIndex = 1:numel(moveInputGroups)
        cachedMoveGroups{groupIndex} = computeCachedMoveGroup( ...
            projectRoot, moveInputGroups{groupIndex}, qHome);
    end

    cachedMoves = repmat(emptyCachedMove(), 1, numel(moveInputs));
    for groupIndex = 1:numel(cachedMoveGroups)
        groupMoves = cachedMoveGroups{groupIndex};
        for groupMoveIndex = 1:numel(groupMoves)
            cachedMoves(groupMoves(groupMoveIndex).index) = groupMoves(groupMoveIndex);
        end
    end
else
    fprintf("Parallel toolbox not available; using serial precompute.\n");
    cachedMoves = repmat(emptyCachedMove(), 1, numel(moveInputs));
    qStart = qHome;
    for moveIndex = 1:numel(moveInputs)
        cachedMoves(moveIndex) = computeCachedMove(arm, moveInputs(moveIndex), qStart, qHome);
        qStart = cachedMoves(moveIndex).qHomeAfter;
    end
end

task1Plan = struct();
task1Plan.generatedAt = string(datetime("now"));
task1Plan.cacheFile = string(cacheFile);
task1Plan.projectRoot = string(projectRoot);
task1Plan.numMoves = numel(cachedMoves);
task1Plan.moves = cachedMoves;

save(cacheFile, "task1Plan");

fprintf("Saved Task 1 IK cache:\n%s\n", cacheFile);
fprintf("Cached moves: %d\n", task1Plan.numMoves);
end

function moveInputs = buildTask1MoveInputs(arm, moves)
moveInputs = repmat(emptyMoveInput(), 1, numel(moves));
capturedIndex = 1;

for moveIndex = 1:numel(moves)
    move = moves(moveIndex);
    startXY = arm.squareToXY(move.from);
    if strcmpi(move.to, "captured pool")
        finalXY = arm.config.capture.poolXYList(capturedIndex, :);
        capturedIndex = capturedIndex + 1;
    else
        finalXY = arm.squareToXY(move.to);
    end

    moveInputs(moveIndex).index = moveIndex;
    moveInputs(moveIndex).from = string(move.from);
    moveInputs(moveIndex).to = string(move.to);
    moveInputs(moveIndex).waitAfter = move.waitAfter;
    moveInputs(moveIndex).startXY = startXY;
    moveInputs(moveIndex).finalXY = finalXY;
end
end

function groupRanges = buildTask1MoveGroups(moveInputs)
groupRanges = zeros(0, 2);
groupStart = 1;

for moveIndex = 1:numel(moveInputs)
    if moveInputs(moveIndex).waitAfter || moveIndex == numel(moveInputs)
        groupRanges(end + 1, :) = [groupStart, moveIndex]; %#ok<AGROW>
        groupStart = moveIndex + 1;
    end
end
end

function cachedMoves = computeCachedMoveGroup(projectRoot, moveInputs, qHome)
addProjectPaths(projectRoot);
clear chessMoveModule
chessMoveModule

cachedMoves = repmat(emptyCachedMove(), 1, numel(moveInputs));
qStart = qHome;

for moveIndex = 1:numel(moveInputs)
    cachedMoves(moveIndex) = computeCachedMove(arm, moveInputs(moveIndex), qStart, qHome);
    qStart = cachedMoves(moveIndex).qHomeAfter;
end
end

function addProjectPaths(projectRoot)
addpath(projectRoot);
addpath(fullfile(projectRoot, "external", "UoM-RS-Starter_Code"));
addpath(fullfile(projectRoot, "external", "UoM_RS_MATLAB_LIBRARIES"));
addpath(genpath(fullfile(projectRoot, "matlab")));
end

function cachedMove = computeCachedMove(arm, moveInput, qStart, qHome)
fprintf("Precompute %02d: %s -> %s\n", ...
    moveInput.index, moveInput.from, moveInput.to);

movePlan = arm.planPickAndPlaceXY(moveInput.startXY, moveInput.finalXY, qStart);
qRetreat = arm.ikXYZ(movePlan.placeAboveXYZ, movePlan.qPlace);
qRetreat(5) = movePlan.qPlace(5);
if moveInput.waitAfter
    qAfterMove = qHome;
else
    qAfterMove = qRetreat;
end

cachedMove = emptyCachedMove();
cachedMove.index = moveInput.index;
cachedMove.from = moveInput.from;
cachedMove.to = moveInput.to;
cachedMove.waitAfter = moveInput.waitAfter;
cachedMove.startXY = moveInput.startXY;
cachedMove.finalXY = moveInput.finalXY;
cachedMove.qHomeBefore = qStart;
cachedMove.qAbovePick = movePlan.qAbovePick;
cachedMove.qPick = movePlan.qPick;
cachedMove.qLift = movePlan.qAbovePick;
cachedMove.qAbovePlace = movePlan.qAbovePlace;
cachedMove.qPlace = movePlan.qPlace;
cachedMove.qRetreat = qRetreat;
cachedMove.qHomeAfter = qAfterMove;
cachedMove.frame5AngleHold = movePlan.frame5AngleHold;
cachedMove.pickErr = posePositionError(arm, movePlan.qPick, movePlan.pickXYZ);
cachedMove.placeErr = posePositionError(arm, movePlan.qPlace, movePlan.placeXYZ);
end

function err = posePositionError(arm, q, targetXYZ)
T = arm.fk(q);
actualXYZ = T(1:3, 4)';
err = norm(actualXYZ - targetXYZ);
end

function moveInput = emptyMoveInput()
moveInput = struct( ...
    "index", [], ...
    "from", "", ...
    "to", "", ...
    "waitAfter", false, ...
    "startXY", [], ...
    "finalXY", []);
end

function cachedMove = emptyCachedMove()
cachedMove = struct( ...
    "index", [], ...
    "from", "", ...
    "to", "", ...
    "waitAfter", false, ...
    "startXY", [], ...
    "finalXY", [], ...
    "qHomeBefore", [], ...
    "qAbovePick", [], ...
    "qPick", [], ...
    "qLift", [], ...
    "qAbovePlace", [], ...
    "qPlace", [], ...
    "qRetreat", [], ...
    "qHomeAfter", [], ...
    "frame5AngleHold", [], ...
    "pickErr", [], ...
    "placeErr", []);
end
