function precomputeTask2TransferIK()
%% precomputeTask2TransferIK.m
% Precompute all board-to-board and board-to-capture-pool Task 2 transfer IK plans.
%
% The cache contains:
%   1. board-square -> board-square/capture-pool targets
%   2. capture-pool -> board-square targets

scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptDir);
cd(projectRoot);

%% User variables
cacheFile = fullfile(scriptDir, "task2_transfer_ik_cache.mat");
files = 'A':'H';
ranks = 1:8;
q5Mode = "default";
useParallel = true;

%% Setup
setup_a4
cleanupSerialConnections()
load("UoM-RS-Var_Def.mat") %#ok<LOAD>
clear chessMoveModule
chessMoveModule

cfg = arm.config;
capturePoolXYList = cfg.capture.poolXYList;
transferInputs = buildTransferInputs(arm, files, ranks, capturePoolXYList);
numTransfers = numel(transferInputs);

fprintf("Precomputing Task 2 transfer IK cache...\n");
fprintf("Transfers: %d from/to pairs.\n", numTransfers);
fprintf("  board -> board/capture: %d pickup squares x %d targets\n", ...
    numel(files) * numel(ranks), ...
    numel(files) * numel(ranks) + size(capturePoolXYList, 1));
fprintf("  capture -> board: %d pool positions x %d board targets\n", ...
    size(capturePoolXYList, 1), numel(files) * numel(ranks));
fprintf("q5Mode: %s\n", q5Mode);

transfers = repmat(emptyTransfer(), 1, numTransfers);

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

    fprintf("Using parallel transfer precompute.\n");
    parfor transferIndex = 1:numTransfers
        transfers(transferIndex) = computeTransferPlan( ...
            projectRoot, transferInputs(transferIndex), q5Mode);
    end
else
    fprintf("Parallel toolbox not available; using serial transfer precompute.\n");
    for transferIndex = 1:numTransfers
        transfers(transferIndex) = computeTransferPlan( ...
            projectRoot, transferInputs(transferIndex), q5Mode);
    end
end

transferTable = repmat(emptyTransfer(), 8, 8, 8, 8);
capturePoolTransferTable = repmat(emptyTransfer(), 8, 8, size(capturePoolXYList, 1));
fromCapturePoolTransferTable = repmat(emptyTransfer(), size(capturePoolXYList, 1), 8, 8);
for transferIndex = 1:numTransfers
    move = transfers(transferIndex);
    if move.fromTargetType == "board" && move.toTargetType == "board"
        transferTable(move.fromRankIndex, move.fromFileIndex, ...
            move.toRankIndex, move.toFileIndex) = move;
    elseif move.fromTargetType == "board" && move.toTargetType == "capturePool"
        capturePoolTransferTable(move.fromRankIndex, move.fromFileIndex, ...
            move.toPoolIndex) = move;
    elseif move.fromTargetType == "capturePool" && move.toTargetType == "board"
        fromCapturePoolTransferTable(move.fromPoolIndex, ...
            move.toRankIndex, move.toFileIndex) = move;
    end
end

feasibleCount = sum([transfers.isFeasible]);
failedTransfers = transfers(~[transfers.isFeasible]);

task2TransferPlan = struct();
task2TransferPlan.generatedAt = string(datetime("now"));
task2TransferPlan.cacheFile = string(cacheFile);
task2TransferPlan.projectRoot = string(projectRoot);
task2TransferPlan.files = string(files);
task2TransferPlan.ranks = ranks;
task2TransferPlan.capturePoolXYList = capturePoolXYList;
task2TransferPlan.q5Mode = q5Mode;
task2TransferPlan.numTransfers = numTransfers;
task2TransferPlan.feasibleCount = feasibleCount;
task2TransferPlan.transfers = transfers;
task2TransferPlan.transferTable = transferTable;
task2TransferPlan.capturePoolTransferTable = capturePoolTransferTable;
task2TransferPlan.fromCapturePoolTransferTable = fromCapturePoolTransferTable;
task2TransferPlan.config = cfg;

save(cacheFile, "task2TransferPlan", "-v7.3");

fprintf("Saved Task 2 transfer IK cache:\n%s\n", cacheFile);
fprintf("Feasible transfers: %d/%d\n", feasibleCount, numTransfers);
if ~isempty(failedTransfers)
    fprintf("First failed transfers:\n");
    for i = 1:min(20, numel(failedTransfers))
        fprintf("  %s -> %s: %s\n", failedTransfers(i).fromSquare, ...
            failedTransfers(i).toSquare, failedTransfers(i).errorMessage);
    end
end
end

function transferInputs = buildTransferInputs(arm, files, ranks, capturePoolXYList)
numPickupSquares = numel(files) * numel(ranks);
numBoardTargets = numel(files) * numel(ranks);
numCaptureTargets = size(capturePoolXYList, 1);
transferInputs = repmat(emptyTransferInput(), 1, ...
    numPickupSquares * (numBoardTargets + numCaptureTargets) + ...
    numCaptureTargets * numBoardTargets);
inputIndex = 0;

for fromRankIndex = 1:numel(ranks)
    for fromFileIndex = 1:numel(files)
        fromSquare = string(files(fromFileIndex)) + string(ranks(fromRankIndex));
        fromXY = arm.squareToXY(fromSquare);

        for toRankIndex = 1:numel(ranks)
            for toFileIndex = 1:numel(files)
                toSquare = string(files(toFileIndex)) + string(ranks(toRankIndex));
                toXY = arm.squareToXY(toSquare);

                inputIndex = inputIndex + 1;
                transferInputs(inputIndex).index = inputIndex;
                transferInputs(inputIndex).fromSquare = fromSquare;
                transferInputs(inputIndex).toSquare = toSquare;
                transferInputs(inputIndex).fromFileIndex = fromFileIndex;
                transferInputs(inputIndex).fromRankIndex = fromRankIndex;
                transferInputs(inputIndex).fromPoolIndex = [];
                transferInputs(inputIndex).fromTargetType = "board";
                transferInputs(inputIndex).toFileIndex = toFileIndex;
                transferInputs(inputIndex).toRankIndex = toRankIndex;
                transferInputs(inputIndex).toPoolIndex = [];
                transferInputs(inputIndex).toTargetType = "board";
                transferInputs(inputIndex).fromXY = fromXY;
                transferInputs(inputIndex).toXY = toXY;
            end
        end

        for poolIndex = 1:numCaptureTargets
            inputIndex = inputIndex + 1;
            transferInputs(inputIndex).index = inputIndex;
            transferInputs(inputIndex).fromSquare = fromSquare;
            transferInputs(inputIndex).toSquare = "capture pool " + string(poolIndex);
            transferInputs(inputIndex).fromFileIndex = fromFileIndex;
            transferInputs(inputIndex).fromRankIndex = fromRankIndex;
            transferInputs(inputIndex).fromPoolIndex = [];
            transferInputs(inputIndex).fromTargetType = "board";
            transferInputs(inputIndex).toFileIndex = [];
            transferInputs(inputIndex).toRankIndex = [];
            transferInputs(inputIndex).toPoolIndex = poolIndex;
            transferInputs(inputIndex).toTargetType = "capturePool";
            transferInputs(inputIndex).fromXY = fromXY;
            transferInputs(inputIndex).toXY = capturePoolXYList(poolIndex, :);
        end
    end
end

for fromPoolIndex = 1:numCaptureTargets
    fromSquare = "capture pool " + string(fromPoolIndex);
    fromXY = capturePoolXYList(fromPoolIndex, :);

    for toRankIndex = 1:numel(ranks)
        for toFileIndex = 1:numel(files)
            toSquare = string(files(toFileIndex)) + string(ranks(toRankIndex));
            toXY = arm.squareToXY(toSquare);

            inputIndex = inputIndex + 1;
            transferInputs(inputIndex).index = inputIndex;
            transferInputs(inputIndex).fromSquare = fromSquare;
            transferInputs(inputIndex).toSquare = toSquare;
            transferInputs(inputIndex).fromFileIndex = [];
            transferInputs(inputIndex).fromRankIndex = [];
            transferInputs(inputIndex).fromPoolIndex = fromPoolIndex;
            transferInputs(inputIndex).fromTargetType = "capturePool";
            transferInputs(inputIndex).toFileIndex = toFileIndex;
            transferInputs(inputIndex).toRankIndex = toRankIndex;
            transferInputs(inputIndex).toPoolIndex = [];
            transferInputs(inputIndex).toTargetType = "board";
            transferInputs(inputIndex).fromXY = fromXY;
            transferInputs(inputIndex).toXY = toXY;
        end
    end
end
end

function transfer = computeTransferPlan(projectRoot, input, q5Mode) %#ok<INUSD>
addProjectPaths(projectRoot);
clear chessMoveModule
chessMoveModule

transfer = emptyTransfer();
transfer.index = input.index;
transfer.fromSquare = input.fromSquare;
transfer.toSquare = input.toSquare;
transfer.fromFileIndex = input.fromFileIndex;
transfer.fromRankIndex = input.fromRankIndex;
transfer.fromPoolIndex = input.fromPoolIndex;
transfer.fromTargetType = input.fromTargetType;
transfer.toFileIndex = input.toFileIndex;
transfer.toRankIndex = input.toRankIndex;
transfer.toPoolIndex = input.toPoolIndex;
transfer.toTargetType = input.toTargetType;
transfer.fromXY = input.fromXY;
transfer.toXY = input.toXY;

try
    qHome = arm.config.qHome;
    evalc("movePlan = arm.planPickAndPlaceXY(input.fromXY, input.toXY, qHome, q5Mode);");

    qRetreat = arm.ikXYZ(movePlan.placeAboveXYZ, movePlan.qPlace);
    qRetreat(5) = movePlan.qPlace(5);

    transfer.pickXYZ = movePlan.pickXYZ;
    transfer.placeXYZ = movePlan.placeXYZ;
    transfer.pickAboveXYZ = movePlan.pickAboveXYZ;
    transfer.placeAboveXYZ = movePlan.placeAboveXYZ;
    transfer.qHomeBefore = qHome;
    transfer.qAbovePick = withGripperAngle(movePlan.qAbovePick, ...
        arm.config.gripper.openAngle, arm.config);
    transfer.qPickOpen = withGripperAngle(movePlan.qPick, ...
        arm.config.gripper.openAngle, arm.config);
    transfer.qPickClosed = withGripperAngle(movePlan.qPick, ...
        arm.config.gripper.closeAngle, arm.config);
    transfer.qLiftClosed = withGripperAngle(movePlan.qAbovePick, ...
        arm.config.gripper.closeAngle, arm.config);
    transfer.qAbovePlaceClosed = withGripperAngle(movePlan.qAbovePlace, ...
        arm.config.gripper.closeAngle, arm.config);
    transfer.qPlaceClosed = withGripperAngle(movePlan.qPlace, ...
        arm.config.gripper.closeAngle, arm.config);
    transfer.qPlaceOpen = withGripperAngle(movePlan.qPlace, ...
        arm.config.gripper.openAngle, arm.config);
    transfer.qRetreatOpen = withGripperAngle(qRetreat, ...
        arm.config.gripper.openAngle, arm.config);
    transfer.frame5AngleHold = movePlan.frame5AngleHold;
    transfer.frame5AnglePick = arm.frame5GroundAngle(movePlan.qPick);
    transfer.frame5AnglePlace = arm.frame5GroundAngle(movePlan.qPlace);
    transfer.pickErr = posePositionError(arm, movePlan.qPick, movePlan.pickXYZ);
    transfer.placeErr = posePositionError(arm, movePlan.qPlace, movePlan.placeXYZ);
    transfer.isFeasible = transfer.pickErr <= arm.config.pickPlaceTolerance && ...
        transfer.placeErr <= arm.config.pickPlaceTolerance;
    transfer.errorMessage = "";
catch ME
    transfer.isFeasible = false;
    transfer.errorMessage = string(ME.message);
end

if mod(input.index, 100) == 0
    fprintf("Precomputed transfer %04d: %s -> %s, feasible=%d\n", ...
        input.index, input.fromSquare, input.toSquare, transfer.isFeasible);
end
end

function addProjectPaths(projectRoot)
addpath(projectRoot);
addpath(fullfile(projectRoot, "external", "UoM-RS-Starter_Code"));
addpath(fullfile(projectRoot, "external", "UoM_RS_MATLAB_LIBRARIES"));
addpath(genpath(fullfile(projectRoot, "matlab")));
end

function input = emptyTransferInput()
input = struct( ...
    "index", [], ...
    "fromSquare", "", ...
    "toSquare", "", ...
    "fromFileIndex", [], ...
    "fromRankIndex", [], ...
    "fromPoolIndex", [], ...
    "fromTargetType", "", ...
    "toFileIndex", [], ...
    "toRankIndex", [], ...
    "toPoolIndex", [], ...
    "toTargetType", "", ...
    "fromXY", [], ...
    "toXY", []);
end

function transfer = emptyTransfer()
transfer = struct( ...
    "index", [], ...
    "fromSquare", "", ...
    "toSquare", "", ...
    "fromFileIndex", [], ...
    "fromRankIndex", [], ...
    "fromPoolIndex", [], ...
    "fromTargetType", "", ...
    "toFileIndex", [], ...
    "toRankIndex", [], ...
    "toPoolIndex", [], ...
    "toTargetType", "", ...
    "fromXY", [], ...
    "toXY", [], ...
    "pickXYZ", [], ...
    "placeXYZ", [], ...
    "pickAboveXYZ", [], ...
    "placeAboveXYZ", [], ...
    "qHomeBefore", [], ...
    "qAbovePick", [], ...
    "qPickOpen", [], ...
    "qPickClosed", [], ...
    "qLiftClosed", [], ...
    "qAbovePlaceClosed", [], ...
    "qPlaceClosed", [], ...
    "qPlaceOpen", [], ...
    "qRetreatOpen", [], ...
    "frame5AngleHold", [], ...
    "frame5AnglePick", [], ...
    "frame5AnglePlace", [], ...
    "pickErr", inf, ...
    "placeErr", inf, ...
    "isFeasible", false, ...
    "errorMessage", "");
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
