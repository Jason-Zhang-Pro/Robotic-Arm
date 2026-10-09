function repairTask2TransferIKCache()
%% repairTask2TransferIKCache.m
% Repair only the failed entries in task2_transfer_ik_cache.mat.
%
% This script keeps the existing successful 64x64 transfer cache and tries
% heavier/more varied planning only on failed from-square -> to-square pairs.

scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptDir);
cd(projectRoot);

%% User variables
cacheFile = fullfile(scriptDir, "task2_transfer_ik_cache.mat");
backupFile = fullfile(scriptDir, ...
    "task2_transfer_ik_cache_before_repair.mat");
useParallel = true;
batchSize = 5;
q5ModesToTry = ["default", "placeFlip", "flip"];

%% Setup
setup_a4
cleanupSerialConnections()
load("UoM-RS-Var_Def.mat") %#ok<LOAD>
clear chessMoveModule
chessMoveModule

if exist(cacheFile, "file") ~= 2
    error("Task 2 transfer cache not found: %s", cacheFile);
end

data = load(cacheFile, "task2TransferPlan");
task2TransferPlan = data.task2TransferPlan;
transfers = task2TransferPlan.transfers;

failedIndices = find(~[transfers.isFeasible]);
fprintf("Current feasible transfers: %d/%d\n", ...
    sum([transfers.isFeasible]), numel(transfers));
fprintf("Failed transfers to repair: %d\n", numel(failedIndices));

if isempty(failedIndices)
    fprintf("Nothing to repair.\n");
    return;
end

if exist(backupFile, "file") ~= 2
    copyfile(cacheFile, backupFile);
    fprintf("Backup saved:\n%s\n", backupFile);
end

repairInputs = buildRepairInputs(transfers, failedIndices);

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
    fprintf("Using parallel batched repair.\n");
else
    fprintf("Parallel toolbox not available; using serial batched repair.\n");
end

recoveredCount = 0;
numBatches = ceil(numel(repairInputs) / batchSize);
fprintf("Repairing in %d batches, up to %d transfers per save.\n", ...
    numBatches, batchSize);

for batchIndex = 1:numBatches
    firstRepair = (batchIndex - 1) * batchSize + 1;
    lastRepair = min(batchIndex * batchSize, numel(repairInputs));
    batchInputs = repairInputs(firstRepair:lastRepair);
    batchResults = repmat(emptyTransfer(), 1, numel(batchInputs));

    fprintf("\nBatch %d/%d: repairing failed entries %d-%d of %d...\n", ...
        batchIndex, numBatches, firstRepair, lastRepair, numel(repairInputs));

    if canUseParallel
        parfor batchItemIndex = 1:numel(batchInputs)
            batchResults(batchItemIndex) = repairOneTransfer( ...
                projectRoot, batchInputs(batchItemIndex), q5ModesToTry);
        end
    else
        for batchItemIndex = 1:numel(batchInputs)
            batchResults(batchItemIndex) = repairOneTransfer( ...
                projectRoot, batchInputs(batchItemIndex), q5ModesToTry);
        end
    end

    batchRecoveredCount = 0;
    for batchItemIndex = 1:numel(batchResults)
        repaired = batchResults(batchItemIndex);
        originalIndex = batchInputs(batchItemIndex).index;
        if repaired.isFeasible
            transfers(originalIndex) = repaired;
            recoveredCount = recoveredCount + 1;
            batchRecoveredCount = batchRecoveredCount + 1;
        else
            transfers(originalIndex).errorMessage = repaired.errorMessage;
        end
    end

    task2TransferPlan = saveRepairProgress( ...
        cacheFile, task2TransferPlan, transfers, recoveredCount, ...
        firstRepair, lastRepair, numel(repairInputs));

    fprintf("Batch %d saved. Batch recovered %d/%d. Total recovered %d/%d. Feasible now %d/%d.\n", ...
        batchIndex, batchRecoveredCount, numel(batchResults), ...
        recoveredCount, numel(repairInputs), ...
        task2TransferPlan.feasibleCount, numel(transfers));
end

fprintf("\nRepair recovered: %d/%d\n", recoveredCount, numel(repairInputs));
fprintf("New feasible transfers: %d/%d\n", ...
    task2TransferPlan.feasibleCount, numel(transfers));
fprintf("Updated cache saved:\n%s\n", cacheFile);

remaining = transfers(~[transfers.isFeasible]);
if ~isempty(remaining)
    fprintf("Remaining failed transfers:\n");
    for i = 1:min(30, numel(remaining))
        fprintf("  %s -> %s: %s\n", remaining(i).fromSquare, ...
            remaining(i).toSquare, remaining(i).errorMessage);
    end
end
end

function repairInputs = buildRepairInputs(transfers, failedIndices)
repairInputs = repmat(emptyRepairInput(), 1, numel(failedIndices));

for i = 1:numel(failedIndices)
    move = transfers(failedIndices(i));
    repairInputs(i).index = failedIndices(i);
    if isfield(move, "destinationType")
        repairInputs(i).destinationType = move.destinationType;
    else
        repairInputs(i).destinationType = "board";
    end
    repairInputs(i).fromSquare = move.fromSquare;
    repairInputs(i).toSquare = move.toSquare;
    repairInputs(i).fromFileIndex = move.fromFileIndex;
    repairInputs(i).fromRankIndex = move.fromRankIndex;
    repairInputs(i).toFileIndex = move.toFileIndex;
    repairInputs(i).toRankIndex = move.toRankIndex;
    if isfield(move, "captureIndex")
        repairInputs(i).captureIndex = move.captureIndex;
    else
        repairInputs(i).captureIndex = [];
    end
    repairInputs(i).fromXY = move.fromXY;
    repairInputs(i).toXY = move.toXY;
end
end

function task2TransferPlan = saveRepairProgress(cacheFile, task2TransferPlan, ...
    transfers, recoveredCount, firstRepair, lastRepair, totalRepairCount)
transferTable = repmat(emptyTransfer(), 8, 8, 8, 8);
capturedPoolCount = repairedCapturedPoolCount(task2TransferPlan, transfers);
captureTransferTable = repmat(emptyTransfer(), 8, 8, capturedPoolCount);
for transferIndex = 1:numel(transfers)
    move = transfers(transferIndex);
    if isCaptureDestination(move)
        captureTransferTable(move.fromRankIndex, move.fromFileIndex, ...
            move.captureIndex) = move;
    else
        transferTable(move.fromRankIndex, move.fromFileIndex, ...
            move.toRankIndex, move.toFileIndex) = move;
    end
end

task2TransferPlan.transfers = transfers;
task2TransferPlan.transferTable = transferTable;
task2TransferPlan.captureTransferTable = captureTransferTable;
task2TransferPlan.capturedPoolCount = capturedPoolCount;
task2TransferPlan.feasibleCount = sum([transfers.isFeasible]);
task2TransferPlan.repairedAt = string(datetime("now"));
task2TransferPlan.repairRecoveredCount = recoveredCount;
task2TransferPlan.repairAttemptedCount = totalRepairCount;
task2TransferPlan.lastSavedRepairRange = [firstRepair, lastRepair];

save(cacheFile, "task2TransferPlan", "-v7.3");
end

function repaired = repairOneTransfer(projectRoot, input, q5ModesToTry)
addProjectPaths(projectRoot);
clear chessMoveModule
chessMoveModule

repaired = emptyTransfer();
repaired.index = input.index;
repaired.destinationType = input.destinationType;
repaired.fromSquare = input.fromSquare;
repaired.toSquare = input.toSquare;
repaired.fromFileIndex = input.fromFileIndex;
repaired.fromRankIndex = input.fromRankIndex;
repaired.toFileIndex = input.toFileIndex;
repaired.toRankIndex = input.toRankIndex;
repaired.captureIndex = input.captureIndex;
repaired.fromXY = input.fromXY;
repaired.toXY = input.toXY;

bestTransfer = repaired;
bestScore = inf;
messages = strings(1, 0);
qStartSeeds = buildQStartSeeds(arm, input);

for seedIndex = 1:size(qStartSeeds, 1)
    qStart = qStartSeeds(seedIndex, :);
    for modeIndex = 1:numel(q5ModesToTry)
        q5Mode = q5ModesToTry(modeIndex);
        candidate = computeTransferCandidate(arm, input, qStart, q5Mode);

        if candidate.isFeasible
            repaired = candidate;
            fprintf("Repaired %s -> %s using seed %d, q5Mode=%s\n", ...
                input.fromSquare, input.toSquare, seedIndex, q5Mode);
            return;
        end

        score = candidate.pickErr + candidate.placeErr;
        if score < bestScore
            bestScore = score;
            bestTransfer = candidate;
        end
        messages(end + 1) = candidate.errorMessage; %#ok<AGROW>
    end
end

repaired = bestTransfer;
repaired.errorMessage = strjoin(unique(messages, "stable"), " | ");
fprintf("Still failed %s -> %s\n", input.fromSquare, input.toSquare);
end

function qStartSeeds = buildQStartSeeds(arm, input)
qHome = arm.config.qHome;
qBias = arm.config.qBias;
qFromFree = arm.ikXYZ([input.fromXY, arm.config.motion.gripZ], qHome);
qToFree = arm.ikXYZ([input.toXY, arm.config.motion.gripZ], qFromFree);

qStartSeeds = [
    qHome;
    qBias;
    qFromFree;
    qToFree;
    qHome(1), -0.6, -0.8, 1.4, qHome(5), qHome(6);
    qHome(1), -0.9, -0.6, 1.5, qHome(5), qHome(6);
    qHome(1), -0.4, -1.0, 1.5, qHome(5), qHome(6);
    qHome(1),  0.6, -1.2, 1.0, qHome(5), qHome(6);
    qHome(1),  0.9, -1.4, 1.1, qHome(5), qHome(6)
];
end

function transfer = computeTransferCandidate(arm, input, qStart, q5Mode)
transfer = emptyTransfer();
transfer.index = input.index;
transfer.destinationType = input.destinationType;
transfer.fromSquare = input.fromSquare;
transfer.toSquare = input.toSquare;
transfer.fromFileIndex = input.fromFileIndex;
transfer.fromRankIndex = input.fromRankIndex;
transfer.toFileIndex = input.toFileIndex;
transfer.toRankIndex = input.toRankIndex;
transfer.captureIndex = input.captureIndex;
transfer.fromXY = input.fromXY;
transfer.toXY = input.toXY;

try
    evalc("movePlan = arm.planPickAndPlaceXY(input.fromXY, input.toXY, qStart, q5Mode);");

    qRetreat = arm.ikXYZ(movePlan.placeAboveXYZ, movePlan.qPlace);
    qRetreat(5) = movePlan.qPlace(5);

    transfer.pickXYZ = movePlan.pickXYZ;
    transfer.placeXYZ = movePlan.placeXYZ;
    transfer.pickAboveXYZ = movePlan.pickAboveXYZ;
    transfer.placeAboveXYZ = movePlan.placeAboveXYZ;
    transfer.qHomeBefore = qStart;
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
    transfer.errorMessage = "q5Mode=" + string(q5Mode) + ...
        ", pickErr=" + string(transfer.pickErr) + ...
        ", placeErr=" + string(transfer.placeErr);
catch ME
    transfer.isFeasible = false;
    transfer.errorMessage = "q5Mode=" + string(q5Mode) + ": " + string(ME.message);
end
end

function count = repairedCapturedPoolCount(task2TransferPlan, transfers)
if isfield(task2TransferPlan, "capturedPoolCount")
    count = task2TransferPlan.capturedPoolCount;
    return;
end

count = 0;
if isfield(transfers, "captureIndex")
    captureIndices = [transfers.captureIndex];
    if ~isempty(captureIndices)
        count = max(captureIndices);
    end
end
end

function tf = isCaptureDestination(move)
tf = isfield(move, "destinationType") && ...
    strcmpi(string(move.destinationType), "capture");
end

function addProjectPaths(projectRoot)
addpath(projectRoot);
addpath(fullfile(projectRoot, "external", "UoM-RS-Starter_Code"));
addpath(fullfile(projectRoot, "external", "UoM_RS_MATLAB_LIBRARIES"));
addpath(genpath(fullfile(projectRoot, "matlab")));
end

function input = emptyRepairInput()
input = struct( ...
    "index", [], ...
    "destinationType", "", ...
    "fromSquare", "", ...
    "toSquare", "", ...
    "fromFileIndex", [], ...
    "fromRankIndex", [], ...
    "toFileIndex", [], ...
    "toRankIndex", [], ...
    "captureIndex", [], ...
    "fromXY", [], ...
    "toXY", []);
end

function transfer = emptyTransfer()
transfer = struct( ...
    "index", [], ...
    "destinationType", "", ...
    "fromSquare", "", ...
    "toSquare", "", ...
    "fromFileIndex", [], ...
    "fromRankIndex", [], ...
    "toFileIndex", [], ...
    "toRankIndex", [], ...
    "captureIndex", [], ...
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
