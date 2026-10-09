function precomputeTask2TransferIK()
%% precomputeTask2TransferIK.m
% Precompute board-to-board and board-to-captured-pool Task 2 transfer IK plans.
%
% The cache contains every board pickup square to every board target square,
% plus every board pickup square to each captured-pool position configured in
% chessMoveModule.

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
capturedPoolXYList = cfg.capture.poolXYList;
transferInputs = buildTransferInputs(arm, files, ranks);
numTransfers = numel(transferInputs);
boardSquareCount = numel(files) * numel(ranks);
capturedPoolCount = size(capturedPoolXYList, 1);

fprintf("Precomputing Task 2 transfer IK cache...\n");
fprintf("Transfers: %d from/to pairs (%d starts x (%d board targets + %d captured-pool targets)).\n", ...
    numTransfers, boardSquareCount, boardSquareCount, capturedPoolCount);
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
captureTransferTable = repmat(emptyTransfer(), 8, 8, capturedPoolCount);
for transferIndex = 1:numTransfers
    move = transfers(transferIndex);
    if strcmpi(string(move.destinationType), "capture")
        captureTransferTable(move.fromRankIndex, move.fromFileIndex, ...
            move.captureIndex) = move;
    else
        transferTable(move.fromRankIndex, move.fromFileIndex, ...
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
task2TransferPlan.boardSquareCount = boardSquareCount;
task2TransferPlan.capturedPoolCount = capturedPoolCount;
task2TransferPlan.capturedPoolXYList = capturedPoolXYList;
task2TransferPlan.q5Mode = q5Mode;
task2TransferPlan.numTransfers = numTransfers;
task2TransferPlan.feasibleCount = feasibleCount;
task2TransferPlan.transfers = transfers;
task2TransferPlan.transferTable = transferTable;
task2TransferPlan.captureTransferTable = captureTransferTable;
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

function transferInputs = buildTransferInputs(arm, files, ranks)
boardSquareCount = numel(files) * numel(ranks);
capturedPoolXYList = arm.config.capture.poolXYList;
capturedPoolCount = size(capturedPoolXYList, 1);
transferInputs = repmat(emptyTransferInput(), 1, ...
    boardSquareCount * (boardSquareCount + capturedPoolCount));
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
                transferInputs(inputIndex).destinationType = "board";
                transferInputs(inputIndex).fromSquare = fromSquare;
                transferInputs(inputIndex).toSquare = toSquare;
                transferInputs(inputIndex).fromFileIndex = fromFileIndex;
                transferInputs(inputIndex).fromRankIndex = fromRankIndex;
                transferInputs(inputIndex).toFileIndex = toFileIndex;
                transferInputs(inputIndex).toRankIndex = toRankIndex;
                transferInputs(inputIndex).captureIndex = [];
                transferInputs(inputIndex).fromXY = fromXY;
                transferInputs(inputIndex).toXY = toXY;
            end
        end

        for captureIndex = 1:capturedPoolCount
            inputIndex = inputIndex + 1;
            transferInputs(inputIndex).index = inputIndex;
            transferInputs(inputIndex).destinationType = "capture";
            transferInputs(inputIndex).fromSquare = fromSquare;
            transferInputs(inputIndex).toSquare = "captured pool " + string(captureIndex);
            transferInputs(inputIndex).fromFileIndex = fromFileIndex;
            transferInputs(inputIndex).fromRankIndex = fromRankIndex;
            transferInputs(inputIndex).toFileIndex = [];
            transferInputs(inputIndex).toRankIndex = [];
            transferInputs(inputIndex).captureIndex = captureIndex;
            transferInputs(inputIndex).fromXY = fromXY;
            transferInputs(inputIndex).toXY = capturedPoolXYList(captureIndex, :);
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
