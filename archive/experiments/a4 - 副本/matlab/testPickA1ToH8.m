%% testPickA1ToH8.m
% Test pick-and-place from A1 to H8 using chessMoveModule.
% The pick/place poses keep the same frame5-ground angle internally.

scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptDir);
cd(projectRoot);

%% User variables
port = "COM5";
fromSquare = "A1";
toSquare = "H8";
requireConfirmation = true;

%% Setup
setup_a4
cleanupSerialConnections()
load("UoM-RS-Var_Def.mat")
chessMoveModule

startXY = arm.squareToXY(fromSquare);
finalXY = arm.squareToXY(toSquare);

fprintf("This script will MOVE a piece from %s to %s.\n", fromSquare, toSquare);
fprintf("Start XY:\n");
disp(startXY);
fprintf("Final XY:\n");
disp(finalXY);
fprintf("Pick/place will use the same frame5 x-axis ground angle inside arm.pickAndPlaceXY.\n");

if requireConfirmation
    reply = input('Type GO to run A1 -> H8 pick-and-place: ', 's');
    if ~strcmp(reply, 'GO')
        fprintf("Cancelled. No motion command was sent.\n");
        return;
    end
end

%% Connect
bot = robot(port, MAX_ID, BAUDRATE);
bot.ConnectToArduino();

arm.applySavedOffset(bot);

bot.getID();
fprintf("Number of connected motors: %d\n", bot.num_ID);
fprintf("Connected motor IDs:\n");
disp(bot.connected_ID);

requiredIDs = 1:5;
missingIDs = setdiff(requiredIDs, bot.connected_ID(1:bot.num_ID));
if ~isempty(missingIDs)
    error("Cannot run pick/place test. Missing required arm motor IDs: %s", ...
        mat2str(missingIDs));
end

qStart = arm.readCurrentQ(bot);
fprintf("Current offset-corrected feedback qStart:\n");
disp(qStart);

movePlan = arm.planPickAndPlaceXY(startXY, finalXY, qStart);
qPickFree = movePlan.qPickFree;
qPlaceFree = movePlan.qPlaceFree;
frame5AnglePickFree = movePlan.frame5AnglePickFree;
frame5AnglePlaceFree = movePlan.frame5AnglePlaceFree;
frame5AngleHold = movePlan.frame5AngleHold;
qPick = movePlan.qPick;
qPlace = movePlan.qPlace;
qAbovePick = movePlan.qAbovePick;
qAbovePlace = movePlan.qAbovePlace;

fprintf("Free pick/place frame5 x-axis ground angles: %.4f rad / %.2f deg, %.4f rad / %.2f deg\n", ...
    frame5AnglePickFree, rad2deg(frame5AnglePickFree), ...
    frame5AnglePlaceFree, rad2deg(frame5AnglePlaceFree));
fprintf("Held frame5 x-axis ground angle: %.4f rad / %.2f deg\n", ...
    frame5AngleHold, rad2deg(frame5AngleHold));
fprintf("Constrained pick/place frame5 x-axis ground angles: %.4f rad / %.2f deg, %.4f rad / %.2f deg\n", ...
    arm.frame5GroundAngle(qPick), rad2deg(arm.frame5GroundAngle(qPick)), ...
    arm.frame5GroundAngle(qPlace), rad2deg(arm.frame5GroundAngle(qPlace)));
fprintf("Pick/place frame5 x-axis ground angle errors: %.4f rad / %.2f deg, %.4f rad / %.2f deg\n", ...
    abs(frame5AngleHold - arm.frame5GroundAngle(qPick)), ...
    rad2deg(abs(frame5AngleHold - arm.frame5GroundAngle(qPick))), ...
    abs(frame5AngleHold - arm.frame5GroundAngle(qPlace)), ...
    rad2deg(abs(frame5AngleHold - arm.frame5GroundAngle(qPlace))));
printPoseError(arm, "Pick A1", qPick, [startXY, arm.config.motion.gripZ]);
printPoseError(arm, "Place H8", qPlace, [finalXY, arm.config.motion.gripZ]);
printElbowFrame5Heights(arm, ...
    [qStart; qAbovePick; qPick; qAbovePlace; qPlace], ...
    ["Current"; "Above A1"; "Pick A1"; "Above H8"; "Place H8"]);

plotStickFigureKeyPoses(arm, ...
    [qStart; qAbovePick; qPick; qAbovePlace; qPlace], ...
    ["Current"; "Above A1"; "Pick A1"; "Above H8"; "Place H8"]);

arm.pickAndPlaceXY(bot, startXY, finalXY, qStart);

fprintf("A1 -> H8 pick-and-place test complete.\n");

function plotStickFigureKeyPoses(arm, qList, labels)
figure("Name", "A1 to H8 key robot poses", "Color", "w");
hold on; grid on; axis equal;
xlabel("x [m]");
ylabel("y [m]");
zlabel("z [m]");
title("Stick figure key poses");
view(45, 25);

colors = lines(size(qList, 1));

for i = 1:size(qList, 1)
    jointXYZ = arm.jointPositions(qList(i, :));
    plot3(jointXYZ(:, 1), jointXYZ(:, 2), jointXYZ(:, 3), "-o", ...
        "LineWidth", 1.5, ...
        "MarkerSize", 5, ...
        "Color", colors(i, :), ...
        "DisplayName", labels(i));

    text(jointXYZ(end, 1), jointXYZ(end, 2), jointXYZ(end, 3), ...
        " " + labels(i), ...
        "Color", colors(i, :), ...
        "FontSize", 9);
end

boardX = [arm.config.board.nearEdgeX, arm.config.board.farEdgeX, ...
          arm.config.board.farEdgeX, arm.config.board.nearEdgeX, ...
          arm.config.board.nearEdgeX];
boardY = [arm.config.board.yMin, arm.config.board.yMin, ...
          arm.config.board.yMax, arm.config.board.yMax, ...
          arm.config.board.yMin];
boardZ = arm.config.motion.gripZ * ones(size(boardX));
plot3(boardX, boardY, boardZ, "k-", "LineWidth", 1.2, "DisplayName", "Board edge");

legend("Location", "bestoutside");
end

function printElbowFrame5Heights(arm, qList, labels)
fprintf("\nElbow / frame5 height check:\n");
for i = 1:size(qList, 1)
    jointXYZ = arm.jointPositions(qList(i, :));
    elbowZ = jointXYZ(4, 3);
    frame5Z = jointXYZ(6, 3);
    fprintf("%-10s elbow z = %.4f m, frame5 z = %.4f m, diff = %.4f m\n", ...
        labels(i), elbowZ, frame5Z, elbowZ - frame5Z);
end
end

function printPoseError(arm, label, q, targetXYZ)
T = arm.fk(q);
actualXYZ = T(1:3, 4)';
err = norm(actualXYZ - targetXYZ);
fprintf("%s target XYZ: [%.4f %.4f %.4f], actual XYZ: [%.4f %.4f %.4f], error = %.4f m\n", ...
    label, targetXYZ(1), targetXYZ(2), targetXYZ(3), ...
    actualXYZ(1), actualXYZ(2), actualXYZ(3), err);
end
