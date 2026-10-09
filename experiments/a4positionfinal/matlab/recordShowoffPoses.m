function recordShowoffPoses()
%% recordShowoffPoses.m
% Record current robot postures for a showoff loop.
% Press r in the control window to record the current posture.
% Press c in the control window to finish and save.

scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptDir);
cd(projectRoot);

%% User variables
port = "COM5";
outputFile = fullfile(scriptDir, "showoff_pose_recording.mat");
samplePause = 0.15;
displayEvery = 1;
applySavedOffset = true;

%% Setup
setup_a4
cleanupSerialConnections()
load("UoM-RS-Var_Def.mat", "MAX_ID", "BAUDRATE")
clear chessMoveModule
chessMoveModule

bot = robot(port, MAX_ID, BAUDRATE);
bot.ConnectToArduino();

if applySavedOffset
    arm.applySavedOffset(bot);
end

bot.getID();
connectedIDs = bot.connected_ID(1:bot.num_ID);
fprintf("Connected motor IDs:\n");
disp(connectedIDs);

requiredArmIDs = 1:arm.config.arm.numKinematicJoints;
missingArmIDs = setdiff(requiredArmIDs, connectedIDs);
if ~isempty(missingArmIDs)
    error("Cannot record showoff poses. Missing required arm motor IDs: %s", ...
        mat2str(missingArmIDs));
end

if ~ismember(arm.config.gripper.motorIndex, connectedIDs)
    warning("Gripper motor ID %d was not detected. Gripper values will still be recorded from feedback.", ...
        arm.config.gripper.motorIndex);
end

recordedQ = zeros(0, arm.config.motor.numJoints);
recordedAt = strings(0, 1);
sampleIndex = 0;

fig = figure( ...
    "Name", "Showoff pose recorder: r = record, c = complete", ...
    "NumberTitle", "off", ...
    "MenuBar", "none", ...
    "ToolBar", "none", ...
    "KeyPressFcn", @recordShowoffKeyPress);
setappdata(fig, "lastKey", "");
axis off
text(0.5, 0.6, "r: record current posture", ...
    "HorizontalAlignment", "center", "FontSize", 14);
text(0.5, 0.4, "c: finish and save", ...
    "HorizontalAlignment", "center", "FontSize", 14);

fprintf("Recording showoff poses. Focus the recorder window, press r to record, c to save.\n");

while ishandle(fig)
    sampleIndex = sampleIndex + 1;
    qModel = arm.readCurrentQ(bot);
    qMotorOffset = bot.q_FB;

    if mod(sampleIndex, displayEvery) == 0
        currentTime = string(datetime("now", "Format", "HH:mm:ss.SSS"));
        fprintf("\n%s | recorded poses: %d\n", currentTime, size(recordedQ, 1));
        fprintf("model rad: ");
        fprintf("%10.5f ", qModel);
        fprintf("\n");
        fprintf("model deg: ");
        fprintf("%10.2f ", rad2deg(qModel));
        fprintf("\n");
        fprintf("motor rad: ");
        fprintf("%10.5f ", qMotorOffset);
        fprintf("\n");
        fprintf("motor deg: ");
        fprintf("%10.2f ", rad2deg(qMotorOffset));
        fprintf("\n");
    end

    drawnow
    lastKey = string(getappdata(fig, "lastKey"));
    if lastKey == "r"
        recordedQ(end + 1, :) = qModel; %#ok<AGROW>
        recordedAt(end + 1, 1) = string(datetime("now")); %#ok<AGROW>
        fprintf("Recorded showoff pose %d.\n", size(recordedQ, 1));
        setappdata(fig, "lastKey", "");
    elseif lastKey == "c"
        break;
    end

    pause(samplePause);
end

if ishandle(fig)
    close(fig);
end

showoffPlan = struct();
showoffPlan.generatedAt = string(datetime("now"));
showoffPlan.outputFile = string(outputFile);
showoffPlan.port = port;
showoffPlan.recordedQ = recordedQ;
showoffPlan.recordedAt = recordedAt;
showoffPlan.numPoses = size(recordedQ, 1);
showoffPlan.config = arm.config;
showoffPlan.gripper.openAngle = arm.config.gripper.openAngle;
showoffPlan.gripper.closeAngle = arm.config.gripper.closeAngle;
showoffPlan.gripper.releaseAngle = arm.config.gripper.openAngle + ...
    arm.config.gripper.prePickExtraOpenAngle;
showoffPlan.gripper.prePickExtraOpenAngle = arm.config.gripper.prePickExtraOpenAngle;

save(outputFile, "showoffPlan");

fprintf("Saved %d showoff poses to:\n%s\n", showoffPlan.numPoses, outputFile);
end

function recordShowoffKeyPress(fig, event)
key = lower(string(event.Key));
if key == "r" || key == "c"
    setappdata(fig, "lastKey", key);
end
end
