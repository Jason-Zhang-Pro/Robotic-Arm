%% testMoveModuleHomeZero.m
% Test moving the robot to all-zero joint posture using chessMoveModule.
% This uses the move module's saved offset, interpolation, and angle wrapping.

scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptDir);
cd(projectRoot);

%% User variables
port = "COM5";
targetQ = [0, 0, 0, 0, 0, deg2rad(-25)];
requireConfirmation = true;

%% Setup
setup_a4
cleanupSerialConnections()
load("UoM-RS-Var_Def.mat")
chessMoveModule

fprintf("This script will MOVE the robot to targetQ:\n");
disp(targetQ);

if requireConfirmation
    reply = input('Type GO to move using chessMoveModule: ', 's');
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

qStart = arm.readCurrentQ(bot);
fprintf("Current offset-corrected feedback qStart:\n");
disp(qStart);
fprintf("Move module timing: numSteps = %d, stepPause = %.3f s, estimated duration = %.2f s\n", ...
    arm.config.motion.numSteps, ...
    arm.config.motion.stepPause, ...
    arm.config.motion.numSteps * arm.config.motion.stepPause);

%% Move to zero through move module
arm.moveJoints(bot, qStart, targetQ);

pause(0.5);
qFinal = arm.readCurrentQ(bot);

fprintf("Final offset-corrected feedback:\n");
disp(qFinal);

fprintf("Move module zero test complete.\n");
