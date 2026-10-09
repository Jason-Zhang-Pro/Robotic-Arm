%% driveServosToZero.m
% Drives the connected servos to a target 180 degree posture.
% WARNING: This sends motion commands to the motors.

scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptDir);
cd(projectRoot);

%% User variables
port = "COM5";
targetQ = pi * ones(1, 6);       % Target servo angles [rad], pi = 180 deg
numMoveSteps = 80;                % More steps = slower, smoother motion
stepPause = 0.04;                 % Pause between commands [s]
targetTolerance = 1e-10;           % Final angle tolerance check [rad]
maxZeroAttempts = 10;             % Retry moving to target this many times
retryKickAngle = pi - 1.0;        % Move here before retrying target [rad]
errorCompensationGain = 0.1;      % Extra correction based on previous error
requireConfirmation = true;       % Set false only after testing safely

%% Setup
setup_a4
load("UoM-RS-Var_Def.mat")

targetQ = reshape(targetQ, 1, []);
if numel(targetQ) < MAX_ID
    targetQ = [targetQ, zeros(1, MAX_ID - numel(targetQ))];
elseif numel(targetQ) > MAX_ID
    targetQ = targetQ(1:MAX_ID);
end

fprintf("This script will MOVE servos on %s to targetQ:\n", port);
disp(targetQ);
fprintf("TargetQ in degrees:\n");
disp(rad2deg(targetQ));

if requireConfirmation
    reply = input('Type GO to move servos to 180 deg: ', 's');
    if ~strcmp(reply, 'GO')
        fprintf("Cancelled. No motion command was sent.\n");
        return;
    end
end

%% Connect and read current posture
bot = robot(port, MAX_ID, BAUDRATE);
bot.ConnectToArduino();

bot.getID();
fprintf("Number of connected motors: %d\n", bot.num_ID);
fprintf("Connected motor IDs:\n");
disp(bot.connected_ID);

if ~any(bot.connected_ID == 6)
    warning("Motor ID 6 is not online, so the gripper servo cannot be driven to 180 deg.");
end

bot.getFB();
qStart = bot.q_FB;

fprintf("Current feedback qStart:\n");
disp(qStart);

bot.setDriveControlMode(zeros(1, MAX_ID));

%% Smoothly move to targetQ, retrying via a nearby posture if needed
connectedIDs = bot.connected_ID(1:bot.num_ID);
qCurrent = qStart;
targetCommandQ = targetQ;
success = false;

for attempt = 1:maxZeroAttempts
    fprintf("Target attempt %d of %d...\n", attempt, maxZeroAttempts);
    moveSmooth(bot, qCurrent, targetCommandQ, numMoveSteps, stepPause);

    pause(0.5);
    bot.getFB();
    qCurrent = bot.q_FB;
    finalError = abs(qCurrent - targetQ);

    fprintf("Feedback after target attempt %d:\n", attempt);
    disp(qCurrent);
    fprintf("Absolute final error:\n");
    disp(finalError);

    retryIDs = connectedIDs(finalError(connectedIDs) > targetTolerance);

    if isempty(retryIDs)
        success = true;
        fprintf("All connected motors are within targetTolerance = %.8g rad.\n", targetTolerance);
        break;
    end

    if attempt < maxZeroAttempts
        fprintf("Motors outside tolerance: ");
        fprintf("%d ", retryIDs);
        fprintf("\n");
        fprintf("Moving only those motors to %.4f rad before retrying target.\n", retryKickAngle);

        measuredError = qCurrent - targetQ;
        targetCommandQ = targetQ;
        targetCommandQ(retryIDs) = targetQ(retryIDs) - ...
            errorCompensationGain * measuredError(retryIDs);

        fprintf("Next compensated target command for motors outside tolerance:\n");
        disp(targetCommandQ);

        kickQ = qCurrent;
        kickQ(retryIDs) = retryKickAngle;
        moveSmooth(bot, qCurrent, kickQ, numMoveSteps, stepPause);
        pause(0.5);
        bot.getFB();
        qCurrent = bot.q_FB;
    end
end

if ~success
    warning("Some connected motors are still outside targetTolerance = %.8g rad after %d attempts.", ...
        targetTolerance, maxZeroAttempts);
end

fprintf("Move complete.\n");

function moveSmooth(bot, qStart, qTarget, numMoveSteps, stepPause)
for k = 1:numMoveSteps
    s = k / numMoveSteps;
    s = s * s * (3 - 2 * s);
    qCmd = (1 - s) * qStart + s * qTarget;

    bot.setQ(qCmd);
    bot.driveMotors();
    pause(stepPause);
end
end
