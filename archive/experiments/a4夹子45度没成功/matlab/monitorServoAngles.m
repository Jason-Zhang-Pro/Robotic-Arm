%% monitorServoAngles.m
% Real-time servo angle monitor in radians and degrees.
% Press Ctrl+C in MATLAB Command Window to stop.

scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptDir);
cd(projectRoot);

%% User variables
port = "COM4";
samplePause = 0.25;       % Seconds between readings
applySavedOffset = true;  % Show angles relative to saved zero offset
showRawMotorAngles = true; % Also show raw motor-frame feedback

%% Setup
setup_a4
cleanupSerialConnections()
load("UoM-RS-Var_Def.mat")
chessMoveModule

bot = robot(port, MAX_ID, BAUDRATE);
bot.ConnectToArduino();

bot.getID();
connectedIDs = bot.connected_ID(1:bot.num_ID);

fprintf("Connected motor IDs:\n");
disp(connectedIDs);

if applySavedOffset
    arm.applySavedOffset(bot);
end

fprintf("Monitoring servo angles. Press Ctrl+C to stop.\n");

while true
    qModelRad = arm.readCurrentQ(bot);
    qModelDeg = rad2deg(qModelRad);

    fprintf("\n%s\n", datestr(now, "HH:MM:SS.FFF"));
    fprintf("ID       : ");
    fprintf("%10d ", 1:arm.config.motor.numJoints);
    fprintf("\n");

    fprintf("model rad: ");
    fprintf("%10.5f ", qModelRad);
    fprintf("\n");

    fprintf("model deg: ");
    fprintf("%10.2f ", qModelDeg);
    fprintf("\n");

    if showRawMotorAngles
        rawRad = bot.q_FB(1:arm.config.motor.numJoints);
        rawDeg = rad2deg(rawRad);

        fprintf("raw rad  : ");
        fprintf("%10.5f ", rawRad);
        fprintf("\n");

        fprintf("raw deg  : ");
        fprintf("%10.2f ", rawDeg);
        fprintf("\n");
    end

    if ~isempty(connectedIDs)
        fprintf("online   : ");
        for id = 1:arm.config.motor.numJoints
            if any(connectedIDs == id)
                fprintf("%10s ", "yes");
            else
                fprintf("%10s ", "no");
            end
        end
        fprintf("\n");
    end

    pause(samplePause);
end
