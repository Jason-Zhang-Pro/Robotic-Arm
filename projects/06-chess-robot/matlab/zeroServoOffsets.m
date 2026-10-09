%% zeroServoOffsets.m
% Records the current physical posture as the MATLAB zero reference.
% This script does not move the motors and does not write to motor EEPROM.

scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptDir);
cd(projectRoot);

%% User variables
port = "COM5";
offsetFile = fullfile(projectRoot, "servo_zero_offsets.mat");

%% Setup
setup_a4
cleanupSerialConnections()
load("UoM-RS-Var_Def.mat")

availablePorts = serialportlist("available");
fprintf("Available serial ports:\n");
disp(availablePorts);

if ~any(availablePorts == port)
    error("Port %s is not available. Close Arduino Serial Monitor/MATLAB connections or update port.", port);
end

fprintf("Connecting on %s...\n", port);

bot = robot(port, MAX_ID, BAUDRATE);
try
    bot.ConnectToArduino();
catch ME
    fprintf("Failed to connect on %s.\n", port);
    fprintf("Check that the normal UoM-RS-Arduino_Firmware is uploaded, not the ID change/check sketch.\n");
    fprintf("Also close Arduino Serial Monitor and any other MATLAB bot objects.\n");
    rethrow(ME);
end

bot.getID();
fprintf("Number of connected motors: %d\n", bot.num_ID);
fprintf("Connected motor IDs:\n");
disp(bot.connected_ID);

if bot.num_ID == 0
    error("Arduino connected, but no motors were detected. Check motor power, Serial1 wiring, and IDs.");
end

bot.getFB();
motorOffset = bot.q_FB;
savedMotorOffset = motorOffset;

fprintf("Current raw feedback saved as zero offset:\n");
disp(motorOffset);

save(offsetFile, "motorOffset", "savedMotorOffset");
bot.setOffset(motorOffset);

bot.getFB();
fprintf("Feedback after applying saved offset. These should be near zero:\n");
disp(bot.q_FB);

fprintf("Saved zero offsets to:\n%s\n", offsetFile);
fprintf("No motor movement command was sent.\n");
