%% testJoystickInput.m
% Safe joystick input test for Assignment 4.
% This script only reads joystick values from the Arduino. It does not
% command any motor motion.

scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptDir);
cd(projectRoot);

%% User variables
port = "COM5";
samplePause = 0.10;        % Seconds between readings
numSamples = inf;          % Set to a number for a finite test
deadband = 0.08;           % Normalised centre deadband
adcMaxRaw = 687;           % Measured full-scale value with joystick on 3.3 V

%% Setup
setup_a4
cleanupSerialConnections()
load("UoM-RS-Var_Def.mat")

availablePorts = serialportlist("available");
fprintf("Available serial ports:\n");
disp(availablePorts);

if ~any(availablePorts == port)
    error("Port %s is not available. Update port or close other serial tools.", port);
end

fprintf("Connecting to Arduino on %s...\n", port);
bot = robot(port, MAX_ID, BAUDRATE);
bot.ConnectToArduino();

fprintf("Connected. Reading joystick only. Press Ctrl+C to stop.\n");
fprintf("Using returned column order: raw_sw raw_x raw_y.\n");
fprintf("Button is treated as pressed when raw_sw is 1.\n");
fprintf("Columns: raw_sw raw_x raw_y | norm_x norm_y btn\n\n");

centreRaw = readJoystickRaw(bot);
fprintf("Centre sample used for normalisation:\n");
disp(centreRaw);

sampleIndex = 0;
while sampleIndex < numSamples
    sampleIndex = sampleIndex + 1;

    raw = readJoystickRaw(bot);
    normalised = normaliseJoystick(raw, centreRaw, deadband, adcMaxRaw);

    fprintf("%04d  %5d %5d %5d | %+6.3f %+6.3f %d\n", ...
        sampleIndex, raw(1), raw(2), raw(3), ...
        normalised(1), normalised(2), normalised(3));

    pause(samplePause);
end

function raw = readJoystickRaw(bot)
% Read raw [button x y] from the Arduino without using robot.getJOY().
% robot.getJOY applies centre offset and nonlinear scaling internally.
writeline(bot.serial_obj, "JOY");
[raw, err] = readSerial(bot.serial_obj, 3, false);
if err
    pause(0.02);
    flush(bot.serial_obj);
    writeline(bot.serial_obj, "JOY");
    [raw, err] = readSerial(bot.serial_obj, 3, false);
    if err
        warning("Joystick serial read reported an error.");
    end
end
raw = reshape(raw, 1, []);
end

function normalised = normaliseJoystick(raw, centreRaw, deadband, adcMaxRaw)
% Map returned [button x y] to [normalised_x normalised_y button].
normalised = zeros(1, 3);

for axisIndex = 1:2
    rawIndex = axisIndex + 1;
    delta = double(raw(rawIndex) - centreRaw(rawIndex));
    if delta >= 0
        scale = max(1, double(adcMaxRaw) - double(centreRaw(rawIndex)));
    else
        scale = max(1, double(centreRaw(rawIndex)));
    end

    value = delta / scale;
    value = max(-1, min(1, value));
    if abs(value) < deadband
        value = 0;
    end
    normalised(axisIndex) = value;
end

normalised(3) = raw(1) == 1;
end
