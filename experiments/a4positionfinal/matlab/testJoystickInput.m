%% testJoystickInput.m
% Safe raw analog input test for Assignment 4.
% This script only reads A0, A1, and A2 from the Arduino.
% It does not command any motor motion.

scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptDir);
cd(projectRoot);

%% User variables
port = "COM5";
samplePause = 0.10;
numSamples = inf;
adcMaxRaw = 1023;
buttonThreshold = 10;
centreRawXY = [518, 499];
deadband = 0.08;

%% Setup
setup_a4
cleanupSerialConnections()
load("UoM-RS-Var_Def.mat", "MAX_ID", "BAUDRATE")

availablePorts = serialportlist("available");
fprintf("Available serial ports:\n");
disp(availablePorts);

if ~any(availablePorts == port)
    error("Port %s is not available. Update port or close other serial tools.", port);
end

fprintf("Connecting to Arduino on %s...\n", port);
bot = robot(port, MAX_ID, BAUDRATE);
bot.ConnectToArduino();

fprintf("Connected. Reading raw analog values only. Press Ctrl+C to stop.\n");
fprintf("Using measured column order: raw_sw raw_x raw_y.\n");
fprintf("Button threshold: raw_sw < %d means pressed.\n", buttonThreshold);
fprintf("Joystick centre: raw_x = %d, raw_y = %d, range 0-%d.\n", ...
    centreRawXY(1), centreRawXY(2), adcMaxRaw);
fprintf("Columns: raw_sw raw_x raw_y | norm_x norm_y btn\n\n");

sampleIndex = 0;
while sampleIndex < numSamples
    sampleIndex = sampleIndex + 1;

    raw = readRawAnalogPins(bot);
    normalised = normaliseJoystick(raw, centreRawXY, deadband, adcMaxRaw, ...
        buttonThreshold);

    fprintf("%04d  %6d %6d %6d | %+6.3f %+6.3f %d\n", ...
        sampleIndex, raw(1), raw(2), raw(3), ...
        normalised(1), normalised(2), normalised(3));

    pause(samplePause);
end

function raw = readRawAnalogPins(bot)
writeline(bot.serial_obj, "JOY");
[raw, err] = readSerial(bot.serial_obj, 3, false);
if err
    pause(0.02);
    flush(bot.serial_obj);
    writeline(bot.serial_obj, "JOY");
    [raw, err] = readSerial(bot.serial_obj, 3, false);
    if err
        warning("Raw analog serial read reported an error.");
    end
end
raw = reshape(raw, 1, []);
end

function normalised = normaliseJoystick(raw, centreRawXY, deadband, adcMaxRaw, ...
    buttonThreshold)
normalised = zeros(1, 3);

for axisIndex = 1:2
    rawIndex = axisIndex + 1;
    centre = centreRawXY(axisIndex);
    delta = double(raw(rawIndex) - centre);
    if delta >= 0
        scale = max(1, double(adcMaxRaw) - double(centre));
    else
        scale = max(1, double(centre));
    end

    value = delta / scale;
    value = max(-1, min(1, value));
    if abs(value) < deadband
        value = 0;
    end
    normalised(axisIndex) = value;
end

normalised(3) = raw(1) < buttonThreshold;
end
