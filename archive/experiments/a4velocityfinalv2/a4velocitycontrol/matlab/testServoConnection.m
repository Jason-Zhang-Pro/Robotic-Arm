%% testServoConnection.m
% Safe communication test for MATLAB -> Arduino -> servo feedback.
% This script only reads motor IDs and feedback. It does not command motion.

scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptDir);
cd(projectRoot);

if exist("bot", "var") && isprop(bot, "serial_obj")
    try
        delete(bot.serial_obj);
    catch
    end
    clear bot
end

try
    delete(serialportfind("all"));
catch
    try
        delete(serialportfind);
    catch
    end
end

setup_a4
load("UoM-RS-Var_Def.mat")

port = "COM5";
feedbackSamples = 20;
samplePause = 0.5;

fprintf("Testing robot connection on %s...\n", port);

if exist("serialportfind", "file") == 2
    try
        oldSerialObjects = serialportfind("all");
    catch
        oldSerialObjects = serialportfind;
    end
    if ~isempty(oldSerialObjects)
        delete(oldSerialObjects);
    end
elseif exist("instrfind", "file") == 2
    oldSerialObjects = instrfind;
    if ~isempty(oldSerialObjects)
        fclose(oldSerialObjects);
        delete(oldSerialObjects);
    end
end

bot = robot(port, MAX_ID, BAUDRATE);
bot.ConnectToArduino();

fprintf("Arduino handshake complete.\n");

bot.getID();
fprintf("Number of connected motors: %d\n", bot.num_ID);
fprintf("Connected motor IDs:\n");
disp(bot.connected_ID);

if bot.num_ID == 0
    warning("MATLAB connected to Arduino on %s, but Arduino reported 0 motors.", port);
    warning("Check servo power, TTL bus wiring, motor IDs, and that the UoM-RS firmware is uploaded.");
end

fprintf("Reading motor feedback. Gently move one joint by hand and watch values change.\n");
for sampleIndex = 1:feedbackSamples
    bot.getFB();
    fprintf("Sample %02d: ", sampleIndex);
    fprintf("%8.4f ", bot.q_FB);
    fprintf("\n");
    pause(samplePause);
end

fprintf("Feedback test complete. No motion commands were sent.\n");
