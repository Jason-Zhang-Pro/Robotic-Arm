function cleanupSerialConnections()
% cleanupSerialConnections
% Releases old MATLAB serial objects and a lingering base-workspace bot.

try
    evalin("base", "if exist('bot','var'), try, delete(bot.serial_obj); catch, end, clear bot, end");
catch
end

try
    delete(serialportfind("all"));
catch
    try
        delete(serialportfind);
    catch
    end
end

try
    oldSerialObjects = instrfind;
    if ~isempty(oldSerialObjects)
        fclose(oldSerialObjects);
        delete(oldSerialObjects);
    end
catch
end

try
    clear serialport
catch
end
end
