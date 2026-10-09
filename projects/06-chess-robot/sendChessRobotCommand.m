function sendChessRobotCommand(commandName, varargin)
%% sendChessRobotCommand.m
% Write a command for the persistent chess robot MATLAB session.
%
% Examples:
%   sendChessRobotCommand("MOVE", "E2", "E4")
%   sendChessRobotCommand("HOME")
%   sendChessRobotCommand("STOP")

projectRoot = fileparts(mfilename("fullpath"));
commandFile = fullfile(projectRoot, "matlab", "robot_command.txt");

commandName = upper(string(commandName));
commandId = string(datetime("now", "Format", "yyyyMMddHHmmssSSS"));

parts = [commandName, commandId, upper(string(varargin))];
commandText = strjoin(parts, ",");

fid = fopen(commandFile, "w");
if fid < 0
    error("Could not write command file: %s", commandFile);
end
cleanupObj = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, "%s\n", commandText);

fprintf("Sent robot command: %s\n", commandText);
end
