function editServoOffsets(targetOffsetDeg)
%% editServoOffsets.m
% Manually set servo zero offsets in both savedMotorOffset and motorOffset.
%
% Usage:
%   editServoOffsets
%   editServoOffsets([172.2 174.8 172.7 193.1 188.9 171.6 0 0])
%
% Offset direction:
%   q_FB = rawFeedback - offset
% Increasing offset makes the MATLAB-reported angle smaller.

scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptDir);
offsetFile = fullfile(projectRoot, "servo_zero_offsets.mat");

if exist(offsetFile, "file") ~= 2
    error("Offset file not found: %s", offsetFile);
end

data = load(offsetFile);
if isfield(data, "motorOffset")
    baseOffset = reshape(data.motorOffset, 1, []);
elseif isfield(data, "savedMotorOffset")
    baseOffset = reshape(data.savedMotorOffset, 1, []);
else
    error("Offset file must contain motorOffset or savedMotorOffset.");
end

if numel(baseOffset) < 8
    baseOffset = [baseOffset, zeros(1, 8 - numel(baseOffset))];
end

currentOffsetDeg = rad2deg(baseOffset);

fprintf("Offset file:\n%s\n", offsetFile);
fprintf("Current offset [deg]:\n");
disp(currentOffsetDeg);

if nargin < 1 || isempty(targetOffsetDeg)
    fprintf("Enter the final target offset in degrees as a vector.\n");
    fprintf("Current vector for copy/paste:\n");
    fprintf("[");
    fprintf("%.4f ", currentOffsetDeg(1:end-1));
    fprintf("%.4f]\n", currentOffsetDeg(end));
    targetOffsetDeg = input("Target offset [deg]: ");
end

targetOffsetDeg = reshape(targetOffsetDeg, 1, []);
if numel(targetOffsetDeg) < numel(baseOffset)
    targetOffsetDeg = [targetOffsetDeg, currentOffsetDeg(numel(targetOffsetDeg)+1:end)];
elseif numel(targetOffsetDeg) > numel(baseOffset)
    targetOffsetDeg = targetOffsetDeg(1:numel(baseOffset));
end

newOffset = deg2rad(targetOffsetDeg);

savedMotorOffset = newOffset;
motorOffset = newOffset;

fprintf("Delta [deg]:\n");
disp(targetOffsetDeg - currentOffsetDeg);
fprintf("New offset [deg]:\n");
disp(targetOffsetDeg);

reply = input("Type SAVE to update both savedMotorOffset and motorOffset: ", "s");
if ~strcmp(reply, "SAVE")
    fprintf("Cancelled. Offset file was not changed.\n");
    return;
end

save(offsetFile, "savedMotorOffset", "motorOffset", "-append");
fprintf("Saved both savedMotorOffset and motorOffset.\n");
end
