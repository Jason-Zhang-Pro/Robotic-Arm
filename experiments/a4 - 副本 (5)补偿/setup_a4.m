% setup_a4.m
% Run this once at the start of each MATLAB session for Assignment 4.

rootDir = fileparts(mfilename('fullpath'));

starterDir = fullfile(rootDir, 'external', 'UoM-RS-Starter_Code');
matlabLibDir = fullfile(rootDir, 'external', 'UoM_RS_MATLAB_LIBRARIES');
projectMatlabDir = fullfile(rootDir, 'matlab');

addpath(starterDir);
addpath(matlabLibDir);

if exist(projectMatlabDir, 'dir') == 7
    addpath(genpath(projectMatlabDir));
end

cleanupSerialConnections()

POSITION = 0;
VELOCITY = 1;
MAX_ID = 8;
ERROR = -1;
BAUDRATE = 115200;

save(fullfile(rootDir, 'UoM-RS-Var_Def.mat'), ...
    'POSITION', 'VELOCITY', 'MAX_ID', 'ERROR', 'BAUDRATE');

fprintf('Assignment 4 setup complete.\n');
fprintf('Starter code: %s\n', starterDir);
fprintf('MATLAB library: %s\n', matlabLibDir);
fprintf('Project MATLAB folder: %s\n', projectMatlabDir);
