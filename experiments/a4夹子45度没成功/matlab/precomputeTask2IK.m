function precomputeTask2IK()
%% precomputeTask2IK.m
% Rebuild all Task 2 IK caches used by runTask2JoystickControl.

scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptDir);
cd(projectRoot);

fprintf("Rebuilding Task 2 joystick cache...\n");
precomputeTask2JoystickIK();

fprintf("\nRebuilding Task 2 transfer cache...\n");
precomputeTask2TransferIK();

fprintf("\nTask 2 IK precompute complete.\n");
end
