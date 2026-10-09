function imageFile = captureChessboardFrame()
%% captureChessboardFrame.m
% Capture one webcam frame of the chessboard for move recognition.

scriptDir = fileparts(mfilename('fullpath'));
outputDir = fullfile(scriptDir, "webcam_frames");
if exist(outputDir, "dir") ~= 7
    mkdir(outputDir);
end

timestamp = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
imageFile = fullfile(outputDir, "board_" + timestamp + ".png");
latestFile = fullfile(outputDir, "latest_board.png");

cameraNames = webcamlist;
if isempty(cameraNames)
    error("No webcam detected by MATLAB webcamlist.");
end

fprintf("Detected webcams:\n");
disp(cameraNames(:));

cam = webcam(1);
cleanupObj = onCleanup(@() clear("cam")); %#ok<NASGU>

pause(0.5);
frame = snapshot(cam);
imwrite(frame, imageFile);
imwrite(frame, latestFile);

fprintf("Captured chessboard frame:\n%s\n", imageFile);
fprintf("Latest frame copy:\n%s\n", latestFile);
end
