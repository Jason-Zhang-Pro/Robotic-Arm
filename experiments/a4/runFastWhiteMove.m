function runFastWhiteMove(fromSquare, toSquare)
%% runFastWhiteMove.m
% Fast entry point for one robot chess move after a webcam frame was already
% captured and the move was selected.

projectRoot = fileparts(mfilename("fullpath"));
addpath(genpath(fullfile(projectRoot, "matlab")));
runWebcamSelectedWhiteMove(fromSquare, toSquare, false);
end
