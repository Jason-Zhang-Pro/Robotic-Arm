
% sendJointPos.m
% --------------------------
% Licenting Information: You are free to use or extend this project
% for educational purposes provided that (1) you do not distribute or
% publish solutions, (2) you retain this notice, and (3) you provide 
% clear attribution to the University of Melbourne, Department of 
% Mechanical Engineering and Product Design Lab.
% 
% Attribution Information: The ChessBot project was developed in collaboration
% between Product Design Lab & the University of Melbourne. The core project was 
% primarily developed by Professor Denny Oetomo (doetomo@unimelb.edu.au). All 
% Starter Code and libraries were developed by Nathan Batham of Product
% Design Lab
% (nathan@productdesignlab.com.au)


function updateDriveMode(s, value, numMotors)
    
    % Send updated control modes over serial. This is done by
    % first converting the int to a string and then adding a
    % terminating non-numeric string character.
    
    % External Variables
    % @ s                   - Serial object.
    % @ jointVel            - Vector of joint velocites to be sent
    %                   to each motor in rad/s.
    
    
    % Send drive motor command
    writeline(s,'UDM');
    writeline(s,'UDM');

    % Send joint velocities individually
    for i=1:numMotors
        writeline(s, num2str(value(i), '%d') + "e");
    end
    
    % wait for arduino ack
    while ( read(s, 1, 'char') ~= 'd' ) 
    end

end