
% readFB.m
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


function setDigitalIO(s, state)

    % Read motor position by sending a position read command to the
    % Arduino and then calling the readSerial() function to receive
    % the data.
    
    % External Variables
    % @ s                   - Serial object.
    % @ state           - State of the pin to be set.
    
    % Internal Functions
    % @ readSerial()        - Read data sent by Arduino over serial.
    
    % Send state command to Arduino

    if state == 1
        writeline(s,'DON');
        writeline(s,'DON');
    else
        writeline(s,'DOF');
        writeline(s,'DOF');
    end
    
    

end