
% readFB.m
% --------------------------
% Licenting Information: You are free to use or extend this project
% for educational purposes provided that (1) you do not distribute or
% publish solutions, (2) you retain this notice, and (3) you provide 
% clear attribution to the University of Melbourne, Department of 
% Mechanical Engineering.
% 
% Attribution Information: The ChessBot project was developed at the
% University of Melbourne. The core project was primarily developed
% by Professor Denny Oetomo (doetomo@unimelb.edu.au). The ChessBot 
% Skeleton Code was developed by Nathan Batham 
% (nathan.batham@unimelb.edu.au)


function [joy_input, e] = readJOY(s)

    % Read motor position by sending a position read command to the
    % Arduino and then calling the readSerial() function to receive
    % the data.
    
    % External Variables
    % @ s                   - Serial object.
    % @ numMotors           - Number of motors expected to read from.
    
    % Internal Functions
    % @ readSerial()        - Read data sent by Arduino over serial.
    
    % Send position read command to Arduino
    writeline(s,'JOY');
%     writeline(s,'FBK');
        
    % Read motor feedback in SCS format
    [joy_input, e] = readSerial(s, 3);

    if e
        writeline(s,'JOY');
        [joy_input, e] = readSerial(s, 3);
        if e ~= ERROR
            "Error Handled"
        end
    end
    

end