
% readSerial.m
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



function [data, e] = readSerial(s, numElements, flush_flag)
    
    % Read data sent by Arduino over serial. This is done by checking 
    % how many characters are being sent and then reading the specified 
    % amount of chars.
    
    % External Variables
    % @ s                   - Serial object.
    % @ numElements         - Number of elements expected to be read.
    
    % Pre-allocate data vector
    data = zeros(1, numElements);
    
    % Set error flag to false
    e = false;
    
    
    for i=1:numElements
        
        % Check number of chars are to be sent
        dataTemp = read(s, 1, 'char');
        
        % Read appropriate number of chars
        if char(dataTemp) == '1'
            q_fb_tmp = read(s, 1, 'char');
        elseif char(dataTemp) == '2'
            q_fb_tmp = read(s, 2, 'char');
        elseif char(dataTemp) == '3'
            q_fb_tmp = read(s, 3, 'char');
        elseif char(dataTemp) == '4'
            q_fb_tmp = read(s, 4, 'char');
        elseif char(dataTemp) == '5'
            q_fb_tmp = read(s, 5, 'char');
        else
            % Report error if unexpected char identified and set flag
            "Error"
            char(dataTemp)
            e = true;
            break
        end
        
        % Combine chars into a string and convert into a double before
        % adding to the data vector.
        data(i) = str2double(strcat(q_fb_tmp(:)));
    
    end
    
    % Clear buffer at end of read
    Bytes = s.NumBytesAvailable;
    if Bytes > 0 && flush_flag
        flush(s);
    end

end