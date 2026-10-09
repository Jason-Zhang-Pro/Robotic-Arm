
% robot.m
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


classdef robot < handle
    %robot Class for connecting to & controlling robot motors
    %   This class contains the variables and methods required to interface
    %   with the Arduino attached to both FeeTech STS3215 & STS3045M
    %   motors.
    
    properties
        control_mode            % Indicates if motors are in pos or vel mode
        q_ref                   % Joint position reference
        q_dot_ref               % Joint velocity reference
        q_FB                    % Motor feedback
        ID                      % Array of all possible IDs up to MAX_ID
        max_id                  % Local copy of MAX_ID, the maxium number of connected motors
        connected_ID            % Array of IDs of connected motors
        num_ID                  % Number of connected motors
        serial_obj              % SerialPort object for communicating with Arduino
        serial_port             % The port identifier of the Arduino
        serial_baudrate         % The baudrate for communicating with the Arduino
        offset                  % Offset to calibrate home position
        joystick                % Joystick Input
        joy_offset              % Joystick centre offset
        
    end

    methods
        function obj = robot(port,MAX_ID,BAUDRATE)
            %Robot Construct an instance of this class
            %   This function initialises all variables

            obj.max_id = MAX_ID;
            obj.control_mode = zeros(1,obj.max_id, 'logical');
            obj.q_ref = zeros(1,obj.max_id, 'double');
            obj.q_dot_ref = zeros(1,obj.max_id, 'double');
            obj.q_FB = zeros(1,obj.max_id, 'double');
            obj.serial_port = port;
            obj.serial_baudrate = BAUDRATE;
            obj.offset = zeros(1,obj.max_id, 'double');
            obj.joystick = [0 0 0];
            obj.joy_offset = [0 0];

            for i=1:obj.max_id
                obj.ID(i) = i;
            end


        end


        function obj = ConnectToArduino(obj)
            %ConnectToArduino Open serial port & handshake with Arduino
            obj.serial_obj = establishSerial(obj.serial_port, obj.serial_baudrate);

        end
        
        function obj = getID(obj)
            % getID Request IDs of connected motors from Arduino
            [obj.num_ID, obj.connected_ID] = getMotorIDs(obj.serial_obj);

        end
        
        function obj = getFB(obj)
            % getFB Request motor feedback from Arduino
            obj.q_FB = readFB(obj.serial_obj, obj.max_id) - obj.offset;

        end

        function obj = getJOY(obj)
            % getJOY Request joystick input from Arduino
            obj.joystick = readJOY(obj.serial_obj);

            % apply offset
            obj.joystick(1:2) = obj.joystick(1:2) - obj.joy_offset;

            
            for i = 1:2 
                
                % Read Sign
                tmp_sign = sign(obj.joystick(i));

                % normalise
                obj.joystick(i) = obj.joystick(i)/obj.joy_offset(i);
                
                % apply non-linear scaling
                obj.joystick(i) = tmp_sign * (obj.joystick(i)^2);
    
            end

            

        end
        
        function obj = setJoyOffset(obj)
                
            tmp_jopy_os = readJOY(obj.serial_obj);
            % getJOY Request joystick input from Arduino
            obj.joy_offset(1) = tmp_jopy_os(1);
            obj.joy_offset(2) = tmp_jopy_os(2);

        end

        function obj = setDigitalPin(obj, pin_state)
            % getFB Request motor feedback from Arduino
            setDigitalIO(obj.serial_obj, pin_state)

        end

        function obj = setOffset(obj, motor_offset)
            % getFB Request motor feedback from Arduino
            for i=1:length(motor_offset)
                obj.offset(i) = motor_offset(i);
            end

        end
        
        function obj = driveMotors(obj)
            % driveMotors Send reference pos/vel to Arduino
            
            % Merge position & velocity values into one array for efficient
            % communication based on control mode.
            value = zeros(1,obj.max_id, 'double');
            for i=1:length(obj.control_mode)
                if obj.control_mode(i) == 0
                    value(i) = obj.q_ref(i);
                else
                    value(i) = obj.q_dot_ref(i);
                end

            end

            % Send array to Arduino
            sendMotorCommand(obj.serial_obj, value, obj.max_id)

        end

        function obj = setQ(obj, q)
            % setQ Set the internal reference position
             for i = 1:length(q)
                 obj.q_ref(i) = q(i) + obj.offset(i);
             end
        end

        function obj = setQDot(obj, q_dot)
            % setQDot Set the internal reference velocity
            for i = 1:length(q_dot)
                obj.q_dot_ref(i) = q_dot(i);
            end
        end

        function obj = setDriveControlMode(obj, new_drive_control_mode)
            % setDriveControlMode Set the new control modes and send to
            % Arduino
            for i = 1:length(new_drive_control_mode)
                obj.control_mode(i) = new_drive_control_mode(i);
            end
            updateDriveMode(obj.serial_obj, obj.control_mode, obj.max_id);
        end
        

    end
end