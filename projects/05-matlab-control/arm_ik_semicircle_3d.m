clear; clc;
load('UoM-RS-Var_Def.mat');

%% ================= Settings =================
COM_PORT = "COM3";
motor_ids = [1 2 3];                 % [base, shoulder, elbow]
dt = 0.04;                           % control period (s)

% 3-axis arm geometry (cm)
H  = 4.5;
L1 = 19.5;
L2 = 13.0;

% Home / baseline
use_fixed_home = true;
fixed_home_deg = [330.1172  123.6621  314.7363];
joint_sign = [1, 1, 1];
max_joint_delta_deg = 200;           % can later change to [a b c]

% Path settings -------------- 保持你的方法不动
arc_center = [18.37, 0, 1];       % [x, y, z] cm
arc_radius = 13;
num_arc_points = 100;
num_return_points = 100;

% % Command smoothing
alpha = 0.5;                        % smaller -> smoother
max_step_deg = 4;                    % max change per step

% Feedback / diagnosis
use_feedback_logging = true;         % read feedback every loop
feedback_pause = 0.005;              % small pause before reading feedback
print_debug = true;                  % print cmd/fb error info

DEG2RAD = pi / 180;
RAD2DEG = 180 / pi;

%% ================= Connect Robot =================
bot = robot(COM_PORT, MAX_ID, BAUDRATE);
bot = bot.ConnectToArduino();
bot.serial_obj.Timeout = 1;
bot = bot.getID();

mode = zeros(1, MAX_ID);
mode(motor_ids) = POSITION;
bot = bot.setDriveControlMode(mode);

%% ================= Get Home / Startup =================
if use_fixed_home
    home_deg = fixed_home_deg;
    fprintf('Using fixed home_deg: [%.2f, %.2f, %.2f]\n', home_deg);
else
    bot = bot.getFB();
    home_deg = bot.q_FB(motor_ids) * RAD2DEG;
    fprintf('Captured home_deg: [%.2f, %.2f, %.2f]\n', home_deg);
end

startup_servo_deg = home_deg;   % fallback
for k = 1:3
    try
        bot = bot.getFB();
        startup_servo_deg = bot.q_FB(motor_ids) * RAD2DEG;
        break;
    catch
        pause(0.05);
    end
end
fprintf('Recorded startup_servo_deg: [%.2f, %.2f, %.2f]\n', startup_servo_deg);

%% ================= Generate Trajectory =================
phi = linspace(-pi/2, pi/2, num_arc_points);

x_arc = arc_center(1) + arc_radius * cos(phi);
y_arc = arc_center(2) + arc_radius * sin(phi);
z_arc = arc_center(3) * ones(1, num_arc_points);

x_ret = x_arc(1) * ones(1, num_return_points);
y_ret = linspace(y_arc(end), y_arc(1), num_return_points);
z_ret = z_arc(1) * ones(1, num_return_points);

x_des = [x_arc, x_ret];
y_des = [y_arc, y_ret];
z_des = [z_arc, z_ret];
num_total = numel(x_des);

%% ================= Logs =================
t_data = zeros(num_total, 1);

cmd_deg_data = zeros(num_total, 3);
fb_deg_data  = NaN(num_total, 3);

cmd_xyz_data = zeros(num_total, 3);
fb_xyz_data  = NaN(num_total, 3);

err_deg_data = NaN(num_total, 3);
err_xyz_data = NaN(num_total, 3);

target_xyz_data = [x_des(:), y_des(:), z_des(:)];

%% ================= Move to Start Point =================
start_xyz = [x_des(1), y_des(1), z_des(1)];
servo_start_deg = xyz_to_servo_deg(start_xyz, H, L1, L2, ...
    home_deg, joint_sign, max_joint_delta_deg);

bot = send_servo_command(bot, motor_ids, servo_start_deg, MAX_ID, DEG2RAD);
pause(3.0);

%% ================= Main Control Loop =================
t0 = tic;
servo_deg_prev = servo_start_deg;

for i = 1:num_total
    target_xyz = [x_des(i), y_des(i), z_des(i)];

    % IK -> raw servo command
    servo_deg_cmd = xyz_to_servo_deg(target_xyz, H, L1, L2, ...
        home_deg, joint_sign, max_joint_delta_deg);

    % Smoothing + step limit
    if i == 1
        servo_deg_smooth = servo_deg_cmd;
    else
        servo_deg_smooth = alpha * servo_deg_cmd + (1 - alpha) * servo_deg_prev;

        delta_cmd = servo_deg_smooth - servo_deg_prev;
        delta_cmd = max(min(delta_cmd, max_step_deg), -max_step_deg);
        servo_deg_smooth = servo_deg_prev + delta_cmd;
    end

    servo_deg_prev = servo_deg_smooth;

    % Send command
    bot = send_servo_command(bot, motor_ids, servo_deg_smooth, MAX_ID, DEG2RAD);

    % FK for commanded pose
    q_model_cmd_deg = (servo_deg_smooth - home_deg) ./ joint_sign;
    ee_xyz_cmd = fk_from_model_deg(q_model_cmd_deg, H, L1, L2);

    % Read feedback
    fb_deg = [NaN NaN NaN];
    ee_xyz_fb = [NaN NaN NaN];

    if use_feedback_logging
        pause(feedback_pause);
        try
            bot = bot.getFB();
            fb_deg = bot.q_FB(motor_ids) * RAD2DEG;

            q_model_fb_deg = (fb_deg - home_deg) ./ joint_sign;
            ee_xyz_fb = fk_from_model_deg(q_model_fb_deg, H, L1, L2);
        catch
            % keep NaN if feedback read fails
        end
    end

    % Errors
    err_deg = fb_deg - servo_deg_smooth;
    err_xyz = ee_xyz_fb - target_xyz;

    % Log data
    t_data(i) = toc(t0);

    cmd_deg_data(i, :) = servo_deg_smooth;
    fb_deg_data(i, :) = fb_deg;

    cmd_xyz_data(i, :) = ee_xyz_cmd;
    fb_xyz_data(i, :) = ee_xyz_fb;

    err_deg_data(i, :) = err_deg;
    err_xyz_data(i, :) = err_xyz;

    if print_debug
        fprintf(['Step %3d | target_xyz = [%6.2f %6.2f %6.2f] | ' ...
                 'cmd_deg = [%7.2f %7.2f %7.2f] | ' ...
                 'fb_deg = [%7.2f %7.2f %7.2f] | ' ...
                 'err_deg = [%6.2f %6.2f %6.2f]\n'], ...
                 i, ...
                 target_xyz(1), target_xyz(2), target_xyz(3), ...
                 servo_deg_smooth(1), servo_deg_smooth(2), servo_deg_smooth(3), ...
                 fb_deg(1), fb_deg(2), fb_deg(3), ...
                 err_deg(1), err_deg(2), err_deg(3));
    end

    % Timing control
    t_next = i * dt;
    t_now = toc(t0);
    if t_next > t_now
        pause(t_next - t_now);
    end
end

%% ================= Return to Startup =================
bot = send_servo_command(bot, motor_ids, startup_servo_deg, MAX_ID, DEG2RAD);
pause(1.0);

%% ================= Summary =================
valid_deg = ~isnan(err_deg_data(:,1));
valid_xyz = ~isnan(err_xyz_data(:,1));

if any(valid_deg)
    mean_abs_joint_err = mean(abs(err_deg_data(valid_deg,:)), 1);
    max_abs_joint_err  = max(abs(err_deg_data(valid_deg,:)), [], 1);

    fprintf('\n===== Joint Error Summary (deg) =====\n');
    fprintf('Mean abs error: [%.2f, %.2f, %.2f]\n', mean_abs_joint_err);
    fprintf('Max  abs error: [%.2f, %.2f, %.2f]\n', max_abs_joint_err);
end

if any(valid_xyz)
    mean_abs_xyz_err = mean(abs(err_xyz_data(valid_xyz,:)), 1);
    max_abs_xyz_err  = max(abs(err_xyz_data(valid_xyz,:)), [], 1);

    fprintf('\n===== End-Effector Error Summary (cm) =====\n');
    fprintf('Mean abs error: [%.2f, %.2f, %.2f]\n', mean_abs_xyz_err);
    fprintf('Max  abs error: [%.2f, %.2f, %.2f]\n', max_abs_xyz_err);
end

%% ================= Save CSV =================
T = table( ...
    t_data, ...
    target_xyz_data(:,1), target_xyz_data(:,2), target_xyz_data(:,3), ...
    cmd_deg_data(:,1), cmd_deg_data(:,2), cmd_deg_data(:,3), ...
    fb_deg_data(:,1), fb_deg_data(:,2), fb_deg_data(:,3), ...
    cmd_xyz_data(:,1), cmd_xyz_data(:,2), cmd_xyz_data(:,3), ...
    fb_xyz_data(:,1), fb_xyz_data(:,2), fb_xyz_data(:,3), ...
    err_deg_data(:,1), err_deg_data(:,2), err_deg_data(:,3), ...
    err_xyz_data(:,1), err_xyz_data(:,2), err_xyz_data(:,3), ...
    'VariableNames', { ...
    'time_s', ...
    'target_x_cm','target_y_cm','target_z_cm', ...
    'cmd_id1_deg','cmd_id2_deg','cmd_id3_deg', ...
    'fb_id1_deg','fb_id2_deg','fb_id3_deg', ...
    'cmd_x_cm','cmd_y_cm','cmd_z_cm', ...
    'fb_x_cm','fb_y_cm','fb_z_cm', ...
    'err_id1_deg','err_id2_deg','err_id3_deg', ...
    'err_x_cm','err_y_cm','err_z_cm'});

csv_file = 'ik_semicircle_feedback_debug.csv';
writetable(T, csv_file);
fprintf('\nSaved diagnostic CSV: %s\n', csv_file);

%% ================= Simple Plots =================
figure;
plot(t_data, cmd_deg_data(:,1), 'LineWidth', 1.2); hold on;
plot(t_data, fb_deg_data(:,1), '--', 'LineWidth', 1.0);
plot(t_data, cmd_deg_data(:,2), 'LineWidth', 1.2);
plot(t_data, fb_deg_data(:,2), '--', 'LineWidth', 1.0);
plot(t_data, cmd_deg_data(:,3), 'LineWidth', 1.2);
plot(t_data, fb_deg_data(:,3), '--', 'LineWidth', 1.0);
xlabel('Time (s)');
ylabel('Joint angle (deg)');
title('Commanded vs Feedback Joint Angles');
legend('cmd q1','fb q1','cmd q2','fb q2','cmd q3','fb q3');
grid on;

figure;
plot3(target_xyz_data(:,1), target_xyz_data(:,2), target_xyz_data(:,3), 'LineWidth', 1.5); hold on;
plot3(fb_xyz_data(:,1), fb_xyz_data(:,2), fb_xyz_data(:,3), '--', 'LineWidth', 1.2);
xlabel('X (cm)');
ylabel('Y (cm)');
zlabel('Z (cm)');
title('Target Trajectory vs Feedback FK Trajectory');
legend('target xyz','feedback xyz');
grid on;
axis equal;

%% ================= Local Functions =================
function servo_deg = xyz_to_servo_deg(target_xyz, H, L1, L2, home_deg, joint_sign, max_joint_delta_deg)
    x = target_xyz(1);
    y = target_xyz(2);
    z = target_xyz(3);

    q1 = atan2(y, x);
    r = hypot(x, y);
    z_local = z - H;

    D = (r^2 + z_local^2 - L1^2 - L2^2) / (2 * L1 * L2);
    D = min(max(D, -1), 1);

    % elbow-down IK
    q3 = atan2(-sqrt(max(0, 1 - D^2)), D);
    q2 = atan2(z_local, r) - atan2(L2 * sin(q3), L1 + L2 * cos(q3));

    q_model_deg = [q1, q2, q3] * 180 / pi;
    q_model_deg(2) = q_model_deg(2) - 90;

    servo_raw = home_deg + joint_sign .* q_model_deg;

    if numel(max_joint_delta_deg) == 1
        low_lim = home_deg - max_joint_delta_deg;
        high_lim = home_deg + max_joint_delta_deg;
    else
        low_lim = home_deg - max_joint_delta_deg(:).';
        high_lim = home_deg + max_joint_delta_deg(:).';
    end

    servo_deg = min(max(servo_raw, low_lim), high_lim);
end

function bot = send_servo_command(bot, motor_ids, servo_deg, MAX_ID, DEG2RAD)
    q_cmd = zeros(1, MAX_ID);
    q_cmd(motor_ids) = servo_deg * DEG2RAD;
    bot = bot.setQ(q_cmd);
    bot = bot.driveMotors();
end

function ee_xyz = fk_from_model_deg(q_model_deg, H, L1, L2)
    q1 = q_model_deg(1) * pi / 180;
    q2 = (90 + q_model_deg(2)) * pi / 180;
    q3 = q_model_deg(3) * pi / 180;

    x = cos(q1) * (L1 * cos(q2) + L2 * cos(q2 + q3));
    y = sin(q1) * (L1 * cos(q2) + L2 * cos(q2 + q3));
    z = H + L1 * sin(q2) + L2 * sin(q2 + q3);

    ee_xyz = [x, y, z];
end