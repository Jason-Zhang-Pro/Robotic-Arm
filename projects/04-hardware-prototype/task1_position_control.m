clear; clc;
load('UoM-RS-Var_Def.mat');

%% ================= Settings =================
COM_PORT = "COM3";
motor_ids = 1:6;                     % all motors in order
arm_motor_ids = [1 2 3];             % [base, shoulder, elbow]
wrist_hold_ids = [4 5];              % held at startup angle for now
gripper_id = 6;
dt = 0.10;                           % control period (s), larger is slower/safer

% Run only the first Task 1 move while tuning.
% Increase this after e2 -> e4 is reliable.
moves_to_run = 1;
pre_motion_pause = 5.0;              % time to get ready before motors start moving

% Safety calibration mode. Enable only when testing one motor direction.
direction_test_mode = false;
direction_test_motor_id = 1;         % test one motor at a time: 1, then 2, then 3
direction_test_delta_deg = -3;       % small command test; negative command should move toward -X board

% Trial fix: mirror the target points before IK.
mirror_target_x_for_control = true;

% Direct real-arm joint trial. This bypasses chessboard IK completely.
manual_joint_trial_mode = true;
manual_joint_delta_deg = [0, 5, 0];   % [motor1, motor2, motor3] relative to startup zero
manual_return_to_zero = true;

% Optional gripper command. Keep disabled until open/close angles are tuned.
use_gripper = false;
gripper_open_deg = 0;
gripper_closed_deg = 35;

% URDF-derived 3-axis geometry (cm)
H = 5.0;                             % shoulder_lift z offset
base_forward_x_sign = 1;             % base zero direction faces global +X; board is on global -X
shoulder_x_offset = 2.5;             % shoulder offset in the base-forward direction
L1 = 19.0;                           % upper_arm_link
lower_arm_length = 13.0;
wrist_length = 5.5;
grasp_offset_from_wrist_end = 11.0;  % gripper_center_link is 1 cm behind 12 cm jaw tip
L2 = lower_arm_length + wrist_length + grasp_offset_from_wrist_end;

% Chessboard geometry from the real setup (cm)
square_size = 3.0;
board_thickness = 0.3;
board_near_edge_distance = 13.5;
board_size = 8 * square_size;
board_center = [-(board_near_edge_distance + board_size/2), 0, 0];

piece_height = 4.5;
grasp_depth_from_top = 1.0;
grasp_z = board_thickness + piece_height - grasp_depth_from_top;
safe_z = 17.5;
home_xyz = [base_forward_x_sign * shoulder_x_offset, 0.0, H + L1 + L2];
captured_pool_start = [-(board_near_edge_distance - 5.5), -10.5, board_thickness];
captured_pool_spacing = 2.4;

% Home / baseline
% Default: read the current 1-6 motor angles once and treat that pose as zero.
use_fixed_home = false;
fixed_home_deg = [330.1172  123.6621  314.7363];  % fallback only if use_fixed_home = true
% setQ command direction is opposite to the observed feedback direction on the real arm.
joint_sign = [1, -1, 1];
max_joint_delta_deg = 200;           % can later change to [a b c]

% Command smoothing
alpha = 0.25;                        % smaller -> smoother
max_joint_speed_deg_s = 1.5;         % safety speed limit for motors 1-3
max_step_deg = max_joint_speed_deg_s * dt;
points_per_segment = 120;
fprintf('Safety speed limit: %.2f deg/s, %.2f deg/command, dt = %.2f s\n', ...
    max_joint_speed_deg_s, max_step_deg, dt);

% Feedback / diagnosis
use_feedback_logging = true;
feedback_pause = 0.005;
print_debug = true;

DEG2RAD = pi / 180;
RAD2DEG = 180 / pi;

%% ================= Task 1 Move List =================
task_moves = {
    'e2', 'e4'
    'g1', 'f3'
    'f1', 'b5'
    'b1', 'c3'
    'e1', 'g1'
    'h1', 'f1'
    'd5', 'captured_pool'
    'e4', 'd5'
    'd5', 'captured_pool'
    'c3', 'd5'
    'c6', 'captured_pool'
    'b5', 'c6'
    'c2', 'c3'
    'f3', 'g5'
    'd2', 'd4'
    'd1', 'f3'
    'd5', 'captured_pool'
    'f3', 'd5'
    'c1', 'captured_pool'
    'a1', 'c1'
    'g1', 'h1'
    'f2', 'f4'
    'g5', 'h3'
    'g2', 'g3'
    'c1', 'd1'
    'f1', 'e1'
};
moves_to_run = min(moves_to_run, size(task_moves, 1));

%% ================= Connect Robot =================
bot = robot(COM_PORT, MAX_ID, BAUDRATE);
bot = bot.ConnectToArduino();
bot.serial_obj.Timeout = 1;
bot = bot.getID();

mode = zeros(1, MAX_ID);
mode(motor_ids) = POSITION;
bot = bot.setDriveControlMode(mode);

%% ================= Read Zero / Startup =================
startup_servo_deg = zeros(1, numel(motor_ids));
startup_read_ok = false;
for k = 1:3
    try
        bot = bot.getFB();
        startup_servo_deg = bot.q_FB(motor_ids) * RAD2DEG;
        startup_read_ok = true;
        break;
    catch
        pause(0.05);
    end
end

if ~startup_read_ok
    error('Failed to read startup motor feedback. Keep the robot at zero position and try again.');
end

if use_fixed_home
    home_deg = fixed_home_deg;
    fprintf('Using fixed home_deg: [%.2f, %.2f, %.2f]\n', home_deg);
else
    home_deg = startup_servo_deg(arm_motor_ids);
    fprintf('Captured current arm angles as zero/home_deg: [%.2f, %.2f, %.2f]\n', home_deg);
end

hold_servo_deg = startup_servo_deg;
hold_servo_deg(arm_motor_ids) = home_deg;

fprintf('Recorded startup zero for all motors deg: [%s]\n', sprintf('%.2f ', startup_servo_deg));
fprintf('Holding motors 4-5 at startup deg: [%.2f, %.2f]\n', hold_servo_deg(wrist_hold_ids));

if direction_test_mode
    if ~ismember(direction_test_motor_id, arm_motor_ids)
        error('direction_test_motor_id must be one of motors 1, 2, or 3.');
    end

    test_servo_deg = startup_servo_deg;
    test_servo_deg(direction_test_motor_id) = startup_servo_deg(direction_test_motor_id) + direction_test_delta_deg;

    fprintf('\n===== Direction Test Mode =====\n');
    fprintf('Only motor %d will move by %+0.2f deg, then return to zero.\n', ...
        direction_test_motor_id, direction_test_delta_deg);
    fprintf('Watch whether this commanded motion moves toward the chessboard.\n');
    fprintf('Motion starts in %.1f seconds. Press Ctrl+C now if not ready.\n', pre_motion_pause);
    pause(pre_motion_pause);

    bot = ramp_all_servo_command(bot, motor_ids, startup_servo_deg, test_servo_deg, ...
        MAX_ID, DEG2RAD, max_step_deg, dt);
    pause(1.0);
    bot = ramp_all_servo_command(bot, motor_ids, test_servo_deg, startup_servo_deg, ...
        MAX_ID, DEG2RAD, max_step_deg, dt);

    fprintf('Direction test complete. Set direction_test_mode = false after directions are calibrated.\n');
    return;
end

if manual_joint_trial_mode
    trial_servo_deg = startup_servo_deg;
    trial_servo_deg(arm_motor_ids) = startup_servo_deg(arm_motor_ids) + manual_joint_delta_deg;

    fprintf('\n===== Manual Joint Trial Mode =====\n');
    fprintf('Bypassing IK. Motor 1-3 delta command: [%.2f, %.2f, %.2f] deg.\n', manual_joint_delta_deg);
    fprintf('Use this to find the real direction that moves from all-up zero toward the board.\n');
    fprintf('Motion starts in %.1f seconds. Press Ctrl+C now if not ready.\n', pre_motion_pause);
    pause(pre_motion_pause);

    bot = ramp_all_servo_command(bot, motor_ids, startup_servo_deg, trial_servo_deg, ...
        MAX_ID, DEG2RAD, max_step_deg, dt);

    if manual_return_to_zero
        pause(2.0);
        bot = ramp_all_servo_command(bot, motor_ids, trial_servo_deg, startup_servo_deg, ...
            MAX_ID, DEG2RAD, max_step_deg, dt);
    end

    fprintf('Manual joint trial complete. Change manual_joint_delta_deg and rerun until the direction is correct.\n');
    return;
end

%% ================= Generate Task 1 Trajectory =================
[target_xyz_data, action_data] = build_task1_xyz_path( ...
    task_moves, moves_to_run, board_center, square_size, board_thickness, ...
    grasp_z, safe_z, home_xyz, captured_pool_start, captured_pool_spacing, ...
    points_per_segment);

if mirror_target_x_for_control
    target_xyz_data(:, 1) = 2 * home_xyz(1) - target_xyz_data(:, 1);
    fprintf('Mirror trial enabled: control target X is mirrored around home X = %.2f cm before IK.\n', home_xyz(1));
end

num_total = size(target_xyz_data, 1);

first_pick_xyz = square_to_xyz(task_moves{1, 1}, board_center, square_size, board_thickness);
first_above_xyz = [first_pick_xyz(1), first_pick_xyz(2), safe_z];
first_grasp_xyz = [first_pick_xyz(1), first_pick_xyz(2), grasp_z];
if mirror_target_x_for_control
    fprintf('Original e2 X %.2f cm -> mirrored control X %.2f cm.\n', ...
        first_pick_xyz(1), 2 * home_xyz(1) - first_pick_xyz(1));
    first_above_xyz(1) = 2 * home_xyz(1) - first_above_xyz(1);
    first_grasp_xyz(1) = 2 * home_xyz(1) - first_grasp_xyz(1);
end
first_above_servo = xyz_to_servo_deg(first_above_xyz, H, base_forward_x_sign, shoulder_x_offset, L1, L2, ...
    home_deg, joint_sign, max_joint_delta_deg);
first_grasp_servo = xyz_to_servo_deg(first_grasp_xyz, H, base_forward_x_sign, shoulder_x_offset, L1, L2, ...
    home_deg, joint_sign, max_joint_delta_deg);
fprintf('First move e2 above delta deg from zero: [%.2f, %.2f, %.2f]\n', first_above_servo - home_deg);
fprintf('First move e2 grasp delta deg from zero: [%.2f, %.2f, %.2f]\n', first_grasp_servo - home_deg);
fprintf('Motion starts in %.1f seconds. Press Ctrl+C now if the direction looks wrong.\n', pre_motion_pause);
pause(pre_motion_pause);

%% ================= Logs =================
t_data = zeros(num_total, 1);
cmd_deg_data = zeros(num_total, 3);
fb_deg_data  = NaN(num_total, 3);
cmd_xyz_data = zeros(num_total, 3);
fb_xyz_data  = NaN(num_total, 3);
err_deg_data = NaN(num_total, 3);
err_xyz_data = NaN(num_total, 3);

%% ================= Move to Start Point =================
start_xyz = target_xyz_data(1, :);
servo_start_deg = xyz_to_servo_deg(start_xyz, H, base_forward_x_sign, shoulder_x_offset, L1, L2, ...
    home_deg, joint_sign, max_joint_delta_deg);

bot = send_servo_command(bot, motor_ids, hold_servo_deg, arm_motor_ids, servo_start_deg, MAX_ID, DEG2RAD);
pause(2.0);

if use_gripper
    hold_servo_deg(gripper_id) = gripper_open_deg;
    bot = send_servo_command(bot, motor_ids, hold_servo_deg, arm_motor_ids, servo_start_deg, MAX_ID, DEG2RAD);
    pause(0.4);
end

%% ================= Main Control Loop =================
t0 = tic;
servo_deg_prev = servo_start_deg;

for i = 1:num_total
    target_xyz = target_xyz_data(i, :);

    if use_gripper
        if strcmp(action_data(i), "close_gripper")
            hold_servo_deg(gripper_id) = gripper_closed_deg;
            bot = send_servo_command(bot, motor_ids, hold_servo_deg, arm_motor_ids, servo_deg_prev, MAX_ID, DEG2RAD);
            pause(0.4);
        elseif strcmp(action_data(i), "open_gripper")
            hold_servo_deg(gripper_id) = gripper_open_deg;
            bot = send_servo_command(bot, motor_ids, hold_servo_deg, arm_motor_ids, servo_deg_prev, MAX_ID, DEG2RAD);
            pause(0.4);
        end
    end

    servo_deg_cmd = xyz_to_servo_deg(target_xyz, H, base_forward_x_sign, shoulder_x_offset, L1, L2, ...
        home_deg, joint_sign, max_joint_delta_deg);

    if i == 1
        servo_deg_smooth = servo_deg_cmd;
    else
        servo_deg_smooth = alpha * servo_deg_cmd + (1 - alpha) * servo_deg_prev;
        delta_cmd = servo_deg_smooth - servo_deg_prev;
        delta_cmd = max(min(delta_cmd, max_step_deg), -max_step_deg);
        servo_deg_smooth = servo_deg_prev + delta_cmd;
    end
    servo_deg_prev = servo_deg_smooth;

    bot = send_servo_command(bot, motor_ids, hold_servo_deg, arm_motor_ids, servo_deg_smooth, MAX_ID, DEG2RAD);

    q_model_cmd_deg = (servo_deg_smooth - home_deg) ./ joint_sign;
    ee_xyz_cmd = fk_from_model_deg(q_model_cmd_deg, H, base_forward_x_sign, shoulder_x_offset, L1, L2);

    fb_deg = [NaN NaN NaN];
    ee_xyz_fb = [NaN NaN NaN];
    if use_feedback_logging
        pause(feedback_pause);
        try
            bot = bot.getFB();
            fb_deg = bot.q_FB(arm_motor_ids) * RAD2DEG;
            q_model_fb_deg = (fb_deg - home_deg) ./ joint_sign;
            ee_xyz_fb = fk_from_model_deg(q_model_fb_deg, H, base_forward_x_sign, shoulder_x_offset, L1, L2);
        catch
        end
    end

    err_deg = fb_deg - servo_deg_smooth;
    err_xyz = ee_xyz_fb - target_xyz;

    t_data(i) = toc(t0);
    cmd_deg_data(i, :) = servo_deg_smooth;
    fb_deg_data(i, :) = fb_deg;
    cmd_xyz_data(i, :) = ee_xyz_cmd;
    fb_xyz_data(i, :) = ee_xyz_fb;
    err_deg_data(i, :) = err_deg;
    err_xyz_data(i, :) = err_xyz;

    if print_debug
        fprintf(['Step %3d/%3d | %-13s | target_xyz = [%6.2f %6.2f %6.2f] | ' ...
                 'cmd_deg = [%7.2f %7.2f %7.2f] | fb_deg = [%7.2f %7.2f %7.2f]\n'], ...
                 i, num_total, action_data(i), ...
                 target_xyz(1), target_xyz(2), target_xyz(3), ...
                 servo_deg_smooth(1), servo_deg_smooth(2), servo_deg_smooth(3), ...
                 fb_deg(1), fb_deg(2), fb_deg(3));
    end

    t_next = i * dt;
    t_now = toc(t0);
    if t_next > t_now
        pause(t_next - t_now);
    end
end

%% ================= Return to Startup =================
bot = send_all_servo_command(bot, motor_ids, startup_servo_deg, MAX_ID, DEG2RAD);
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
    t_data, action_data, ...
    target_xyz_data(:,1), target_xyz_data(:,2), target_xyz_data(:,3), ...
    cmd_deg_data(:,1), cmd_deg_data(:,2), cmd_deg_data(:,3), ...
    fb_deg_data(:,1), fb_deg_data(:,2), fb_deg_data(:,3), ...
    cmd_xyz_data(:,1), cmd_xyz_data(:,2), cmd_xyz_data(:,3), ...
    fb_xyz_data(:,1), fb_xyz_data(:,2), fb_xyz_data(:,3), ...
    err_deg_data(:,1), err_deg_data(:,2), err_deg_data(:,3), ...
    err_xyz_data(:,1), err_xyz_data(:,2), err_xyz_data(:,3), ...
    'VariableNames', { ...
    'time_s','action', ...
    'target_x_cm','target_y_cm','target_z_cm', ...
    'cmd_id1_deg','cmd_id2_deg','cmd_id3_deg', ...
    'fb_id1_deg','fb_id2_deg','fb_id3_deg', ...
    'cmd_x_cm','cmd_y_cm','cmd_z_cm', ...
    'fb_x_cm','fb_y_cm','fb_z_cm', ...
    'err_id1_deg','err_id2_deg','err_id3_deg', ...
    'err_x_cm','err_y_cm','err_z_cm'});

csv_file = 'task1_position_control_debug.csv';
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
title('Task 1 Position Control: Commanded vs Feedback Joint Angles');
legend('cmd q1','fb q1','cmd q2','fb q2','cmd q3','fb q3');
grid on;

figure;
plot3(target_xyz_data(:,1), target_xyz_data(:,2), target_xyz_data(:,3), 'LineWidth', 1.5); hold on;
plot3(fb_xyz_data(:,1), fb_xyz_data(:,2), fb_xyz_data(:,3), '--', 'LineWidth', 1.2);
xlabel('X (cm)');
ylabel('Y (cm)');
zlabel('Z (cm)');
title('Task 1 Target Trajectory vs Feedback FK Trajectory');
legend('target xyz','feedback xyz');
grid on;
axis equal;

%% ================= Local Functions =================
function [xyz_path, actions] = build_task1_xyz_path(task_moves, moves_to_run, board_center, square_size, board_z, ...
        grasp_z, safe_z, home_xyz, captured_pool_start, captured_pool_spacing, points_per_segment)
    xyz_path = [];
    actions = strings(0, 1);
    capture_count = 0;

    append_point(home_xyz, "home");
    for move_idx = 1:moves_to_run
        from_square = task_moves{move_idx, 1};
        to_square = task_moves{move_idx, 2};
        from_xyz = square_to_xyz(from_square, board_center, square_size, board_z);

        if strcmp(to_square, 'captured_pool')
            capture_count = capture_count + 1;
            to_xyz = captured_pool_xyz(captured_pool_start, captured_pool_spacing, capture_count);
        else
            to_xyz = square_to_xyz(to_square, board_center, square_size, board_z);
        end

        waypoints = [
            home_xyz
            from_xyz(1), from_xyz(2), safe_z
            from_xyz(1), from_xyz(2), grasp_z
            from_xyz(1), from_xyz(2), safe_z
            to_xyz(1),   to_xyz(2),   safe_z
            to_xyz(1),   to_xyz(2),   grasp_z
            to_xyz(1),   to_xyz(2),   safe_z
            home_xyz
        ];
        labels = ["home"; "above_pick"; "close_gripper"; "lift_piece"; ...
                  "above_place"; "open_gripper"; "clear_place"; "home"];

        for k = 1:size(waypoints, 1)-1
            segment = interpolate_xyz(waypoints(k, :), waypoints(k+1, :), points_per_segment);
            if ~isempty(xyz_path)
                segment = segment(2:end, :);
            end
            segment_actions = repmat("move", size(segment, 1), 1);
            segment_actions(end) = labels(k+1);
            xyz_path = [xyz_path; segment]; %#ok<AGROW>
            actions = [actions; segment_actions]; %#ok<AGROW>
        end
    end

    function append_point(xyz, label)
        xyz_path = [xyz_path; xyz];
        actions = [actions; label];
    end
end

function segment = interpolate_xyz(p0, p1, n)
    s = linspace(0, 1, n).';
    s = 0.5 - 0.5 * cos(pi * s);
    segment = (1 - s) .* p0 + s .* p1;
end

function xyz = square_to_xyz(square, board_center, square_size, board_z)
    file_idx = double(lower(square(1))) - double('a');
    rank_idx = str2double(square(2)) - 1;
    board_size = 8 * square_size;
    x0 = board_center(1) - board_size / 2;
    y0 = board_center(2) - board_size / 2;
    xyz = [x0 + (file_idx + 0.5) * square_size, ...
           y0 + (rank_idx + 0.5) * square_size, ...
           board_z];
end

function xyz = captured_pool_xyz(start_xyz, spacing, count)
    col = mod(count - 1, 3);
    row = floor((count - 1) / 3);
    xyz = start_xyz + [col * spacing, row * spacing, 0];
end

function servo_deg = xyz_to_servo_deg(target_xyz, H, base_forward_x_sign, shoulder_x_offset, L1, L2, home_deg, joint_sign, max_joint_delta_deg)
    x = target_xyz(1);
    y = target_xyz(2);
    z = target_xyz(3);

    x_forward = base_forward_x_sign * x;
    q1 = atan2(y, x_forward);
    r_base = hypot(x_forward, y);
    r = r_base - shoulder_x_offset;
    z_local = z - H;

    D = (r^2 + z_local^2 - L1^2 - L2^2) / (2 * L1 * L2);
    D = min(max(D, -1), 1);

    q3 = atan2(-sqrt(max(0, 1 - D^2)), D);
    q2 = atan2(z_local, r) - atan2(L2 * sin(q3), L1 + L2 * cos(q3));

    q_model_deg = [q1, q2, q3] * 180 / pi;
    q_model_deg(2) = q_model_deg(2) - 90;

    servo_raw = home_deg + joint_sign .* q_model_deg;

    if isscalar(max_joint_delta_deg)
        low_lim = home_deg - max_joint_delta_deg;
        high_lim = home_deg + max_joint_delta_deg;
    else
        low_lim = home_deg - max_joint_delta_deg(:).';
        high_lim = home_deg + max_joint_delta_deg(:).';
    end

    servo_deg = min(max(servo_raw, low_lim), high_lim);
end

function bot = send_servo_command(bot, motor_ids, hold_servo_deg, arm_motor_ids, arm_servo_deg, MAX_ID, DEG2RAD)
    q_cmd = zeros(1, MAX_ID);
    q_cmd(motor_ids) = hold_servo_deg * DEG2RAD;
    q_cmd(arm_motor_ids) = arm_servo_deg * DEG2RAD;
    bot = bot.setQ(q_cmd);
    bot = bot.driveMotors();
end

function bot = send_all_servo_command(bot, motor_ids, servo_deg, MAX_ID, DEG2RAD)
    q_cmd = zeros(1, MAX_ID);
    q_cmd(motor_ids) = servo_deg * DEG2RAD;
    bot = bot.setQ(q_cmd);
    bot = bot.driveMotors();
end

function bot = ramp_all_servo_command(bot, motor_ids, from_deg, to_deg, MAX_ID, DEG2RAD, max_step_deg, dt)
    delta = to_deg - from_deg;
    steps = max(1, ceil(max(abs(delta(motor_ids))) / max_step_deg));

    for k = 1:steps
        s = k / steps;
        servo_deg = from_deg + s * delta;
        q_cmd = zeros(1, MAX_ID);
        q_cmd(motor_ids) = servo_deg * DEG2RAD;
        bot = bot.setQ(q_cmd);
        bot = bot.driveMotors();
        pause(dt);
    end
end

function ee_xyz = fk_from_model_deg(q_model_deg, H, base_forward_x_sign, shoulder_x_offset, L1, L2)
    q1 = q_model_deg(1) * pi / 180;
    q2 = (90 + q_model_deg(2)) * pi / 180;
    q3 = q_model_deg(3) * pi / 180;

    radial = shoulder_x_offset + L1 * cos(q2) + L2 * cos(q2 + q3);
    x = base_forward_x_sign * cos(q1) * radial;
    y = sin(q1) * radial;
    z = H + L1 * sin(q2) + L2 * sin(q2 + q3);

    ee_xyz = [x, y, z];
end
