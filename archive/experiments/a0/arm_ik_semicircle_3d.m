clear; clc;
load('UoM-RS-Var_Def.mat');

% ================= User Settings =================
COM_PORT = "COM8";
motor_ids = [1 2 3];                 % [base, upper arm, forearm]
dt = 0.03;                           % control period (s)

% 3-axis arm geometry (cm)
H = 4.5;                             % base axis to shoulder axis
L1 = 19.5;                           % upper arm length
L2 = 13.0;                           % forearm length

% Home/baseline settings (same convention as motor_read.m)
use_fixed_home = true;
fixed_home_deg = [229.04, 304.63, 315.44];
home_deg = [0, 0, 0];
joint_sign = [1, 1, 1];              % flip to -1 if direction is opposite
max_joint_delta_deg = 120;            % safety limit (relative to home_deg)

% Semicircle target in WORLD frame:
% center = (18, 0, 0), plane = XOY, bulge toward +X
arc_center = [18.0, 0.0, 0.0];        % [x, y, z] cm
arc_radius = 10.0;                     % cm
num_points = 180;                    % more points -> smoother semicircle
return_points = 120;                 % more points -> smoother return segment
% =================================================

DEG2RAD = pi / 180;
RAD2DEG = 180 / pi;

% Connect robot
bot = robot(COM_PORT, MAX_ID, BAUDRATE);
bot = bot.ConnectToArduino();
bot.serial_obj.Timeout = 1;
bot = bot.getID();

mode = zeros(1, MAX_ID);
mode(motor_ids) = POSITION;
bot = bot.setDriveControlMode(mode);

% Determine vertical baseline (home_deg)
if use_fixed_home
    home_deg = fixed_home_deg;
    fprintf('Using fixed vertical home_deg: [%.2f, %.2f, %.2f]\n', ...
        home_deg(1), home_deg(2), home_deg(3));
else
    bot = bot.getFB();
    home_deg = bot.q_FB(motor_ids) * RAD2DEG;
    fprintf('Captured vertical home_deg at startup: [%.2f, %.2f, %.2f]\n', ...
        home_deg(1), home_deg(2), home_deg(3));
end
% Capture actual power-on/current position as return target (not home_deg)
startup_servo_deg = home_deg;  % fallback
max_read_trials = 3;
for k = 1:max_read_trials
    try
        bot = bot.getFB();
        startup_servo_deg = bot.q_FB(motor_ids) * RAD2DEG;
        break;
    catch
        pause(0.05);
    end
end
fprintf('Recorded startup_servo_deg: [%.2f, %.2f, %.2f]\n', ...
    startup_servo_deg(1), startup_servo_deg(2), startup_servo_deg(3));

% Precompute closed path in XOY plane:
% 1) semicircle (bulging toward +X), 2) return along +/−Y back to start.
phi = linspace(-pi/2, pi/2, num_points);
x_arc = arc_center(1) + arc_radius * cos(phi);
y_arc = arc_center(2) + arc_radius * sin(phi);
z_arc = arc_center(3) * ones(1, num_points);

% End of arc -> start of arc, move only along Y axis (X and Z fixed).
x_ret = x_arc(1) * ones(1, return_points);
y_ret = linspace(y_arc(end), y_arc(1), return_points);
z_ret = z_arc(1) * ones(1, return_points);

x_des = [x_arc, x_ret];
y_des = [y_arc, y_ret];
z_des = [z_arc, z_ret];
num_total = numel(x_des);

% Logs
t_data = zeros(num_total, 1);
cmd_deg_data = zeros(num_total, 3);
fb_deg_data = zeros(num_total, 3);   % kept for CSV compatibility; filled with NaN
ee_xyz_data = zeros(num_total, 3);

% Move to start point first, then hold for 1 second
xd0 = x_des(1);
yd0 = y_des(1);
zd0 = z_des(1);
q10 = atan2(yd0, xd0);
r0 = hypot(xd0, yd0);
z_local0 = zd0 - H;
D0 = (r0^2 + z_local0^2 - L1^2 - L2^2) / (2 * L1 * L2);
D0 = min(max(D0, -1), 1);
q30 = atan2(-sqrt(max(0, 1 - D0^2)), D0);
q20 = atan2(z_local0, r0) - atan2(L2 * sin(q30), L1 + L2 * cos(q30));
q_model0_deg = [q10, q20, q30] * RAD2DEG;
q_model0_deg(2) = q_model0_deg(2) - 90;
servo0_raw = home_deg + joint_sign .* q_model0_deg;
low_lim = home_deg - max_joint_delta_deg;
high_lim = home_deg + max_joint_delta_deg;
servo0_deg = min(max(servo0_raw, low_lim), high_lim);

q_cmd0 = zeros(1, MAX_ID);
q_cmd0(motor_ids) = servo0_deg * DEG2RAD;
bot = bot.setQ(q_cmd0);
bot = bot.driveMotors();
pause(3.0);

% Visualization disabled for smoother communication/control loop.
% fig = figure('Name', '3-Axis IK Semicircle (motor_read convention)', 'NumberTitle', 'off');
% tl = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
% ax = nexttile(tl, 1);
% grid(ax, 'on'); axis(ax, 'equal'); hold(ax, 'on');
% xlabel(ax, 'X (cm)'); ylabel(ax, 'Y (cm)'); zlabel(ax, 'Z (cm)');
% view(ax, 45, 25);
% title(ax, 'Semicircle IK + Realtime Arm');
% plot3(ax, x_des, y_des, z_des, 'k--', 'LineWidth', 1.0, 'DisplayName', 'Desired arc');
% reach = L1 + L2 + norm(arc_center(1:2)) + arc_radius + 5;
% xlim(ax, [-reach, reach]);
% ylim(ax, [-reach, reach]);
% zlim(ax, [0, H + L1 + L2 + 8]);
% plot3(ax, [0 0], [0 0], [0 H], 'k-', 'LineWidth', 2);
% h_link = plot3(ax, [0 0 0 0], [0 0 0 0], [0 H H H], 'b-o', ...
%     'LineWidth', 2, 'MarkerFaceColor', 'b', 'DisplayName', 'Arm');
% h_trace = animatedline('Color', 'r', 'LineWidth', 1.4, 'DisplayName', 'EE trace');
% legend(ax, 'show', 'Location', 'best');
% ax_info = nexttile(tl, 2);
% axis(ax_info, [0 1 0 1]); axis(ax_info, 'off');
% title(ax_info, 'Realtime Status (World XOY semicircle)');
% t_cmd = text(ax_info, 0.05, 0.78, 'CMD(deg): [0, 0, 0]', 'Color', [0.10 0.35 0.85], 'FontSize', 11, 'FontWeight', 'bold');
% t_fb  = text(ax_info, 0.05, 0.60, 'FB(deg):  [0, 0, 0]', 'Color', [0.10 0.55 0.10], 'FontSize', 11, 'FontWeight', 'bold');
% t_ee  = text(ax_info, 0.05, 0.42, 'EE: (0.00, 0.00, 0.00) cm', 'Color', [0.25 0.25 0.25], 'FontSize', 11);
% t_iq  = text(ax_info, 0.05, 0.24, 'IK status: OK', 'Color', [0.85 0.10 0.10], 'FontSize', 11, 'FontWeight', 'bold');

t0 = tic;
for i = 1:num_total
    % if ~ishandle(fig)
    %     break;
    % end

    xd = x_des(i);
    yd = y_des(i);
    zd = z_des(i);

    % IK: yaw + planar 2-link
    q1 = atan2(yd, xd);
    r = hypot(xd, yd);
    z_local = zd - H;

    D = (r^2 + z_local^2 - L1^2 - L2^2) / (2 * L1 * L2);
    D_clamped = min(max(D, -1), 1);

    % if abs(D) > 1
    %     t_iq.String = sprintf('IK status: clipped (idx=%d)', i);
    % else
    %     t_iq.String = 'IK status: OK';
    % end

    % Elbow-down branch
    q3 = atan2(-sqrt(max(0, 1 - D_clamped^2)), D_clamped);
    q2 = atan2(z_local, r) - atan2(L2 * sin(q3), L1 + L2 * cos(q3));

    % Convert IK model angles to servo angles with motor_read convention:
    % q2_model = q2_deg - 90 (so +Z vertical corresponds to 0 delta)
    q_model_deg = [q1, q2, q3] * RAD2DEG;
    q_model_deg(2) = q_model_deg(2) - 90;
    servo_deg_cmd_raw = home_deg + joint_sign .* q_model_deg;
    low_lim = home_deg - max_joint_delta_deg;
    high_lim = home_deg + max_joint_delta_deg;
    servo_deg_cmd = min(max(servo_deg_cmd_raw, low_lim), high_lim);

    % if any(abs(servo_deg_cmd - servo_deg_cmd_raw) > 1e-9)
    %     t_iq.String = sprintf('IK status: joint limited to +/-%.0f deg', max_joint_delta_deg);
    % end
    servo_rad_cmd = servo_deg_cmd * DEG2RAD;

    q_cmd = zeros(1, MAX_ID);
    q_cmd(motor_ids) = servo_rad_cmd;
    bot = bot.setQ(q_cmd);
    bot = bot.driveMotors();

    % No feedback read here (avoid readFB ERROR path in library).
    % Use commanded joint angles for FK/logging.
    q1f = q_model_deg(1) * DEG2RAD;
    q2f = (90 + q_model_deg(2)) * DEG2RAD;
    q3f = q_model_deg(3) * DEG2RAD;

    p3 = [cos(q1f) * (L1 * cos(q2f) + L2 * cos(q2f + q3f)), ...
          sin(q1f) * (L1 * cos(q2f) + L2 * cos(q2f + q3f)), ...
          H + L1 * sin(q2f) + L2 * sin(q2f + q3f)];

    t_data(i) = toc(t0);
    cmd_deg_data(i, :) = servo_deg_cmd;
    fb_deg_data(i, :) = [NaN, NaN, NaN];
    ee_xyz_data(i, :) = p3;

    % drawnow nocallbacks;  % visualization disabled
    t_next = i * dt;
    t_now = toc(t0);
    if t_next > t_now
        pause(t_next - t_now);
    end
end
pause(1.0);
% Return to startup position after finishing full closed semicircle
q_cmd_back = zeros(1, MAX_ID);
q_cmd_back(motor_ids) = startup_servo_deg * DEG2RAD;
bot = bot.setQ(q_cmd_back);
bot = bot.driveMotors();
pause(1.0);

% Trim and save CSV
valid_idx = t_data > 0 | (1:num_total)' == 1;
t_data = t_data(valid_idx);
cmd_deg_data = cmd_deg_data(valid_idx, :);
fb_deg_data = fb_deg_data(valid_idx, :);
ee_xyz_data = ee_xyz_data(valid_idx, :);

%csv_file = sprintf('ik_semicircle_%s.csv', datestr(now, 'yyyymmdd_HHMMSS'));
csv_file = sprintf('ik_semicircle.csv');
T = table( ...
    t_data, ...
    cmd_deg_data(:,1), cmd_deg_data(:,2), cmd_deg_data(:,3), ...
    fb_deg_data(:,1), fb_deg_data(:,2), fb_deg_data(:,3), ...
    ee_xyz_data(:,1), ee_xyz_data(:,2), ee_xyz_data(:,3), ...
    'VariableNames', {'time_s', ...
    'cmd_id1_deg','cmd_id2_deg','cmd_id3_deg', ...
    'fb_id1_deg','fb_id2_deg','fb_id3_deg', ...
    'ee_x_cm','ee_y_cm','ee_z_cm'});
writetable(T, csv_file);
fprintf('Saved IK trajectory CSV: %s\n', csv_file);
