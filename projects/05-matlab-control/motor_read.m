clear; clc;
load('UoM-RS-Var_Def.mat');

% -------- User settings --------
COM_PORT = "COM3";              % Change to your actual COM port
motor_ids = [1 2 3];            % [base, upper arm, forearm]
sample_dt = 0.02;               % Sampling interval (s)
read_duration = 120;            % Total read time (s)

% 3-axis arm geometry (cm)
H = 4.5;                        % base axis to shoulder axis
L1 = 19.5;                      % upper arm length
L2 = 13.0;                      % forearm length

% Servo reference (deg): captured at startup as vertical reference
home_deg = [0, 0, 0];
joint_sign = [1, 1, 1];         % flip to -1 if any joint direction is opposite
% ------------------------------

RAD2DEG = 180 / pi;
DEG2RAD = pi / 180;

bot = robot(COM_PORT, MAX_ID, BAUDRATE);
bot = bot.ConnectToArduino();
bot.serial_obj.Timeout = 1;
bot = bot.getID();

% Read-only mode: do not command movement
mode = zeros(1, MAX_ID);
mode(motor_ids) = POSITION;
bot = bot.setDriveControlMode(mode);

% Capture startup feedback as vertical baseline
bot = bot.getFB();
home_deg = bot.q_FB(motor_ids) * RAD2DEG;
fprintf('Captured vertical home_deg at startup: [%.2f, %.2f, %.2f]\n', ...
    home_deg(1), home_deg(2), home_deg(3));

num_samples = floor(read_duration / sample_dt) + 1;
t_data = zeros(num_samples, 1);
q_deg_data = zeros(num_samples, numel(motor_ids));
q_delta_deg_data = zeros(num_samples, numel(motor_ids));
ee_xyz_data = zeros(num_samples, 3);

% 3D visualization
fig = figure('Name', 'Manual Move - 3D Arm Realtime', 'NumberTitle', 'off');
tl = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
ax = nexttile(tl, 1);
grid(ax, 'on'); axis(ax, 'equal'); hold(ax, 'on');
xlabel(ax, 'X (cm)'); ylabel(ax, 'Y (cm)'); zlabel(ax, 'Z (cm)');
view(ax, 45, 25);
title(ax, 'Realtime Arm Pose from Encoder Angles');

reach = L1 + L2 + 5;
xlim(ax, [-reach, reach]);
ylim(ax, [-reach, reach]);
zlim(ax, [0, H + L1 + L2 + 5]);

% Draw base axis
plot3(ax, [0 0], [0 0], [0 H], 'k-', 'LineWidth', 2);

h_arm = plot3(ax, [0 0 0 0], [0 0 0 0], [0 H H H], 'b-o', ...
    'LineWidth', 2, 'MarkerFaceColor', 'b');
h_trace = animatedline('Color', 'r', 'LineWidth', 1.2);

ax_info = nexttile(tl, 2);
axis(ax_info, [0 1 0 1]);
axis(ax_info, 'off');
title(ax_info, 'Joint Delta From Startup (deg)');
t_id1 = text(ax_info, 0.05, 0.80, 'ID1: 0.00 deg', 'Color', [0.85 0.10 0.10], 'FontSize', 12, 'FontWeight', 'bold');
t_id2 = text(ax_info, 0.05, 0.62, 'ID2: 0.00 deg', 'Color', [0.10 0.55 0.10], 'FontSize', 12, 'FontWeight', 'bold');
t_id3 = text(ax_info, 0.05, 0.44, 'ID3: 0.00 deg', 'Color', [0.10 0.25 0.85], 'FontSize', 12, 'FontWeight', 'bold');
t_ee  = text(ax_info, 0.05, 0.20, 'EE: (0.00, 0.00, 0.00) cm', 'Color', [0.2 0.2 0.2], 'FontSize', 11);

t0 = tic;
for i = 1:num_samples
    if ~ishandle(fig)
        break;
    end

    % Feedback only (manual movement by user)
    bot = bot.getFB();
    q_rad = bot.q_FB(motor_ids);
    q_deg = q_rad * RAD2DEG;

    % Convert servo angle -> model joint angle
    q_model_deg = (q_deg - home_deg) ./ joint_sign;
    q1 = q_model_deg(1) * DEG2RAD;
    % Define vertical (+Z) as the zero-reference posture at startup.
    % When q_model_deg(2)=0, upper arm points along +Z.
    q2 = (90 + q_model_deg(2)) * DEG2RAD;
    q3 = q_model_deg(3) * DEG2RAD;

    % Forward kinematics
    p0 = [0, 0, 0];
    p1 = [0, 0, H];
    p2 = [cos(q1) * L1 * cos(q2), ...
          sin(q1) * L1 * cos(q2), ...
          H + L1 * sin(q2)];
    p3 = [cos(q1) * (L1 * cos(q2) + L2 * cos(q2 + q3)), ...
          sin(q1) * (L1 * cos(q2) + L2 * cos(q2 + q3)), ...
          H + L1 * sin(q2) + L2 * sin(q2 + q3)];

    set(h_arm, ...
        'XData', [p0(1), p1(1), p2(1), p3(1)], ...
        'YData', [p0(2), p1(2), p2(2), p3(2)], ...
        'ZData', [p0(3), p1(3), p2(3), p3(3)]);
    addpoints(h_trace, p3(1), p3(2), p3(3));

    t_data(i) = toc(t0);
    q_deg_data(i, :) = q_deg;
    q_delta_deg_data(i, :) = q_deg - home_deg;
    ee_xyz_data(i, :) = p3;

    t_id1.String = sprintf('ID1 (base): %+7.2f deg', q_delta_deg_data(i, 1));
    t_id2.String = sprintf('ID2 (upper): %+7.2f deg', q_delta_deg_data(i, 2));
    t_id3.String = sprintf('ID3 (fore):  %+7.2f deg', q_delta_deg_data(i, 3));
    t_ee.String = sprintf('EE: (%.2f, %.2f, %.2f) cm', p3(1), p3(2), p3(3));

    drawnow nocallbacks;
    t_next = i * sample_dt;
    t_now = toc(t0);
    if t_next > t_now
        pause(t_next - t_now);
    end
end

% Trim buffers if figure closed early
valid_idx = t_data > 0 | (1:num_samples)' == 1;
t_data = t_data(valid_idx);
q_deg_data = q_deg_data(valid_idx, :);
q_delta_deg_data = q_delta_deg_data(valid_idx, :);
ee_xyz_data = ee_xyz_data(valid_idx, :);

disp('Latest motor angles (deg):');
for k = 1:numel(motor_ids)
    fprintf('ID %d: %.2f deg\n', motor_ids(k), q_deg_data(end, k));
end

% Save trajectory CSV
csv_file = sprintf('motor_trace_%s.csv', datestr(now, 'yyyymmdd_HHMMSS'));
T = table( ...
    t_data, ...
    q_deg_data(:,1), q_deg_data(:,2), q_deg_data(:,3), ...
    q_delta_deg_data(:,1), q_delta_deg_data(:,2), q_delta_deg_data(:,3), ...
    ee_xyz_data(:,1), ee_xyz_data(:,2), ee_xyz_data(:,3), ...
    'VariableNames', {'time_s', ...
    'id1_deg','id2_deg','id3_deg', ...
    'id1_delta_deg','id2_delta_deg','id3_delta_deg', ...
    'ee_x_cm','ee_y_cm','ee_z_cm'});
writetable(T, csv_file);
fprintf('Saved trajectory CSV: %s\n', csv_file);
