clear; clc;
load('UoM-RS-Var_Def.mat');

COM_PORT = "COM3";
bot = robot(COM_PORT, MAX_ID, BAUDRATE);

% ===== 建立连接 =====
bot = bot.ConnectToArduino();
bot.serial_obj.Timeout = 3;

% ===== 获取电机ID =====
bot = bot.getID();

% ===== 设置 POSITION 模式 =====
mode = zeros(1, MAX_ID);
mode(1:3) = POSITION;
bot = bot.setDriveControlMode(mode);

%% ================= STEP 1: 读取当前角度 =================
bot = bot.getFB();

q_init = bot.q_FB(1:3);   % rad
q_init_deg = rad2deg(q_init);

disp("Current joint positions (deg):");
disp(q_init_deg);

%% ================= STEP 2: 用当前读取角度计算末端坐标 =================

% ===== 机械臂参数 =====
H  = 4.5;
L1 = 19.5;
L2 = 13.0;

% ===== 固定 home（不要用当前角度当 home）=====
home_deg = [327.9199 123.6621 314.7363];
joint_sign = [1 1 1];

% ===== servo角 -> model角 =====
q_model_deg = (q_init_deg - home_deg) ./ joint_sign;

disp("Model joint angles (deg):");
disp(q_model_deg);

% ===== FK =====
q1 = q_model_deg(1) * pi / 180;
q2 = (90 + q_model_deg(2)) * pi / 180;
q3 = q_model_deg(3) * pi / 180;

x = cos(q1) * (L1 * cos(q2) + L2 * cos(q2 + q3));
y = sin(q1) * (L1 * cos(q2) + L2 * cos(q2 + q3));
z = H + L1 * sin(q2) + L2 * sin(q2 + q3);

disp("Current end-effector position (cm):");
disp([x y z]);