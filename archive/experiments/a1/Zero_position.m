clear; clc; close all;

%% Parameters
b  = 2.5;
L1 = 5;
L2 = 18;
L3 = 12;
L4 = 6;

%% Joint angles
q1 = 0;
q2 = 0;
q3 = 0;
q4 = 0;
q5 = 0;

%% DH table [a, alpha, d, theta]
DH = [ ...
   -b,   pi/2,   L1,      q1;
    L2,    0,     0,      q2 + pi/2;
    L3,    0,     0,      q3;
     0,   -pi/2,  0,      q4 - pi/2;
     0,    0,    L4,      q5
];

%% Forward kinematics
[origins, T_ee] = forwardKinematics(DH);

O0 = origins(1,:);
O1 = origins(2,:);
O2 = origins(3,:);
O3 = origins(4,:);
O4 = origins(5,:);
O5 = origins(6,:);

%% Output positions
ee_pos = T_ee(1:3,4);

fprintf('\n=== End-Effector Position ===\n');
fprintf('X = %.2f cm\n', ee_pos(1));
fprintf('Y = %.2f cm\n', ee_pos(2));
fprintf('Z = %.2f cm\n', ee_pos(3));

fprintf('\n=== Joint / Frame Origins ===\n');
for i = 1:size(origins,1)
    fprintf('O%d: [%.2f, %.2f, %.2f]\n', ...
        i-1, origins(i,1), origins(i,2), origins(i,3));
end

%% Figure
figure('Color','w','Renderer','opengl');
hold on; grid on; axis equal;
view(38,24);

xlabel('X');
ylabel('Y');
zlabel('Z');

xlim([-25 35]);
ylim([-25 25]);
zlim([-5 35]);

set(gca, 'FontSize', 11, 'LineWidth', 1.0);

drawGround([-30 40], [-30 30], -0.01);

%% Links
drawTube(O0, O1, 1.25, [0.62 0.62 0.65], 48);
drawTube(O1, O2, 1.00, [0.61 0.61 0.66], 48);
drawTube(O2, O3, 1.00, [0.56 0.56 0.61], 48);
drawTube(O3, O4, 0.85, [0.51 0.51 0.56], 48);
drawTube(O4, O5, 0.70, [0.46 0.46 0.51], 48);

%% Joints
drawJointCylinder(O0, [0 0 1], 1.30, 2.0, [0.95 0.95 0.97], 50);
drawBall(O1, 1.25, [0.97 0.97 0.98], 48);
drawBall(O2, 1.10, [0.97 0.97 0.98], 48);
drawBall(O3, 1.00, [0.98 0.98 0.99], 48);
drawBall(O4, 0.90, [0.98 0.98 0.99], 48);

%% Motor caps
drawCap(O2, [0 0 -1], 0.75, 0.9, [0.12 0.12 0.13], 36);
drawCap(O3, [0 0 -1], 0.65, 0.8, [0.12 0.12 0.13], 36);
drawCap(O4, [0 0 -1], 0.55, 0.7, [0.12 0.12 0.13], 36);

%% Gripper
drawSimpleGripper(T_ee, 2.0, 0.9, 2.0);

%% Lighting / material
lighting gouraud;
camlight headlight;
camlight left;
material([0.25 0.65 0.08 8 1]);

%% Joint labels
offset = [1.5, 3.5, 1.5];

drawLabel(O0 + offset, 'Joint 0');
drawLabel(O1 + offset, 'Joint 1');
drawLabel(O2 + offset, 'Joint 2');
drawLabel(O3 + offset, 'Joint 3');
drawLabel(O4 + offset, 'Joint 4');
drawLabel(O5 + offset, 'EE');

%% =========================
function [origins, T] = forwardKinematics(DH)
    n = size(DH,1);
    T = eye(4);
    origins = zeros(n+1,3);
    origins(1,:) = [0 0 0];

    for i = 1:n
        A = dhTransform(DH(i,1), DH(i,2), DH(i,3), DH(i,4));
        T = T * A;
        origins(i+1,:) = T(1:3,4)';
    end
end

function A = dhTransform(a, alpha, d, theta)
    A = [ cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
          sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
          0,           sin(alpha),             cos(alpha),            d;
          0,           0,                      0,                     1 ];
end

function drawGround(xr, yr, z0)
    [X,Y] = meshgrid(xr, yr);
    Z = z0 * ones(size(X));
    surf(X,Y,Z, ...
        'FaceColor',[0.98 0.98 0.98], ...
        'EdgeColor','none', ...
        'FaceAlpha',0.96, ...
        'FaceLighting','gouraud');
end

function drawTube(p1, p2, r, color_rgb, n)
    v = p2 - p1;
    L = norm(v);
    if L < 1e-8, return; end

    [X,Y,Z] = cylinder(r, n);
    Z = Z * L;

    R = rotFromZToVec(v);
    pts = R * [X(:)'; Y(:)'; Z(:)'];

    Xr = reshape(pts(1,:), size(X)) + p1(1);
    Yr = reshape(pts(2,:), size(Y)) + p1(2);
    Zr = reshape(pts(3,:), size(Z)) + p1(3);

    surf(Xr, Yr, Zr, ...
        'FaceColor', color_rgb, ...
        'EdgeColor', 'none', ...
        'FaceLighting', 'gouraud', ...
        'SpecularStrength', 0.12, ...
        'DiffuseStrength', 0.85, ...
        'AmbientStrength', 0.30);
end

function drawBall(c, r, color_rgb, n)
    [X,Y,Z] = sphere(n);
    surf(r*X + c(1), r*Y + c(2), r*Z + c(3), ...
        'FaceColor', color_rgb, ...
        'EdgeColor', 'none', ...
        'FaceLighting', 'gouraud', ...
        'SpecularStrength', 0.15, ...
        'DiffuseStrength', 0.85, ...
        'AmbientStrength', 0.30);
end

function drawJointCylinder(center, axis_vec, radius, length_val, color_rgb, n)
    axis_vec = axis_vec(:) / norm(axis_vec);
    [X,Y,Z] = cylinder(radius, n);
    Z = Z * length_val - length_val/2;

    R = rotFromZToVec(axis_vec);
    pts = R * [X(:)'; Y(:)'; Z(:)'];

    Xr = reshape(pts(1,:), size(X)) + center(1);
    Yr = reshape(pts(2,:), size(Y)) + center(2);
    Zr = reshape(pts(3,:), size(Z)) + center(3);

    surf(Xr, Yr, Zr, ...
        'FaceColor', color_rgb, ...
        'EdgeColor', 'none', ...
        'FaceLighting', 'gouraud', ...
        'SpecularStrength', 0.12, ...
        'DiffuseStrength', 0.85, ...
        'AmbientStrength', 0.30);
end

function drawCap(center, dir_vec, radius, length_val, color_rgb, n)
    dir_vec = dir_vec(:);
    if norm(dir_vec) < 1e-10
        dir_vec = [0;0;1];
    end
    dir_vec = dir_vec / norm(dir_vec);
    start_point = center(:) + 0.15 * radius * dir_vec;
    drawOrientedCylinder(start_point, dir_vec, radius, length_val, color_rgb, n);
end

function drawOrientedCylinder(start_point, axis_vec, radius, length_val, color_rgb, n)
    axis_vec = axis_vec(:) / norm(axis_vec);
    [X,Y,Z] = cylinder(radius, n);
    Z = Z * length_val;

    R = rotFromZToVec(axis_vec);
    pts = R * [X(:)'; Y(:)'; Z(:)'];

    Xr = reshape(pts(1,:), size(X)) + start_point(1);
    Yr = reshape(pts(2,:), size(Y)) + start_point(2);
    Zr = reshape(pts(3,:), size(Z)) + start_point(3);

    surf(Xr, Yr, Zr, ...
        'FaceColor', color_rgb, ...
        'EdgeColor', 'none', ...
        'FaceLighting', 'gouraud', ...
        'SpecularStrength', 0.12, ...
        'DiffuseStrength', 0.85, ...
        'AmbientStrength', 0.30);
end

function drawSimpleGripper(T, palmLen, jawGap, fingerLen)
    o = T(1:3,4);
    R = T(1:3,1:3);

    y = R(:,2);
    z = R(:,3);

    drawOrientedCylinder(o - 0.5*z, z, 0.75, 1.0, [0.15 0.15 0.17], 36);

    palm_center = o + 0.45*palmLen*z;
    drawBlock(palm_center, R, [1.2, 1.8, palmLen], [0.25 0.25 0.28]);

    f1_center = o + palmLen*z + jawGap*y + 0.5*fingerLen*z;
    f2_center = o + palmLen*z - jawGap*y + 0.5*fingerLen*z;

    drawBlock(f1_center, R, [0.35, 0.35, fingerLen], [0.88 0.45 0.15]);
    drawBlock(f2_center, R, [0.35, 0.35, fingerLen], [0.88 0.45 0.15]);
end

function drawBlock(center, R, dims, color_rgb)
    lx = dims(1)/2;
    ly = dims(2)/2;
    lz = dims(3)/2;

    Vlocal = [ ...
        -lx -ly -lz;
         lx -ly -lz;
         lx  ly -lz;
        -lx  ly -lz;
        -lx -ly  lz;
         lx -ly  lz;
         lx  ly  lz;
        -lx  ly  lz]';

    V = (R * Vlocal + center(:))';

    F = [1 2 3 4;
         5 6 7 8;
         1 2 6 5;
         2 3 7 6;
         3 4 8 7;
         4 1 5 8];

    patch('Vertices',V,'Faces',F, ...
          'FaceColor',color_rgb, ...
          'EdgeColor','none', ...
          'FaceLighting','gouraud', ...
          'SpecularStrength',0.10, ...
          'DiffuseStrength',0.85, ...
          'AmbientStrength',0.30);
end

function drawLabel(pos, txt)
    text(pos(1), pos(2), pos(3), txt, ...
        'FontSize', 10, ...
        'Color', [0.1 0.1 0.1], ...
        'BackgroundColor', 'w', ...
        'Margin', 1);
end

function R = rotFromZToVec(v)
    v = v(:);
    if norm(v) < 1e-10
        R = eye(3);
        return;
    end

    v = v / norm(v);
    z0 = [0;0;1];
    c = dot(z0, v);

    if abs(c - 1) < 1e-10
        R = eye(3);
        return;
    elseif abs(c + 1) < 1e-10
        R = [-1 0 0; 0 -1 0; 0 0 1];
        return;
    end

    k = cross(z0, v);
    k = k / norm(k);
    ang = acos(c);

    K = [ 0    -k(3)  k(2);
          k(3)  0    -k(1);
         -k(2)  k(1)  0 ];

    R = eye(3) + sin(ang)*K + (1-cos(ang))*(K*K);
end