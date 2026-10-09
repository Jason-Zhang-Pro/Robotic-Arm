clc;
clear;
close all;

%% =========================================================
%  Symbolic variables
%% =========================================================
syms q1 q2 q3 q4 q5 real
syms L1 L2 L3 L4 b real
PI = sym(pi);

q = [q1; q2; q3; q4; q5];

%% =========================================================
%  DH transformation matrix
%% =========================================================
DH = @(a, alpha, d, theta) [ ...
    cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha),  a*cos(theta);
    sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha),  a*sin(theta);
    sym(0),      sin(alpha),             cos(alpha),             d;
    sym(0),      sym(0),                 sym(0),                 sym(1)];

%% =========================================================
%  Forward kinematics
%% =========================================================
A1 = DH(-b,      PI/2,   L1,      q1);
A2 = DH(L2,      sym(0), sym(0),  q2 + PI/2);
A3 = DH(L3,      sym(0), sym(0),  q3);
A4 = DH(sym(0), -PI/2,   sym(0),  q4 - PI/2);
A5 = DH(sym(0),  sym(0), L4,      q5);

T01 = A1;
T02 = A1*A2;
T03 = A1*A2*A3;
T04 = A1*A2*A3*A4;
T05 = A1*A2*A3*A4*A5;

%% =========================================================
%  Position vectors and z axes
%% =========================================================
p0 = sym([0;0;0]);
p1 = T01(1:3,4);
p2 = T02(1:3,4);
p3 = T03(1:3,4);
p4 = T04(1:3,4);
pe = T05(1:3,4);

z0 = sym([0;0;1]);
z1 = T01(1:3,3);
z2 = T02(1:3,3);
z3 = T03(1:3,3);
z4 = T04(1:3,3);

R = T05(1:3,1:3);

%% =========================================================
%  1) Geometric Jacobian
%% =========================================================
J_geo = [ ...
    [cross(z0, pe - p0); z0], ...
    [cross(z1, pe - p1); z1], ...
    [cross(z2, pe - p2); z2], ...
    [cross(z3, pe - p3); z3], ...
    [cross(z4, pe - p4); z4]  ...
];

J_geo = trig_cleanup(J_geo, q2, q4, PI);

disp('Geometric Jacobian J_geo = ');
disp(J_geo)

%% =========================================================
%  2) Differential Jacobian (partial derivative method)
%% =========================================================

% Linear part
Jv = jacobian(pe, q);

% Angular part
Jw = sym(zeros(3,5));

for i = 1:5
    dR_dqi = diff(R, q(i));
    Omega_i = simplify(dR_dqi * transpose(R));

    Jw(:,i) = simplify([ ...
        Omega_i(3,2);
        Omega_i(1,3);
        Omega_i(2,1) ...
    ]);
end

J_diff = [Jv; Jw];
J_diff = trig_cleanup(J_diff, q2, q4, PI);

disp('Differential Jacobian J_diff = ');
disp(J_diff)

%% =========================================================
%  3) Symbolic comparison
%% =========================================================
J_compare = simplify(J_geo - J_diff);

disp('Difference J_geo - J_diff = ');
disp(J_compare)

is_equal_matrix = arrayfun(@(x) isAlways(x == 0), J_compare);

disp('Element-wise symbolic check (1 = equal):');
disp(is_equal_matrix)

if all(is_equal_matrix, 'all')
    disp('Verification passed: J_geo and J_diff are identical.');
else
    disp('Symbolic comparison not fully confirmed.');
end

%% =========================================================
%  Local function: trig cleanup
%% =========================================================
function J_out = trig_cleanup(J_in, q2, q4, PI)
    J_out = subs(J_in, cos(q2 + PI/2), -sin(q2));
    J_out = subs(J_out, sin(q2 + PI/2),  cos(q2));
    J_out = subs(J_out, cos(q4 - PI/2),  sin(q4));
    J_out = subs(J_out, sin(q4 - PI/2), -cos(q4));
    J_out = simplify(J_out);
end