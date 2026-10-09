clc;
clear;
close all;

syms q1 q2 q3 q4 q5 real
syms L1 L2 L3 L4 b real
PI = sym(pi);

DH = @(a, alpha, d, theta) [ ...
    cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha),  a*cos(theta);
    sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha),  a*sin(theta);
    sym(0),      sin(alpha),             cos(alpha),             d;
    sym(0),      sym(0),                 sym(0),                 sym(1)];

%% A matrices
A1 = DH(-b,      PI/2,   L1,      q1);
A2 = DH(L2,      sym(0), sym(0),  q2 + PI/2);
A3 = DH(L3,      sym(0), sym(0),  q3);
A4 = DH(sym(0), -PI/2,   sym(0),  q4 - PI/2);
A5 = DH(sym(0),  sym(0), L4,      q5);

%% T matrices
T01 = simplify(A1);
T02 = simplify(A1*A2);
T03 = simplify(A1*A2*A3);
T04 = simplify(A1*A2*A3*A4);
T05 = simplify(A1*A2*A3*A4*A5);

%% position vectors p
p0 = sym([0;0;0]);
p1 = simplify(T01(1:3,4));
p2 = simplify(T02(1:3,4));
p3 = simplify(T03(1:3,4));
p4 = simplify(T04(1:3,4));
pe = simplify(T05(1:3,4));

%% z axes
z0 = sym([0;0;1]);
z1 = simplify(T01(1:3,3));
z2 = simplify(T02(1:3,3));
z3 = simplify(T03(1:3,3));
z4 = simplify(T04(1:3,3));

%% Jacobian columns
J1 = simplify([cross(z0, pe - p0); z0]);
J2 = simplify([cross(z1, pe - p1); z1]);
J3 = simplify([cross(z2, pe - p2); z2]);
J4 = simplify([cross(z3, pe - p3); z3]);
J5 = simplify([cross(z4, pe - p4); z4]);

%% Full Jacobian
J = simplify([J1 J2 J3 J4 J5]);

%% Optional: clean trig offsets
J = subs(J, cos(q2 + PI/2), -sin(q2));
J = subs(J, sin(q2 + PI/2),  cos(q2));
J = subs(J, cos(q4 - PI/2),  sin(q4));
J = subs(J, sin(q4 - PI/2), -cos(q4));
J = simplify(J);

%% Display
disp('pe = ');
disp(pe)

disp('J1 = ');
disp(J1)

disp('J2 = ');
disp(J2)

disp('J3 = ');
disp(J3)

disp('J4 = ');
disp(J4)

disp('J5 = ');
disp(J5)

disp('Full Jacobian J = ');
disp(J)