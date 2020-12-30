% upper arm: q1 = z, q2 = -y, q3 = x
% forearm: q4 = z, q5 = x
% wrist: q6 = z, q7 = x

clc; clear all;
syms q1 q2 q3 q4 q5 q6 q7 real;
[a, alpha, d, theta] = setup();


cnt = length(a);
A = cell(cnt, 1);
T = eye(4);

for n = 1:length(a)
    A{n} = simplify(DH(a(n), alpha(n), d(n), theta(n)));
    T = simplify(T * A{n});
end

position = simplify(vpa(T(1:4,4)), 5)

function [A] = DH(a, alpha, d, theta)
    A = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)
        sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)
        0, sin(alpha), cos(alpha), d
        0, 0, 0, 1];
end
   
function [a, alpha, d, theta] = setup()
    syms q1 q2 q3 q4 q5 q6 q7 real
    forearm_len = [0.25; 0.21; 0.26; 0.26; 0.26; 0.23; 0.26];
    upper_arm_len = [0.34; 0.30; 0.32; 0.29; 0.31; 0.33; 0.30];
    
    a = zeros(7,1);
    alpha = [pi/2; pi/2; -pi/2; pi/2; pi/2; pi/2; pi/2];
    d = [0; 0; upper_arm_len(1); 0; forearm_len(1); 0; 0];
    theta = [(pi/2) + q1; (3*pi)/2 + q2; q3; (pi/2) + q4; (pi/2) + q5; ...
        (pi/2) + q6; (pi/2) + q7];
end