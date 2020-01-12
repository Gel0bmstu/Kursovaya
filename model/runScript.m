clear all;

% Set initial conditions

% -----------------------------------------------------------
% Default vals
% -----------------------------------------------------------
pi = 3.14;

g = [0, 9.81, 0]; % [m/s^2]
R = 6371e3; % [m]
M = 5.9726e24; % [kg]
dt = 0;

simulation_time = 10; % [sec]

% -----------------------------------------------------------
% Start SINS params
% -----------------------------------------------------------

% Axels
% -----------------------------------------------------------
Ax = 0;
Ay = 0;
Az = 0;

Ax_prev = 0;
Ay_prev = 0;
Az_prev = 0;

Ax_0 = Ax;
Ay_0 = Ay;
Az_0 = Az;

% Vels
% -----------------------------------------------------------
Vx = 0;
Vy = 0;
Vz = 0;

Vx_prev = 0;
Vy_prev = 0;
Vz_prev = 0;

Vx_0 = Vx;
Vy_0 = Vy;
Vz_0 = Vz;

% Angles
% -----------------------------------------------------------
dFx = 0;
dFy = 0;
dFz = 0;


% C matrix [body -> inertial] initial build
P = quaternion(cos(dFx/2), 0, sin(dFx/2), 0);
Q = quaternion(cos(dFz/2), 0, 0, sin(dFz/2));
R = quaternion(cos(dFy/2), sin(dFy/2), 0, 0);

q = P*Q*R;
[q0, q1, q2, q3] = parts(q);
C = [q0^2 + q1^2 - q2^2 - q3^2, 2 * (q1 * q2 - q3 * q0),   2 * (q1 * q3 + q0 * q2);
     2 * (q1 * q2 + q0 * q3),   q0^2 - q1^2 + q2^2 - q3^2, 2 * (q2 * q3 - q0 * q1);
     2 * (q1 * q3 - q0 * q2),   2 * (q2 * q3 + q0 * q1),   q0^2 - q1^2 - q2^2 + q3^2];


% Star simulink model
sim('model', simulation_time);