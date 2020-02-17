clear all;

% Set initial conditions

% -----------------------------------------------------------
% Default vals
% -----------------------------------------------------------
pi = 3.14;

g = [0, 9.81, 0]; % [m/s^2]
Rad = 6378245; % [m]
h = 200; % [m]
M = 5.9726e24; % Earth mass [kg]
U = 7.29e-5; % [rad/s]
dt = 1/10; % Sampling rate [1/Hz]

simulation_time = 1/dt * 3600; % [sec]
% simulation_time = 84 * 60; 
% -----------------------------------------------------------
% Start SINS params
% -----------------------------------------------------------

% Sensors errors
% -----------------------------------------------------------
wSigma = 0.03 / 57.3 / 3600; % [rad/s]
aSigma = 3.3e-4 * 9.8; % [m/s^2]

% Axels
% -----------------------------------------------------------
Ax_b = 0;
Ay_b = 0;
Az_b = 0;

Ax_b_prev = 0;
Ay_b_prev = 0;
Az_b_prev = 0;

% Ax_0 = Ax;
% Ay_0 = Ay;
% Az_0 = Az;

% Vels
% -----------------------------------------------------------

% Linear velocity increment in body frame
Vx_b = 0;
Vy_b = 0;
Vz_b = 0;

Vx_n_prev = Vx_b;
Vy_n_prev = Vy_b;
Vz_n_prev = Vz_b;

% Vx_0 = Vx;
% Vy_0 = Vy;
% Vz_0 = Vz;

% Angular vels
% -----------------------------------------------------------
Wx_b = 0;
Wy_b = 0;
Wz_b = 0;

Wb = [0 0 0 0];
Wb_ks1 = [0 0 0; 0 0 0; 0 0 0];
Wb_ks2 = Wb_ks1;

Wx_prev = 0;
Wy_prev = 0;
Wz_prev = 0;

% Angles
% -----------------------------------------------------------

% Angular vels incriments
Fx_b = 0;
Fy_b = 0;
Fz_b = 0;
%F_b=[0 0 0];

psi = 0;
teta = 0;
gamma = 0;

% Moscow cords
% fi = 56 / 57.3;
% la = 38 / 57.3;
phi = 90 / 57.3;
la  = 90 / 57.3;

% Rotation quaternions
% -----------------------------------------------------------
% P = quaternion(cos(dFx/2), 0, sin(dFx/2), 0); % around Y 
% Q = quaternion(cos(dFz/2), 0, 0, sin(dFz/2)); % around Z
% R = quaternion(cos(dFy/2), sin(dFy/2), 0, 0); % around X

P = [cos(gamma/2), 0, sin(gamma/2), 0]; % around X
Q = [cos(teta/2), 0, 0, sin(teta/2)]; % around Z
R = [cos(psi/2), sin(psi/2), 0, 0]; % around Y

L = [1, 0, 0, 0];
% L = quatmultiply(P, quatmultiply(Q, R)); % Main quaternion L = R ? Q ? R
sL = conj(L);                            % Complex conjugate of main quaternion
                                         % (Сопряженный кватернион)
                          
% Main quaternion initial build
% q1 = quatmultiply(P, Q);
% q_prev = quatmultiply(q1, R);

% C matrix [body -> inertial] initial build 
q0 = real(L(1));
q1 = real(L(2));
q2 = real(L(3));
q3 = real(L(4));

C_prev = [q0^2 + q1^2 - q2^2 - q3^2, 2 * (q1 * q2 - q3 * q0),   2 * (q1 * q3 + q0 * q2);
     2 * (q1 * q2 + q0 * q3),   q0^2 - q1^2 + q2^2 - q3^2, 2 * (q2 * q3 - q0 * q1);
     2 * (q1 * q3 - q0 * q2),   2 * (q2 * q3 + q0 * q1),   q0^2 - q1^2 - q2^2 + q3^2];
 
% Start simulink model
% Отказался, так как не понял, как заставить работать функции
% из тулбокса кватерниона в матлаб моделях (самописных) симулинка
% sim('model', simulation_time);

Vx_n_plot = zeros(1, simulation_time);
Vy_n_plot = zeros(1, simulation_time);
Vz_n_plot = zeros(1, simulation_time);

Psi_plot = zeros(1, simulation_time);
Teta_plot = zeros(1, simulation_time);
Gamma_plot = zeros(1, simulation_time);

% Programm start
% -----------------------------------------------------------
for i=1:simulation_time
    % Calculate 'q' quat for IMU imulation
    g = getcurrentg(h, phi);
    
    % Emulate IMU noise
    Ax_b = normrnd(g(1), aSigma);
    Ay_b = normrnd(g(2), aSigma);
    Az_b = normrnd(g(3), aSigma);
     
%     Ax_b = g(1);
%     Ay_b = g(2);
%     Az_b = g(3);
     
    Ab = [Ax_b, Ay_b, Az_b];
    
    Fx_b = normrnd(U * sin(phi) * dt, wSigma);
    Fy_b = normrnd(U * cos(phi) * dt, wSigma);
    Fz_b = normrnd(0, wSigma);

%     Fx_b = U * sin(phi);
%     Fy_b = U * cos(phi);
%     Fz_b = 0;
    
    Fb = [Fx_b, Fy_b, Fz_b];
    
    % Calculte angular velosytes
    Wx_b = Wx_b + Fb(1);
    Wy_b = Wy_b + Fb(2);
    Wz_b = Wz_b + Fb(3);
    
    Wb = [Wx_b, Wy_b, Wz_b];
    
    % Calculate orientation vector
    
    % Cross-simetric matrix of angular vels
    Wb_ks = [0 -Wb(3) Wb(2); Wb(3) 0 -Wb(1); -Wb(2) Wb(3) 0];
    
    % Orientation vector increes
    dEf = Fb + Wb_ks1 .* Fb ./ 6 - Wb_ks2 .* Fb ./ 24;
    
    % Amount of orientation vector
    dTeta = sqrt(dEf(1)^2 + dEf(2)^2 + dEf(3)^2);
    
    Wb_ks2 = Wb_ks1;
    Wb_ks1 = Wb_ks;
    
    % Calculate increment of quaternion
    dL0 = 1 - dTeta^2/8 + dTeta^4/384; 
    dL1 = dEf(1) * (0.5 - dTeta^2/48 + dTeta^4/512);
    dL2 = dEf(2) * (0.5 - dTeta^2/48 + dTeta^4/512);
    dL3 = dEf(3) * (0.5 - dTeta^2/48 + dTeta^4/512);
    
    % Incresment L quat
    dL = [dL0 dL1 dL2 dL3];
    % Earth rotation quat
    G = [1, - Fb(1), - Fb(2), -Fb(3)];
    
    % Recalculate main quat
    L = quatmultiply(L, dL);
    L = quatmultiply(G, L);
    
    % Conning compensation of gyros angular vels
    % ----
        
    % Project IMU info to NSSK
    An = quatrotate(L, Ab);
    Fn = quatrotate(L, Fb);
    
    M = quat2rotm(L);
    m0 = sqrt(M(3,1)^2 + M(3,3)^2);
    
%     l = realpartsvar(quaternion(L));
%     M(3,2) = 2 * (l(3) * l(4) + l(1) * l(2));
%     M(3,1) = 2 * (l(2) * l(4) - l(1) * l(3));
%     M(3,3) = l(1)^2 + l(4)^2 - l(2)^2- l(3)^2;
%     M(1,2) = 2 * (l(2) * l(3) - l(1) * l(4));
%     M(2,2) = l(1)^2 + l(3)^2 - l(2)^2- l(4)^2;
    
    teta  = atan2(M(3,2), m0);
    psi   = atan2(M(1,2),M(2,2));
    gamma = -atan2(M(3,1),M(3,3));
% 
%     teta = teta + Fn(2);
%     psi = psi + Fn(3);
%     gamma = gamma + Fn(1);

    % Gyros compensation
    % ----
    
    % Sculling compensation
    Vx_n = Vx_n_prev + (An(1) + Vy_n_prev * Fn(3) - Vz_n_prev * Fn(2) - g(1)) * dt;
    Vy_n = Vy_n_prev + (An(2) + Vz_n_prev * Fn(1) - Vx_n_prev * Fn(3) - g(2)) * dt;
    Vz_n = Vz_n_prev + (An(3) + Vy_n_prev * Fn(1) - Vx_n_prev * Fn(2) - g(3)) * dt;
    
    Vx_n = Vx_n + (2 * U * cos(phi) * Vy_n - Vx_n * (Vx_n / Rad + 2 * U * cos(phi))) * dt;
    Vy_n = Vy_n - (2 * U * sin(phi) * Vx_n - Vx_n * (Vy_n / Rad + 2 * U * sin(phi))) * dt;

    Psi_plot(i) = psi;
    Teta_plot(i) = teta;
    Gamma_plot(i) = gamma;
    
    Vx_n_prev = Vx_n;
    Vy_n_prev = Vy_n;
    Vz_n_prev = Vz_n;
    
    Vx_b_plot(i) = Vx_n;
    Vy_b_plot(i) = Vy_n;
    Vz_b_plot(i) = Vz_n;
    
    % Body to nav valc
    
    % Orientation angles calc
    
    if (0 == mod(i, 1000))
        det(M)
        i
    end
end

t=1:simulation_time;
figure(1);
plot(t, Vx_b_plot);
grid on;

figure(2);
plot(t, Vy_b_plot);
grid on;

figure(3);
plot(t, Vz_b_plot);
grid on;

figure(4);
plot(t, Psi_plot);
grid on;

figure(5);
plot(t, Teta_plot);
grid on;

figure(6);
plot(t, Gamma_plot);
grid on;