clear all;

% Set initial conditions

% -----------------------------------------------------------
% Default vals
% -----------------------------------------------------------

g = [0, 9.8153, 0]; % [m/s^2]
Rad = 6378245; % [m]
h = 200; % [m]
M = 5.9726e24; % Earth mass [kg]
U = 7.29e-5; % [rad/s]
dt = 1/100; % Sampling rate [1/Hz]

simulation_time = 1/dt * 3600; % [sec]

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

Ax_n = zeros(1, simulation_time);
Ay_n = zeros(1, simulation_time);
Az_n = zeros(1, simulation_time);

Ax_n(1) = 0;
Ay_n(1) = 0;
Az_n(1) = 0;

% Vels
% -----------------------------------------------------------

% Linear velocity increment in body frame
Vx_b = 0;
Vy_b = 0;
Vz_b = 0;

Vx_n = zeros(1, simulation_time);
Vy_n = zeros(1, simulation_time);
Vz_n = zeros(1, simulation_time);

Vx_n(1) = 0;
Vy_n(1) = 0;
Vz_n(1) = 0;

% Angular vels
% -----------------------------------------------------------
Wx_b = 0;
Wy_b = 0;
Wz_b = 0;

Wx_n = zeros(1, simulation_time);
Wy_n = zeros(1, simulation_time);
Wz_n = zeros(1, simulation_time);

Wx_n(1) = 0;
Wy_n(1) = 0;
Wz_n(1) = 0;

% Angles
% -----------------------------------------------------------

% Angular vels incriments
Fx_b = 0;
Fy_b = 0;
Fz_b = 0;

Fx_n = zeros(1, simulation_time);
Fy_n = zeros(1, simulation_time);
Fz_n = zeros(1, simulation_time);

Fx_n(1) = 0;
Fy_n(1) = 0;
Fz_n(1) = 0;

psi   = zeros(1, simulation_time);
teta  = zeros(1, simulation_time);
gamma = zeros(1, simulation_time);

psi(1) = 0;
teta(1) = 0;
gamma(2) = 0;

% Moscow cords
phi = 56 / 57.3;
la = 38 / 57.3;
h   = zeros(1, simulation_time);
h(1) = 200;

% Rotation quaternions
% -----------------------------------------------------------
L = [1, 0, 0, 0];
G = [1, - U * cos(phi) * dt, - U * sin(phi) * dt, 0];
                          
% Start simulink model
% Отказался, так как не понял, как заставить работать функции
% из тулбокса кватерниона в матлаб моделях (самописных) симулинка
% sim('model', simulation_time);

tmp1 = zeros(1, simulation_time);
tmp2 = zeros(1, simulation_time);
tmp3 = zeros(1, simulation_time);

% Programm flags
debug = 0;
single_plot = 1;

% Programm start
% -----------------------------------------------------------
for i=2:simulation_time    
    % Calculate g with respect ot NSSK
    g = getcurrentg(200, phi); % Сюда должны передавать h
    gn = quatrotate(G, g);
    
    % Emulate IMU noise
    Ax_b = normrnd(g(1), aSigma);
    Ay_b = normrnd(g(2), aSigma);
    Az_b = normrnd(g(3), aSigma);
     
    Ab = [Ax_b; Ay_b; Az_b];
    
    Fx_b = normrnd(U * cos(phi) * dt, wSigma);
    Fy_b = normrnd(U * sin(phi) * dt, wSigma);
    Fz_b = normrnd(0, wSigma);
    
    Fb = [Fx_b; Fy_b; Fz_b];
    
    % Gyros compensation
    % ----
    
    % Axels compensation
    % ----
    
    % Recalculate G quat
    E = eye(4,4);
    A = make4SkewMatrix([U * cos(phi) * dt; U * sin(phi) * dt; 0]);
    dG = E + 0.5 * A + 0.25 * A^2;
    G = (dG * G')';
    
    % Recalculate main quat
    A = make4SkewMatrix(Fb);
    dL = E + 0.5 * A + 0.25 * A^2;
    L = (dL * L')';
        
    % Project IMU info to NSSK
    An = quatrotate(L, Ab');
    Fn = quatrotate(L, Fb');

    M = quat2rotm(L);
    
    m0 = sqrt(M(3,1)^2 + M(3,3)^2);

    gamma(i)  = atan2(M(3,2), m0);
    teta(i)   = atan2(M(1,2),M(2,2));
    psi(i)    = -atan2(M(3,1),M(3,3));

    Vx_n(i) = Vx_n(i-1) + (An(1) - gn(1)) * dt;
    Vy_n(i) = Vy_n(i-1) + (An(2) - gn(2)) * dt;
    Vz_n(i) = Vz_n(i-1) + (An(3) - gn(3)) * dt;

    h(i) = h(i-1) + Vy_n(i) * dt;
    
    if (debug)
        tmp1(i) = An(1);
        tmp2(i) = An(2);
        tmp3(i) = An(3);
    end

    if (0 == mod(i, 1000))
        i
    end
    
    % Sculling compensation
%     Vx_n(i) = Vx_n(i-1) + (An(1) + Vy_n(i-1) * Fn(3) - Vz_n(i-1) * Fn(2) - gn(1)) * dt;
%     Vy_n(i) = Vy_n(i-1) + (An(2) + Vz_n(i-1) * Fn(1) - Vx_n(i-1) * Fn(3) - gn(2)) * dt;
%     Vz_n(i) = Vz_n(i-1) + (An(3) + Vy_n(i-1) * Fn(1) - Vx_n(i-1) * Fn(2) - gn(3)) * dt;
end

t=1:simulation_time * dt;

if (debug)
    subplot(2, 2, 1);
    plot(t, tmp3(1/dt:1/dt:end));
    title('tmp1');
    grid on;

    subplot(2, 2, 2);
    plot(t, tmp2(1/dt:1/dt:end));
    title('tmp2');
    grid on;

    subplot(2, 2, 3);
    plot(t, tmp3(1/dt:1/dt:end));
    title('tmp3');
    grid on;
    
    subplot(2, 2, 4);
    plot(t, h(1/dt:1/dt:end));
    title('h');
    grid on;
    
    orv = [gamma(simulation_time); psi(simulation_time); teta(simulation_time)];
    orvg = [gamma(simulation_time) * 57.3; psi(simulation_time) * 57.3; teta(simulation_time) * 57.3];
    Vn = [Vx_n(simulation_time); Vy_n(simulation_time); Vz_n(simulation_time)];
elseif (single_plot)
    figure(1);
    plot(t, Vx_n(1/dt:1/dt:end));
    title('Vx');
    xlabel('Время, с');
    ylabel('Линейная скорость, м/с');
    grid on;

    figure(2);
    plot(t, Vy_n(1/dt:1/dt:end));
    title('Vy');
    xlabel('Время, с');
    ylabel('Линейная скорость, м/с');
    grid on;

    figure(3);
    plot(t, Vz_n(1/dt:1/dt:end));
    title('Vz');
    xlabel('Время, с');
    ylabel('Линейная скорость, м/с');
    grid on;

    figure(4);
    plot(t, gamma(1/dt:1/dt:end));
    title('gamma (X)');
    xlabel('Время, с');
    ylabel('Угол, рад');
    grid on;

    figure(5);
    plot(t, psi(1/dt:1/dt:end));
    title('psi (Y)');
    xlabel('Время, с');
    ylabel('Угол, рад');
    grid on;

    figure(6);
    plot(t, teta(1/dt:1/dt:end));
    title('teta (Z)');
    xlabel('Время, с');
    ylabel('Угол, рад');
    grid on;
    
    figure(7);
    plot(t, h(1/dt:1/dt:end));
    title('h');
    xlabel('Время, с');
    ylabel('Высота, м');
    grid on;
else
    subplot(2, 3, 1);
    plot(t, Vx_n(1/dt:1/dt:end));
    title('Vx');
    xlabel('Время, с');
    ylabel('Линейная скорость, м/с');
    grid on;

    subplot(2, 3, 2);
    plot(t, Vy_n(1/dt:1/dt:end));
    title('Vy');
    xlabel('Время, с');
    ylabel('Линейная скорость, м/с');
    grid on;

    subplot(2, 3, 3);
    plot(t, Vz_n(1/dt:1/dt:end));
    title('Vz');
    xlabel('Время, с');
    ylabel('Линейная скорость, м/с');
    grid on;

    subplot(2, 3, 4);
    plot(t, gamma(1/dt:1/dt:end));
    title('gamma (X)');
    xlabel('Время, с');
    ylabel('Угол, рад');
    grid on;

    subplot(2, 3, 5);
    plot(t, psi(1/dt:1/dt:end));
    title('psi (Y)');
    xlabel('Время, с');
    ylabel('Угол, рад');
    grid on;

    subplot(2, 3, 6);
    plot(t, teta(1/dt:1/dt:end));
    title('teta (Z)');
    xlabel('Время, с');
    ylabel('Угол, рад');
    grid on;
end