clear all;

% Set initial conditions

% -----------------------------------------------------------
% Default vals
% -----------------------------------------------------------
settings = ModelSettings(3600, 400);
settings = settings.set_debug_mode_flag(true);
settings = settings.set_print_solutions_flag(true);
settings = settings.set_subplot_print_flag(true);
settings = settings.set_save_solutions_flag(true);

% -----------------------------------------------------------
% Start SINS params
% -----------------------------------------------------------
% Moscow cords
% phi = 56 / 57.3;
% la = 38 / 57.3;
% h   = zeros(1, simulation_time);
% h(1) = 200;

% Errors
% -----------------------------------------------------------
table_error = 1e-5; % Ошибка поворотного стола [рад]
axis_alligment_error = 1e-5; % Ошибка соосности осей БЧЭ и ССК [рад]

% Ошибки рассогласования осей
psi_err   = 1.5; % Рассогласование осей по оси X [град]
teta_err  = 1.0; % Рассогласование осей по оси Z [град]
gamma_err = 2;   % Рассогласование осей по оси Y [град]

% Sensors errors
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

Fx_n(1) = normrnd(U * cos(phi) * dt, wSigma);
Fy_n(1) = normrnd(U * sin(phi) * dt, wSigma);
Fz_n(1) = 0;

psi   = zeros(1, simulation_time);
teta  = zeros(1, simulation_time);
gamma = zeros(1, simulation_time);

psi(1)   = psi_err / 57.3;
teta(1)  = teta_err / 57.3;
gamma(1) = gamma_err / 57.3;

% NIR values
% -----------------------------------------------------------


% Wx_true_bias = (2 * randi([0 1]) - 1) * (-0.03924 + rand * 2 * 0.03924);
% Wy_true_bias = (2 * randi([0 1]) - 1) * (-0.03924 + rand * 2 * 0.03924);
% Wz_true_bias = (2 * randi([0 1]) - 1) * (-0.03924 + rand * 2 * 0.03924);
% 
% Wx_true_scale_f = (1 - 2e-4 + rand * 4e-4);
% Wy_true_scale_f = (1 - 2e-4 + rand * 4e-4);
% Wz_true_scale_f = (1 - 2e-4 + rand * 4e-4);
% 
% fprintf(fw, "%s\n", "Реальное смещение нуля гироскопов:");
% fprintf(fw, "Wx: %6f\n", Wx_true_bias);
% fprintf(fw, "Wy: %6f\n", Wy_true_bias);
% fprintf(fw, "Wz: %6f\n\n", Wz_true_bias);
% 
% fprintf(fw, "%s\n", "Реальные масштабные коэффициенты гироскопов:");
% fprintf(fw, "Wx: %6f\n", Wx_true_scale_f);
% fprintf(fw, "Wy: %6f\n", Wy_true_scale_f);
% fprintf(fw, "Wz: %6f\n\n", Wz_true_scale_f);

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
nir_calibration = 1;

E = eye(4);

function calibrated_params = calibration(calibration_time, settings)
    
    fa = fopen(settings.axels_errors_file_path, 'w+');
    fw = fopen(settings.gyros_errors_file_path, 'w+');

    if (settings.log_calibration_parameters_flag)
        fprintf(fa, "%s\n", "Реальное смещение нуля акселерометров:");
        fprintf(fa, "Ax: %6f\n", Ax_true_bias);
        fprintf(fa, "Ay: %6f\n", Ay_true_bias);
        fprintf(fa, "Az: %6f\n\n", Az_true_bias);

        fprintf(fa, "%s\n", "Реальные масштабные коэффициенты акселерометров:");
        fprintf(fa, "Ax: %6f\n", Ax_true_scale_f);
        fprintf(fa, "Ay: %6f\n", Ay_true_scale_f);
        fprintf(fa, "Az: %6f\n\n", Az_true_scale_f);
    end

    % Setting calibration angles for sensors
    axels_calibrating_angenls = [  
        % X
        0, 90, 0;  %1
        0, 270, 0; %2
        
        % Y
        90, 0, 0;  %3
        270, 0, 0; %4
        
        % Z
        0, 0, 0;   %5
        0, 180, 0; %6
        ];
    
    gyros_calibrating_angenls = [  
        % N and Up
        0, 0, 0;    %1
        0, 180, 0;  %2
        
        % E
        90, 0, 0;   %3
        90, 0, 180; %4
        ];
    
    % Create array 
    % Fill in the array with zeros
    cAx_1 = zeros(1,t_calibrate_1);
    cAx_2 = zeros(1,t_calibrate_1);
    cAy_1 = zeros(1,t_calibrate_1);
    cAy_2 = zeros(1,t_calibrate_1);
    cAz_1 = zeros(1,t_calibrate_1);
    cAz_2 = zeros(1,t_calibrate_1);
end

% Инициализируем процесс калибровки перед началом работы БИНС
% в режиме навигации
if (nir_calibration)
    %
    % X = E (teta)
    % Y = N (gamma)
    % Z = Up (psi)
    % Задаем углы поворота, при которых мы будем калибровать БЧЭ [град]

    
    cAx_1 = zeros(1,t_calibrate_1);
    cAx_2 = zeros(1,t_calibrate_1);
    cAy_1 = zeros(1,t_calibrate_1);
    cAy_2 = zeros(1,t_calibrate_1);
    cAz_1 = zeros(1,t_calibrate_1);
    cAz_2 = zeros(1,t_calibrate_1);
%     
%     cWx_1 = zeros(1,t_calibrate_1);
%     cWx_2 = zeros(1,t_calibrate_1);
%     cWy_1 = zeros(1,t_calibrate_1);
%     cWy_2 = zeros(1,t_calibrate_1);
%     cWz_1 = zeros(1,t_calibrate_1);
%     cWz_2 = zeros(1,t_calibrate_1);
% 
%     Ux = @(psi, teta, gamma) U * cos(phi) * (sin(gamma / 57.3) * cos(psi / 57.3) * sin(teta / 57.3) + sin(gamma / 57.3) * sin(psi / 57.3)) + U * sin(phi) * sin(gamma / 57.3) * cos(teta / 57.3);
%     Uy = @(psi, teta, gamma) U * cos(phi) * cos(psi / 57.3) + U * sin(phi) * sin(teta / 57.3);
%     Uz = @(psi, teta, gamma) U * cos(phi) * (-cos(gamma / 57.3) * cos(psi / 57.3) * sin(teta / 57.3) + sin(gamma / 57.3) * sin(psi / 57.3)) + U * sin(phi) * cos(gamma / 57.3) * cos(teta / 57.3);
    
    % Запишем истинные значения ускорения силы тяжести в ГСК
    gc = 9.81;
    
    % Для каждого поворота
    % Записываем в массив измеренное смоделированное значение
    for i=1:t_calibrate_1
        % Акселерометры
        cAx_1(i) = normrnd(-gc * cos(a_calibrating_angenls(1, 1) / 57.3) * sin(a_calibrating_angenls(1, 2) / 57.3), aSigma) * Ax_true_scale_f + Ax_true_bias;
        cAx_2(i) = normrnd(-gc * cos(a_calibrating_angenls(2, 1) / 57.3) * sin(a_calibrating_angenls(2, 2) / 57.3), aSigma) * Ax_true_scale_f + Ax_true_bias;
        
        cAy_1(i) = normrnd(gc * sin(a_calibrating_angenls(3, 1) / 57.3), aSigma) * Ay_true_scale_f + Ay_true_bias;
        cAy_2(i) = normrnd(gc * sin(a_calibrating_angenls(4, 1) / 57.3), aSigma) * Ay_true_scale_f + Ay_true_bias;
        
        cAz_1(i) = normrnd(gc * cos(a_calibrating_angenls(5, 1) / 57.3) * cos(a_calibrating_angenls(5, 2) / 57.3), aSigma) * Az_true_scale_f + Az_true_bias;
        cAz_2(i) = normrnd(gc * cos(a_calibrating_angenls(6, 1) / 57.3) * cos(a_calibrating_angenls(6, 2) / 57.3), aSigma) * Az_true_scale_f + Az_true_bias;
        
%         % Гироскопы
%         cWx_1(i) = normrnd(Ux(w_calibrating_angenls(1, 1), w_calibrating_angenls(1, 2), w_calibrating_angenls(1, 3)), wSigma) * Wz_true_scale_f + Wz_true_bias;
%         cWx_2(i) = normrnd(Ux(w_calibrating_angenls(2, 1), w_calibrating_angenls(2, 2), w_calibrating_angenls(2, 3)), wSigma) * Wz_true_scale_f + Wz_true_bias;
%         
%         cWy_1(i) = normrnd(Uy(w_calibrating_angenls(1, 1), w_calibrating_angenls(1, 2), w_calibrating_angenls(1, 3)), wSigma) * Wz_true_scale_f + Wz_true_bias;
%         cWy_2(i) = normrnd(Uy(w_calibrating_angenls(2, 1), w_calibrating_angenls(2, 2), w_calibrating_angenls(2, 3)), wSigma) * Wz_true_scale_f + Wz_true_bias;
%         
%         cWz_1(i) = normrnd(Uz(w_calibrating_angenls(3, 1), w_calibrating_angenls(3, 2), w_calibrating_angenls(3, 3)), wSigma) * Wz_true_scale_f + Wz_true_bias;
%         cWz_2(i) = normrnd(Uz(w_calibrating_angenls(4, 1), w_calibrating_angenls(4, 2), w_calibrating_angenls(4, 3)), wSigma) * Wz_true_scale_f + Wz_true_bias;
    end
    
    % Находим среднее значение измеренных величин g
    meanAx1 = mean(cAx_1);
    meanAx2 = mean(cAx_2);

    meanAy1 = mean(cAy_1);
    meanAy2 = mean(cAy_2);

    meanAz1 = mean(cAz_1);
    meanAz2 = mean(cAz_2);

    % Находим смещение нуля и масштабный коэффициент акселерометров
    Ax_z_bias = (meanAx1 + meanAx2) / 2;
    Ay_z_bias = (meanAy1 + meanAy2) / 2;
    Az_z_bias = (meanAz1 + meanAz2) / 2;

    Ax_scale_f = (meanAx2 - meanAx1) / 2 / gc;
    Ay_scale_f = (meanAy1 - meanAy2) / 2 / gc;
    Az_scale_f = (meanAz1 - meanAz2) / 2 / gc;
    
    fprintf(fa, "%s\n", "Вычисленное смещение нуля акселерометров:");
    fprintf(fa, "Ax: %6f\n", Ax_z_bias);
    fprintf(fa, "Ay: %6f\n", Ay_z_bias);
    fprintf(fa, "Az: %6f\n\n", Az_z_bias);

    fprintf(fa, "%s\n", "Вычесленные масштабные коэффициенты акселерометров:");
    fprintf(fa, "Ax: %6f\n", Ax_scale_f);
    fprintf(fa, "Ay: %6f\n", Ay_scale_f);
    fprintf(fa, "Az: %6f\n\n", Az_scale_f);
    
    fprintf(fa, "%s\n", "--------------------------------------------------");
    
    fprintf(fa, "%s\n", "Ошибка калибровки смещения нуля акселерометров:");
    fprintf(fa, "Ax: |%6f - %6f| = %6f\n", Ax_true_bias, Ax_z_bias, abs(Ax_true_bias - Ax_z_bias));
    fprintf(fa, "Ay: |%6f - %6f| = %6f\n", Ay_true_bias, Ay_z_bias, abs(Ay_true_bias - Ay_z_bias));
    fprintf(fa, "Ay: |%6f - %6f| = %6f\n\n", Az_true_bias, Az_z_bias, abs(Az_true_bias - Az_z_bias));
    
    fprintf(fa, "%s\n", "Ошибка калибровки масштабного коэффициента акселерометров:");
    fprintf(fa, "Ax: |%6f - %6f| = %6f\n", Ax_true_scale_f, Ax_scale_f, abs(Ax_true_scale_f - Ax_scale_f));
    fprintf(fa, "Ay: |%6f - %6f| = %6f\n", Ay_true_scale_f, Ay_scale_f, abs(Ay_true_scale_f - Ay_scale_f));
    fprintf(fa, "Ay: |%6f - %6f| = %6f\n\n", Az_true_scale_f, Az_scale_f, abs(Az_true_scale_f - Az_scale_f));
    
%     % Находим среднее значение скоростей Земли U
%     meanWx1 = mean(cWx_1);
%     meanWx2 = mean(cWx_2);
%     
%     meanWy1 = mean(cWy_1);
%     meanWy2 = mean(cWy_2);
%     
%     meanWz1 = mean(cWz_1);
%     meanWz2 = mean(cWz_2);
%     
%     % Находим смещение нуля имасштабный коэффициент гироскопов
%     Wx_z_bias = (meanWx1 + meanWx2) / 2;
%     Wy_z_bias = (meanWy1 + meanWy2) / 2;
%     Wz_z_bias = (meanWz1 + meanWz2) / 2;
%     
%     Wx_scale_f = (meanWx2 - meanWx1) / 2 / Ux(0, 0, 0);
%     Wy_scale_f = (meanWy1 - meanWy2) / 2 / Uy(0, 0, 0);
%     Wz_scale_f = (meanWz1 - meanWz2) / 2 / Uz(0, 0, 0);
%     
%     fprintf(fw, "%s\n", "Вычисленное смещение нуля гироскопов:");
%     fprintf(fw, "Ax: %6f\n", Ax_z_bias);
%     fprintf(fw, "Ay: %6f\n", Ay_z_bias);
%     fprintf(fw, "Az: %6f\n\n", Az_z_bias);
% 
%     fprintf(fw, "%s\n", "Вычесленные масштабные коэффициенты гироскопов:");
%     fprintf(fw, "Ax: %6f\n", Ax_scale_f);
%     fprintf(fw, "Ay: %6f\n", Ay_scale_f);
%     fprintf(fw, "Az: %6f\n\n", Az_scale_f);
%     
%     fprintf(fw, "%s\n", "--------------------------------------------------");
%     
%     fprintf(fw, "%s\n", "Ошибка калибровки смещения нуля гироскопов:");
%     fprintf(fw, "Ax: |%6f - %6f| = %6f\n", Ax_true_bias, Ax_z_bias, abs(Ax_true_bias - Ax_z_bias));
%     fprintf(fw, "Ay: |%6f - %6f| = %6f\n", Ay_true_bias, Ay_z_bias, abs(Ax_true_bias - Ay_z_bias));
%     fprintf(fw, "Ay: |%6f - %6f| = %6f\n\n", Az_true_bias, Az_z_bias, abs(Ax_true_bias - Az_z_bias));
%     
%     fprintf(fw, "%s\n", "Ошибка калибровки масштабного коэффициента акселерометров:");
%     fprintf(fw, "Ax: |%6f - %6f| = %6f\n", Ax_true_scale_f, Ax_scale_f, abs(Ax_true_scale_f - Ax_scale_f));
%     fprintf(fw, "Ay: |%6f - %6f| = %6f\n", Ay_true_scale_f, Ay_scale_f, abs(Ay_true_scale_f - Ay_scale_f));
%     fprintf(fw, "Ay: |%6f - %6f| = %6f\n\n", Az_true_scale_f, Az_scale_f, abs(Az_true_scale_f - Az_scale_f));
%     
    fclose(fa);
    fclose(fw);
end 

% Функции вычисления ускорений и угловых приращений с учеом
% углов рассогласования ССК и "ГСК"
% -----------------------------------------------------------
Ax_err = @(psi, teta, gamma) gc * sin(teta_err / 57.3);
Ay_err = @(psi, teta, gamma) gc * cos(teta_err / 57.3) * cos(gamma / 57.3);
Az_err = @(psi, teta, gamma) - gc * cos(teta_err / 57.3) * sin(gamma / 57.3);

Ux_err = @(psi, teta, gamma) U * cos(phi) * cos(psi / 57.3) + U * sin(phi) * sin(teta / 57.3);
Uy_err = @(psi, teta, gamma) U * cos(phi) * (-cos(gamma / 57.3) * cos(psi / 57.3) * sin(teta / 57.3) + sin(gamma / 57.3) * sin(psi / 57.3)) + U * sin(phi) * cos(gamma / 57.3) * cos(teta / 57.3);
Uz_err = @(psi, teta, gamma) U * cos(phi) * (sin(gamma / 57.3) * cos(psi / 57.3) * sin(teta / 57.3) + sin(gamma / 57.3) * sin(psi / 57.3)) + U * sin(phi) * sin(gamma / 57.3) * cos(teta / 57.3);

% Programm start
% -----------------------------------------------------------
for i=2:simulation_time
    % Calculate g with respect ot NSSK
%     g = getcurrentg(200, phi, teta, gamma); % Сюда должны передавать h
    gn = quatrotate(L, g);

    % Emulate IMU noise
    Ax_b = normrnd(Ax_err(psi_err, teta_err, gamma_err), aSigma) * Ax_scale_f / Ax_true_scale_f + Ax_true_bias - Ax_z_bias;
    Ay_b = normrnd(Ay_err(psi_err, teta_err, gamma_err), aSigma) * Ay_scale_f / Ay_true_scale_f + Ay_true_bias - Ay_z_bias;
    Az_b = normrnd(Az_err(psi_err, teta_err, gamma_err), aSigma) * Az_scale_f / Az_true_scale_f + Az_true_bias - Az_z_bias;
     
    Ab = [Ax_b; Ay_b; Az_b];
    
    Fx_b = normrnd(Ux_err(psi_err, teta_err, gamma_err) * dt, wSigma);
    Fy_b = normrnd(Uy_err(psi_err, teta_err, gamma_err) * dt, wSigma);
    Fz_b = normrnd(Uz_err(psi_err, teta_err, gamma_err) * dt, wSigma);
    
    Fb = [Fx_b; Fy_b; Fz_b];
    
    % Gyros compensation
    % ----
    
    % Axels compensation
    % ----
    
    % Recalculate G quat
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

    gamma(i)  = gamma(1) + atan2(M(3,2), m0);
    teta(i)   = teta(1) + atan2(M(1,2),M(2,2));
    psi(i)    = psi(1) - atan2(M(3,1),M(3,3));

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
    
    if (nir_calibration)
        
    end
    
    % Sculling compensation
%     Vx_n(i) = Vx_n(i-1) + (An(1) + Vy_n(i-1) * Fn(3) - Vz_n(i-1) * Fn(2) - gn(1)) * dt;
%     Vy_n(i) = Vy_n(i-1) + (An(2) + Vz_n(i-1) * Fn(1) - Vx_n(i-1) * Fn(3) - gn(2)) * dt;
%     Vz_n(i) = Vz_n(i-1) + (An(3) + Vy_n(i-1) * Fn(1) - Vx_n(i-1) * Fn(2) - gn(3)) * dt;
end

t=1:simulation_time * dt;
font_size = 25;
font_size_sub = 15;

solution_folder_name = join(['./solutions/', datestr(datetime('now'), "yyyy-mm-dd_HH-MM-SS")]);
mkdir(solution_folder_name);

fe = fopen(join([solution_folder_name, '/errors.txt']), 'w+');

fprintf(fe, "%s\n", "Вектор g, для проверки ухода по скоростям:");
fprintf(fe, 'Ax - gx: %6f\nAy - gy: %6f\nAz - gz: %6f\n\n', An(1) - gn(1), An(2) - gn(2), An(3) - gn(3));

fprintf(fe, 'gx * %f = %6f\ngy * %f = %6f\ngz * %f = %6f\n\n', simulation_time * dt, (An(1) - gn(1)) * simulation_time * dt, simulation_time * dt, (An(2) - gn(2)) * simulation_time * dt, simulation_time * dt, (An(3) - gn(3)) * simulation_time * dt);

fprintf(fe, "%s\n", "Ошибка алгоритма БИНС по скоростям:");
fprintf(fe, "Vx: |%6f - %6f| = %6f [м/с^2]\n", 0, Vx_n(simulation_time), abs(0 - Vx_n(simulation_time)));
fprintf(fe, "Vy: |%6f - %6f| = %6f [м/с^2]\n", 0, Vy_n(simulation_time), abs(0 - Vy_n(simulation_time)));
fprintf(fe, "Vz: |%6f - %6f| = %6f [м/с^2]\n\n", 0, Vz_n(simulation_time), abs(0 - Vz_n(simulation_time)));

fprintf(fe, "%s\n", "Ошибка алгоритма БИНС по углам:");
fprintf(fe, "psi:   |%6f - %6f| = %6f [град]\n", U * sin(phi) * simulation_time * dt * 57.3, psi(simulation_time) * 57.3, abs(U * sin(phi) * simulation_time * dt * 57.3 - psi(simulation_time) * 57.3));
fprintf(fe, "teta:  |%6f - %6f| = %6f [град]\n", 0, teta(simulation_time) * 57.3, abs(0 - teta(simulation_time) * 57.3));
fprintf(fe, "gamma: |%6f - %6f| = %6f [град]\n\n", U * cos(phi) * simulation_time * dt * 57.3, gamma(simulation_time) * 57.3, abs(U * cos(phi) * simulation_time * dt * 57.3 - gamma(simulation_time) * 57.3));
    
if (debug)
    subplot(2, 2, 1);
    plot(t, tmp3(1/dt:1/dt:end));
    set(gca, 'fontsize', font_size);
    % set(gca, 'XLim', [1, simulation_time * dt]);
    title('tmp1');
    
    grid on;

    subplot(2, 2, 2);
    plot(t, tmp2(1/dt:1/dt:end));
    set(gca, 'fontsize', font_size);
    % set(gca, 'XLim', [1, simulation_time * dt]);
    title('tmp2');
    
    grid on;

    subplot(2, 2, 3);
    plot(t, tmp3(1/dt:1/dt:end));
    set(gca, 'fontsize', font_size);
    % set(gca, 'XLim', [1, simulation_time * dt]);
    title('tmp3');
    
    grid on;
    
    subplot(2, 2, 4);
    plot(t, h(1/dt:1/dt:end));
    set(gca, 'fontsize', font_size);
    % set(gca, 'XLim', [1, simulation_time * dt]);
    title('h');
    
    grid on;
    
    orv = [gamma(simulation_time); psi(simulation_time); teta(simulation_time)];
    orvg = [gamma(simulation_time) * 57.3; psi(simulation_time) * 57.3; teta(simulation_time) * 57.3];
    Vn = [Vx_n(simulation_time); Vy_n(simulation_time); Vz_n(simulation_time)];
elseif (single_plot)
    figure(1);
    plot(t, Vx_n(1/dt:1/dt:end));
    set(gca, 'fontsize', font_size);
    % set(gca, 'XLim', [1, simulation_time * dt]);
    set(gcf, 'Position', get(0, 'Screensize'));
    title('Vx');
    xlabel('Время, с');
    ylabel('Линейная скорость, м/с');
    grid on;
    saveas(gcf, join([solution_folder_name, '/Vx.jpg']));

    figure(2);
    plot(t, Vy_n(1/dt:1/dt:end));
    set(gca, 'fontsize', font_size);
    % set(gca, 'XLim', [1, simulation_time * dt]);
    set(gcf, 'Position', get(0, 'Screensize'));
    title('Vy');
    xlabel('Время, с');
    ylabel('Линейная скорость, м/с');
    grid on;
    saveas(gcf, join([solution_folder_name, '/Vy.jpg']));

    figure(3);
    plot(t, Vz_n(1/dt:1/dt:end));
    set(gca, 'fontsize', font_size);
    % set(gca, 'XLim', [1, simulation_time * dt]);
    set(gcf, 'Position', get(0, 'Screensize'));
    title('Vz');
    xlabel('Время, с');
    ylabel('Линейная скорость, м/с');
    grid on;
    saveas(gcf, join([solution_folder_name, '/Vz.jpg']));

    figure(4);
    plot(t, gamma(1/dt:1/dt:end));
    set(gca, 'fontsize', font_size);
    % set(gca, 'XLim', [1, simulation_time * dt]);
    set(gcf, 'Position', get(0, 'Screensize'));
    title('gamma (X)');
    xlabel('Время, с');
    ylabel('Угол, рад');
    grid on;
    saveas(gcf, join([solution_folder_name, '/gamma.jpg']));

    figure(5);
    plot(t, psi(1/dt:1/dt:end));
    set(gca, 'fontsize', font_size);
    % set(gca, 'XLim', [1, simulation_time * dt]);
    set(gcf, 'Position', get(0, 'Screensize'));
    title('psi (Y)');
    xlabel('Время, с');
    ylabel('Угол, рад');
    grid on;
    saveas(gcf, join([solution_folder_name, '/psi.jpg']));

    figure(6);
    plot(t, teta(1/dt:1/dt:end));
    set(gca, 'fontsize', font_size);
    % set(gca, 'XLim', [1, simulation_time * dt]);
    set(gcf, 'Position', get(0, 'Screensize'));
    title('teta (Z)');
    xlabel('Время, с');
    ylabel('Угол, рад');
    grid on;
    saveas(gcf, join([solution_folder_name, '/teta.jpg']));
    
    figure(7);
    plot(t, h(1/dt:1/dt:end));
    set(gca, 'FontSize', font_size);
    % set(gca, 'XLim', [1, simulation_time * dt]);
    set(gcf, 'Position', get(0, 'Screensize'));
    title('h');
    xlabel('Время, с');
    ylabel('Высота, м');
    grid on;
    saveas(gcf, join([solution_folder_name, '/h.jpg']));
else
    subplot(2, 3, 1);
    plot(t, Vx_n(1/dt:1/dt:end));
    set(gca, 'fontsize', font_size_sub);
    % set(gca, 'XLim', [1, simulation_time * dt]);
    title('Vx');
    xlabel('Время, с');
    ylabel('Линейная скорость, м/с');
    
    grid on;

    subplot(2, 3, 2);
    plot(t, Vy_n(1/dt:1/dt:end));
    set(gca, 'fontsize', font_size_sub);
    % set(gca, 'XLim', [1, simulation_time * dt]);
    title('Vy');
    xlabel('Время, с');
    ylabel('Линейная скорость, м/с');
    
    grid on;

    subplot(2, 3, 3);
    plot(t, Vz_n(1/dt:1/dt:end));
    set(gca, 'fontsize', font_size_sub);
    % set(gca, 'XLim', [1, simulation_time * dt]);
    title('Vz');
    xlabel('Время, с');
    ylabel('Линейная скорость, м/с');
    
    grid on;

    subplot(2, 3, 4);
    plot(t, gamma(1/dt:1/dt:end));
    set(gca, 'fontsize', font_size_sub);
    % set(gca, 'XLim', [1, simulation_time * dt]);
    title('gamma (X)');
    xlabel('Время, с');
    ylabel('Угол, рад');
    
    grid on;

    subplot(2, 3, 5);
    plot(t, psi(1/dt:1/dt:end));
    set(gca, 'fontsize', font_size_sub);
    % set(gca, 'XLim', [1, simulation_time * dt]);
    title('psi (Y)');
    xlabel('Время, с');
    ylabel('Угол, рад');
    grid on;

    subplot(2, 3, 6);
    plot(t, teta(1/dt:1/dt:end));
    set(gca, 'fontsize', font_size_sub);
    % set(gca, 'XLim', [1, simulation_time * dt]);
    title('teta (Z)');
    xlabel('Время, с');
    ylabel('Угол, рад');
    
    grid on;
end