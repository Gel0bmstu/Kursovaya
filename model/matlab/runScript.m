clear all;

% Set initial conditions

% -----------------------------------------------------------
% Default vals
% -----------------------------------------------------------
settings = ModelSettings(93*60, 1);
disp('Algorithm setting:');
settings = settings.set_debug_mode_flag(true);
settings = settings.set_display_solutions_flag(true);
settings = settings.set_subplot_print_flag(true);
settings = settings.set_log_algorithm_solutions_flag(true);
disp('Algorithm configured successfully.');

bins = Bins(settings);
bins = bins.run();

figure(1);
subplot(1, 2, 1);
plot(1:bins.settings.simulation_time, bins.R_real);
title('Distance from the center of the Earth to the spacecraft')
xlabel('time, [sec]');
ylabel('R, [m]');
grid on;
subplot(1, 2, 2);
plot(1:bins.settings.simulation_time, bins.vx_real);
title('Linear speed of the spacecraft')
xlabel('time, [sec]');
ylabel('Linear vel (X axis), [m/sec]');
grid on;


% -----------------------------------------------------------
% Start SINS params
% -----------------------------------------------------------

% Errors
% -----------------------------------------------------------
table_error = 1e-5; % Ошибка поворотного стола [рад]
axis_alligment_error = 1e-5; % Ошибка соосности осей БЧЭ и ССК [рад]

% -----------------------------------------------------------
% Ax_err = @(psi, teta, gamma) gc * sin(teta_err / 57.3);
% Ay_err = @(psi, teta, gamma) gc * cos(teta_err / 57.3) * cos(gamma / 57.3);
% Az_err = @(psi, teta, gamma) - gc * cos(teta_err / 57.3) * sin(gamma / 57.3);
% 
% Ux_err = @(psi, teta, gamma) U * cos(phi) * cos(psi / 57.3) + U * sin(phi) * sin(teta / 57.3);
% Uy_err = @(psi, teta, gamma) U * cos(phi) * (-cos(gamma / 57.3) * cos(psi / 57.3) * sin(teta / 57.3) + sin(gamma / 57.3) * sin(psi / 57.3)) + U * sin(phi) * cos(gamma / 57.3) * cos(teta / 57.3);
% Uz_err = @(psi, teta, gamma) U * cos(phi) * (sin(gamma / 57.3) * cos(psi / 57.3) * sin(teta / 57.3) + sin(gamma / 57.3) * sin(psi / 57.3)) + U * sin(phi) * sin(gamma / 57.3) * cos(teta / 57.3);

% Programm start
% -----------------------------------------------------------
% for i=2:simulation_time
%     % Calculate g with respect ot NSSK
% %     g = getcurrentg(200, phi, teta, gamma); % Сюда должны передавать h
%     gn = quatrotate(L, g);
% 
%     % Emulate IMU noise
%     Ax_b = normrnd(Ax_err(psi_err, teta_err, gamma_err), aSigma) * Ax_scale_f / Ax_true_scale_f + Ax_true_bias - Ax_z_bias;
%     Ay_b = normrnd(Ay_err(psi_err, teta_err, gamma_err), aSigma) * Ay_scale_f / Ay_true_scale_f + Ay_true_bias - Ay_z_bias;
%     Az_b = normrnd(Az_err(psi_err, teta_err, gamma_err), aSigma) * Az_scale_f / Az_true_scale_f + Az_true_bias - Az_z_bias;
%      
%     Ab = [Ax_b; Ay_b; Az_b];
%     
%     Fx_b = normrnd(Ux_err(psi_err, teta_err, gamma_err) * dt, wSigma);
%     Fy_b = normrnd(Uy_err(psi_err, teta_err, gamma_err) * dt, wSigma);
%     Fz_b = normrnd(Uz_err(psi_err, teta_err, gamma_err) * dt, wSigma);
%     
%     Fb = [Fx_b; Fy_b; Fz_b];
%     
%     % Gyros compensation
%     % ----
%     
%     % Axels compensation
%     % ----
%     
%     % Recalculate G quat
%     A = make4SkewMatrix([U * cos(phi) * dt; U * sin(phi) * dt; 0]);
%     dG = E + 0.5 * A + 0.25 * A^2;
%     G = (dG * G')';
%     
%     % Recalculate main quat
%     A = make4SkewMatrix(Fb);
%     dL = E + 0.5 * A + 0.25 * A^2;
%     L = (dL * L')';
%         
%     % Project IMU info to NSSK
%     An = quatrotate(L, Ab');
%     Fn = quatrotate(L, Fb');
% 
%     M = quat2rotm(L);
%     
%     m0 = sqrt(M(3,1)^2 + M(3,3)^2);
% 
%     gamma(i)  = gamma(1) + atan2(M(3,2), m0);
%     teta(i)   = teta(1)  + atan2(M(1,2),M(2,2));
%     psi(i)    = psi(1)   - atan2(M(3,1),M(3,3));
% 
%     Vx_n(i) = Vx_n(i-1) + (An(1) - gn(1)) * dt;
%     Vy_n(i) = Vy_n(i-1) + (An(2) - gn(2)) * dt;
%     Vz_n(i) = Vz_n(i-1) + (An(3) - gn(3)) * dt;
% 
%     h(i) = h(i-1) + Vy_n(i) * dt;
% 
%     % Sculling compensation
% %     Vx_n(i) = Vx_n(i-1) + (An(1) + Vy_n(i-1) * Fn(3) - Vz_n(i-1) * Fn(2) - gn(1)) * dt;
% %     Vy_n(i) = Vy_n(i-1) + (An(2) + Vz_n(i-1) * Fn(1) - Vx_n(i-1) * Fn(3) - gn(2)) * dt;
% %     Vz_n(i) = Vz_n(i-1) + (An(3) + Vy_n(i-1) * Fn(1) - Vx_n(i-1) * Fn(2) - gn(3)) * dt;
% end
