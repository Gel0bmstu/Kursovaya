% arr  = razeros(length(obj.Vn), 3);
% f = fopen(obj.settings.path_to_linear_accelerations_file, 'w');
% for i = 2:length(obj.Vn)
%     arr(i, :) = (obj.Vn(i, :) - obj.Vn(i-1, :));
%     fprintf(f, "%d %d %d\n", arr);
% end
% 
% figure(); 
% plot(1:5820, obj.gamma_real); 
% hold on;
% plot(1:5820, obj.gamma);
% title('gamma')
% legend('real', 'calculated')
% grid on;
% 
% figure(); 
% plot(1:5820, obj.teta_real); 
% hold on;
% plot(1:5820, obj.teta);
% title('theta')
% legend('real', 'calculated')
% grid on;
% 
% figure(); 
% plot(1:5820, obj.psi_real); 
% hold on;
% plot(1:5820, obj.psi);
% title('psi')
% legend('real', 'calculated')
% grid on;


a = obj.Sr(:, 1) - obj.Sg(:, 1);
% 
figure(); 
plot(1:98, obj.Sx_g);
hold on;
plot(1:98, obj.Sx_r);
hold off;
legend('Calculated', 'Real');
grid on;

figure(); 
plot(1:98, a)
grid on ;
% for i=1:3
%     figure()
%     plot(data_{i});
%     grid on;
%     title(title_{i});
%     xlabel('iter, [n]');
%     ylabel(ylabel_{i});
%     saveas(gcf, join([obj.settings.solution_folder_name, '/', title_{i}, '.png']));
% end
% 
% title_ = {'Vx_g error', 'Vy_g error', 'Vz_g error'};
% ylabel_ = {'Vx_g, [m/s]', 'Vy_g, [m/s]', 'Vz_g, [m/s]'};
% data_ = {obj.Vr(:, 1) - obj.Vg(:, 1), obj.Vr(:, 2) - obj.Vg(:, 2), obj.Vr(:, 3) - obj.Vg(:, 3)};
% for i=1:3
%     figure()
%     plot(data_{i});
%     grid on;
%     title(title_{i});
%     xlabel('iter, [n]');
%     ylabel(ylabel_{i});
%     saveas(gcf, join([obj.settings.solution_folder_name, '/', title_{i}, '.png']));
% end

% a = obj.la_real - obj.la;
% a(43) = a(42) + (a(44) - a(42)) / 2;
% title_ = {'La error', 'Phi error', 'H error'};
% ylabel_ = {'La, [deg]', 'Phi, [deg]', 'H, [m]'};
% data_ = {a, obj.fi_real - obj.phi, obj.R_real - obj.h};
% for i=1:3
%     figure()
%     plot(data_{i});
%     grid on;
%     title(title_{i});
%     xlabel('time, [min]');
%     ylabel(ylabel_{i});
%     saveas(gcf, join([obj.settings.solution_folder_name, '/', title_{i}, '.png']));
% end

% figure(); 
% plot(1:98, a(:, 1));
% grid on;
