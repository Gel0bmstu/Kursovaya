% arr  = zeros(length(obj.Vn), 3);
% f = fopen(obj.settings.path_to_linear_accelerations_file, 'w');
% for i = 2:length(obj.Vn)
%     arr(i, :) = (obj.Vn(i, :) - obj.Vn(i-1, :));
%     fprintf(f, "%d %d %d\n", arr);
% end

subplot(3,1,1)
plot(1:length(diff(:, 1)), diff(:, 1))
grid on;

subplot(3,1,2)
plot(1:length(diff(:, 1)), diff(:, 2))
grid on;

subplot(3,1,3)
plot(1:length(diff(:, 1)), diff(:, 3))
grid on;