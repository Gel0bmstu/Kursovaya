figure();
plot(1:5820, obj.gamma, 1:5820, obj.gamma_ad, 1:5820, obj.gamma_k);
legend(['gamma'], ['ad'], ['k'])
title('gamma');
grid on;