n = [0, 0, 6789744];
g_prev = [3376736, 2522995, 5313581];
r = [3382551, 2526410, 5317539];

% min = abs(r - [3.380170e+06, 2.525518e+06, 5.319477e+06]);
% angls = [1,1,1];
% 
% for i=-23:1e-4:-22
%     for j=-33:1e-4:-32
%         for k=21:1e-4:22
%             p = i / 57.3;
%             g = j / 57.3;
%             t = k  / 57.3;
% 
%             P = [cos(p/2), 0, 0, sin(p/2)];
%             Q = [cos(t/2), sin(t/2), 0, 0];
%             R = [cos(g/2), 0, sin(g/2), 0];
%             G = quatmultiply(quatmultiply(P, Q), R);
% 
%             g_rotated = quatrotate(G, n);
%             diff = abs(r - g_rotated);
% 
%             if ((diff < min) == [1,1,1])
%                 min = diff;
%                 angls = [p, g, t];
%             end
%         end
%         fprintf("j: %d\n", j);
%     end
%     fprintf("\n\ni: %d\n\n", i);
% end
% 
disp(min)
disp(angls)

p = - 22.3184 / 57.3;
g = - 32.4356 / 57.3;
t = 21.8382  / 57.3;

P = [cos(p/2), 0, 0, sin(p/2)];
Q = [cos(t/2), sin(t/2), 0, 0];
R = [cos(g/2), 0, sin(g/2), 0];
G = quatmultiply(quatmultiply(P, Q), R);

g_1 = quatrotate(G, n);
g_rotated = quatrotate([cos(obj.settings.U/2), 0,0, -sin(obj.settings.U/2)], g_1);
lla = ecef2lla(g_rotated);
fprintf("la:  %.4d\nphi: %.4d\nr:   %.4d\n\n", lla(2),lla(1),lla(3));
fprintf("la diff:  %d\nphi diff: %d\nr diff:   %d\n", obj.la_real(1) - lla(2), obj.fi_real(1) - lla(1), obj.R_real(1) - lla(3));