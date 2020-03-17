function g = getcurrentg(h, phi)
    g_curr = 9.780318 * (1 + 0.005302 * (sin(phi)) ^ 2 - 0.000006 * (sin(2 * phi)) ^ 2) - 0.000003086 * h;
    
    g = [0, g_curr, 0];
%     g = [g_curr * sin(teta / 57.3), g_curr * cos(teta / 57.3) * cos(gamma / 57.3), - g_curr * cos(teta / 57.3) * sin(gamma / 57.3)];
end