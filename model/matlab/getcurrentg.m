function g = getcurrentg(h, phi)
    g_curr = 9.780318 * (1 + 0.005302 * (sin(phi)) ^ 2 - 0.000006 * (sin(2 * phi)) ^ 2) - 0.000003086 * h;
    g = [0, g_curr, 0];
end