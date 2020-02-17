clear all;

phi = 60 / 57.3;
U = [0, 5, 0];

U_ok = [U(2) * cos(phi), U(2) * sin(phi), 0]

rotation = [cos(30/57.3/2), 0, 0, sin(30/57.3/2)];

q = quaternion(rotation);
res = quatrotate(rotation, U)