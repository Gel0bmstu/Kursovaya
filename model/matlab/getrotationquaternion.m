% Function return array of rotation quaternoin elements
function R = getrotationquaternion(psi, teta, gamma, U, dt, phi) 

    Fx_nssk = psi - U * cos(phi) * dt;
    Fy_nssk = teta - U * sin(phi) * dt;
    Fz_nssk = gamma;
    
    psiRotateQuaternion = [cos(Fx_nssk/2), 0, sin(Fx_nssk/2), 0];
    tetaRotateQuaternion = [cos(Fy_nssk/2), 0, 0, sin(Fy_nssk/2)];
    gammaRotateQuaternion = [cos(Fz_nssk/2), sin(Fz_nssk/2), 0, 0];

    R = quatmultiply(psiRotateQuaternion, quatmultiply(tetaRotateQuaternion, gammaRotateQuaternion));
    
%     R = realpartsvar(rotationQuaternion);
end