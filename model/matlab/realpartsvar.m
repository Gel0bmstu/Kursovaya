% Function return array object of real quaternoin q parts
%
% For example: - parts() retun [q0, q1i, q2j, q3k];
%              - realparts() return pure [q0, q1, q2, q3]
%                without Image parts;
%              - realpartsvar() return array object
%                q = [q0, q1, q2, q3].
function q = realpartsvar(q) 
    [q0i, q1i, q2i, q3i] = parts(q);
    q0 = real(q0i);
    q1 = real(q1i);
    q2 = real(q2i);
    q3 = real(q3i);
    
    q = [q0, q1, q2, q3];
end