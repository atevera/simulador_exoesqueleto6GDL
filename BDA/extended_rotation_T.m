%-- Construcción del vector extendido de rotación transpuesto 'R^T'
% Parámetros:
%       - Matriz de rotación [Matriz 6x9]
%         -> Cada renglón corresponde a cada eslabón
% Salida: 
%       - Vector extendido de rotación transpuesto [Matriz 6x36]
%         -> Cada renglón corresponde a cada eslabón
function R_R = extended_rotation_T(R)
    n = size(R,1);
    R_R = sym(zeros(n,6*n));
    for i = 1:n
        R_i = reshape(R(i,:),3,3);
        R_R_temp = [R_i zeros(3); zeros(3) R_i]';
        R_R(i,:) = reshape(R_R_temp,1,6*n);
    end
end

