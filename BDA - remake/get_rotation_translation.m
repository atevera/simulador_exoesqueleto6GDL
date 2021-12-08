%-- Adquiere la matriz de rotación y vector de traslación de la matriz 
%   de transformación homogénea de cada eslabón. 
% Parámetros:
%       - Matriz de transformación homogénea A [Matriz 6x16]
%         -> Cada renglón corresponde a cada eslabón.
% Salida: 
%       - Matriz de rotación   [Matriz 6x9]
%       - Matriz de traslación [Matriz 6x3]
%         -> Cada renglón corresponde a cada eslabón para cada caso. 
function [R, d] = get_rotation_translation(A)
    n = size(A,1);
    R = sym(zeros(n,9));
    d = sym(zeros(n,3));
    for i = 1:n
        A_i = reshape(A(i,:),4,4);
        R_i = A_i(1:3,1:3);
        d_i = A_i(1:3,4);
        R(i,:) = reshape(R_i,1,9);
        d(i,:) = reshape(d_i,1,3);
    end
end

