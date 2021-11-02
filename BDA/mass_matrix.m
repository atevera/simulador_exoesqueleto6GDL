%-- Construcción de la matriz de masas de cada eslabón. 
% Parámetros:
%       - Masa de cada eslabón m(i) [Vector 6x1]
%       - Centro de masa en coordenadas locales r_c [Matriz 3x6]
%         -> Cada columna corresponde a cada eslabón
%       - Tensores de Inercia [Matriz 6x36]
%         -> Cada renglón corresponde a cada eslabón 
% Salida: 
%       - Matriz de masas [Matriz 6x36]
%         -> Cada renglón corresponde a cada eslabón 
function M = mass_matrix(m, r_c, tensor)
    n = length(m);
    M = sym(zeros(n,36));
    for i = 1:n
        %rc_x = OPC(r_c(:,i));
        tensor_i = reshape(tensor(i,:),3,3);
        M_i = [m(i)*eye(3) zeros(3); zeros(3) tensor_i];
        M(i,:) = reshape(M_i, 1,36);
    end
end

