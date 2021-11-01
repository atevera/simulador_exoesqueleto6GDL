%-- Construcción del vector extendido de gravedad
% Parámetros:
%       - Vector de aceleración de la gravedad en coordenadas inerciales
%         [Vector 3x1]
% Salida: 
%       - Vector extendido de gravedad [Vector 6x1]
function G_0 = extended_gravity(g_0)
    G_0 = [g_0; zeros(3,1)];
end

