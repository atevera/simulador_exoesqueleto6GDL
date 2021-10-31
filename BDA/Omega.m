%-- Operador 6D Omega
% Parámetros:
%       - Vector de rotación eje/ángulo [Vector]
% Salida: 
%       - Evaluación de la función [Matriz de 6x6]
% Válido sólo para articulaciones tipo revoluta. 
function Omega = Omega(lambda)
    lamda_OPC = OPC(lambda);
    Omega = [lamda_OPC zeros(3); zeros(3) lamda_OPC];
end

