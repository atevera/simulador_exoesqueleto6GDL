%-- Evaluación del vector de Coriolis C para simulación
% Parámetros:
%       - q:    Vector de coordenadas generalizadas q
%       - dq:   Vector de velocidades generalizadas dq
%       - file: Nombre del archivo [tipo: string]
function C_eval = C_sym_function(q, dq, file)
    C_file = load(file);
    C_eval = double(C_file.c(q(1),q(2),q(3),q(4),q(5),q(6),dq(1),dq(2),dq(3),dq(4),dq(5),dq(6)));
end

