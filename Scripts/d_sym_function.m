%-- Evaluación del vector de disipación d para simulación
% Parámetros:
%       - dq:    Vector de velocidades generalizadas dq
%       - file: Nombre del archivo [tipo: string]
function d_eval = d_sym_function(dq, file)
    d_file = load(file);
    d_eval = double(d_file.tau_d(dq(1),dq(2),dq(3),dq(4),dq(5),dq(6)));
end