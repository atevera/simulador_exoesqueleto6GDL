%-- Evaluación de la matriz de inercia H para simulación
% Parámetros:
%       - q:    Vector de coordenadas generalizadas q
%       - file: Nombre del archivo [tipo: string]
function H_eval = H_sym_function(q, file)
    H_file = load(file);
    H_eval = double(H_file.tau_h(q(1),q(2),q(3),q(4),q(5),q(6)));
end
