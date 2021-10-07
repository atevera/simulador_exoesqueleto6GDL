%-- Evaluación del vector de gravedad g para simulación
% Parámetros:
%       - q:    Vector de coordenadas generalizadas q
%       - file: Nombre del archivo [tipo: string]
function g_eval = g_sym_function(q, file)
    g_file = load(file);
    g_eval = double(g_file.g(q(1),q(2),q(3),q(4),q(5),q(6)));
end

