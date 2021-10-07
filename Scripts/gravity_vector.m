%-- Construcción del vector de gravedad
% Parámetros:
%       - Vector de masas [6x1]
%       - Matriz de Jacobianos del centro de masa. Siendo i el renglón de
%         la matriz del centro de masa i. 
%       - Vector de gravedad g_0 en coordenadas del referencial inercial
%         [6x1].

function g_q = gravity_vector(m, J_cm, g_0)
    n = length(m);
    g_q = sym(zeros(n,3));
    for i = 1:n
        J_cm_i = reshape(J_cm(i,:), 6, n);
        Jv_cm_i = J_cm_i(1:3,:);
        g_q = g_q + m(i)*(Jv_cm_i');
    end
    g_q = simplify(g_q*(-g_0));
end

