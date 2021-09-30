%-- Cálculo de la cinemática directa para cada referencial mediante
%   la multiplicación de matrices de transformación homogénea.
% Parámetros:
%       - Lista de referenciales (no inerciales o bien de
%         de los centros de masa de cada eslabón). 

function ht_matrix = forward_kinematics(referencial)
    num_ref = length(referencial);
    ht_matrix = cell(1,num_ref);
    ht_matrix{1} = referencial{1};
    for i = 2:num_ref
        ht_matrix{i} = ht_matrix{i-1}*referencial{i};
    end
end