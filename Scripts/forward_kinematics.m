%-- Cálculo de la cinemática directa para cada referencial mediante
%   la multiplicación de matrices de transformación homogénea.
% Parámetros:
%       - Lista de referenciales (no inerciales o bien de
%         de los centros de masa de cada eslabón). 

function ht_matrix = forward_kinematics(referencial)
    num_ref = size(referencial,1);
    ht_matrix = sym(zeros(num_ref, 16));
    ht_matrix(1,:) = referencial(1,:);
    for i = 2:num_ref
        ht_prev = reshape(ht_matrix(i-1,:),4,4);
        ht_ref = reshape(referencial(i,:),4,4);
        ht_matrix(i,:) = reshape(ht_prev*ht_ref,1,16);
    end
    ht_matrix = simplify(ht_matrix);
end