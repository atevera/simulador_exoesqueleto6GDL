%-- Cálculo de la cinemática directa para cada referencial mediante
%   la multiplicación de matrices de transformación homogénea.
% Parámetros:
%       - Lista de referenciales (no inerciales o bien de
%         de los centros de masa de cada eslabón). 

function A = forward_kinematics(A_i)
    num_ref = size(A_i,1);
    A = sym(zeros(num_ref, 16));
    A(1,:) = A_i(1,:);
    for i = 2:num_ref
        ht_prev = reshape(A(i-1,:),4,4);
        ht_ref = reshape(A_i(i,:),4,4);
        A(i,:) = reshape(ht_prev*ht_ref,1,16);
    end
    A = simplify(A);
end