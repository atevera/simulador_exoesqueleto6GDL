%-- Construcción de las matrices de transformación homogénea
% Parámetros:
%       - Lista de vectores de traslación entre cada referencial.
%       - Lista de vectores directores de rotación lambda.
%       - Array de coordenadas generalizadas. 

function homogeneous_matrix = local_homogeneous_transform_matrix(translation, lambda, q) 
    num_ref = size(lambda, 2);
    homogeneous_matrix = zeros(num_ref,16);
    for i = 1:num_ref
        rotation_matrix = expm(OPC(lambda(:,i)*q(i)));
        hm_i = [rotation_matrix translation(:,i); 0 0 0 1];
        homogeneous_matrix(i,:) = reshape(hm_i,1,16);
    end
end
