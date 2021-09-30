%-- Construcción de las matrices de transformación homogénea
% Parámetros:
%       - Lista de vectores de traslación entre cada referencial.
%       - Lista de vectores directores de rotación lambda.
%       - Array de coordenadas generalizadas. 

function homogeneous_matrix = homogeneous_transform_matrix(translation, lambda, q) 
    num_ref = length(lambda);
    homogeneous_matrix = cell(1, num_ref);
    % rotation_matrix =cell(3, num_ref); 
    %Segun yo no necesitas declarar un
    % espacio para un array de estas matrices, porque de cualquier manera
    % solo la usas una vez en el cálculo de la matriz de rotacion, cambia
    % con cada iteracion y nunca sale de esta funcion. Mejor solo dejarla
    % como una variable local.
    for i = 1:num_ref
        rotation_matrix = expm(OPC(lambda{i}*q(i)));
        homogeneous_matrix{i} = [rotation_matrix translation{i}'; 0 0 0 1];
    end
end
