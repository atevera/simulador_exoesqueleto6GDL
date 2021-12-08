%-- Construcción del vector extendido Lambda 
% Parámetros:
%       - Vector de rotación eje/ángulo [Vector 3x1]
% Salida: 
%       - Vector extendido Lambda [Matriz 6x36]
%         -> Cada renglón corresponde a cada eslabón 

% function LAMBDA = lambda_extended(rotation_vector)
%     n = size(rotation_vector, 2);
%     LAMBDA = zeros(n,6*n);
%     for i = 1:n
%         lambda_temp = zeros(6,6);
%         lambda_temp(:,i) = [zeros(3,1); rotation_vector(:,i)];
%         LAMBDA(i,:) = reshape(lambda_temp,1,6*n);
%     end
% end

function LAMBDA = lambda_extended(lambda_vector)
    n = size(lambda_vector, 2);
    LAMBDA = zeros(n,6*n);
    for i = 1:n
        lambda_temp = zeros(6,n);
        lambda_temp(:,i) = lambda_vector(:,i);
        LAMBDA(i,:) = reshape(lambda_temp,1,6*n);
    end
end

