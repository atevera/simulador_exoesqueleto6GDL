%-- Construcción de la matriz de inercia considerando la iteración de i=1:N
% Parámetros:
%       - Vector de masas de cada eslabón.
%       - Jacobiano Geométrico de los centros de masa.
%       - Cinemática Directa de cada centro de masa [Lista].
%       - Tensor de inercia de cada centro de masa [Lista].

function H = inertia_matrix(mass,J_cm, forward_kinematics, tensor)
    N = length(mass);
    H = zeros(N);
    for i  = 1:N
        Jv_cm_i = J_cm{i}(1:3,:);
        Jw_cm_i = J_cm{i}(4:6,:);
        rotmatrix0_i = forward_kinematics{i}(1:3,1:3);
        linear = mass(i)*(Jv_cm_i)'*Jv_cm_i;
        angular = (Jw_cm_i')*(rotmatrix0_i)*(tensor{i})*(rotmatrix0_i')*(Jw_cm_i);
        H = H + linear + angular;
    end
        H = (1/2)*H;
end