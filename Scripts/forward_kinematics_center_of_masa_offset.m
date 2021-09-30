%-- Traslación del referencial del eslabón al referencial virtual del
%   centro de masa mediante su matriz de transformación homogénea. 
% Parámetros:
%       - Lista de cinemática directa de cada referencial.
%       - Lista de centros de masa de cada eslabón en coordenadas del
%         referencial local xyz.
function forward_kinematics_mass_center = forward_kinematics_center_of_masa_offset(forward_kinematics, offset)
    num_cm = length(offset);
    forward_kinematics_mass_center = cell(1,num_cm);
    I = eye(3);
    for i = 1:num_cm
        forward_kinematics_mass_center{i} = forward_kinematics{i}*[I offset{i}'; 0 0 0 1]; 
    end
end