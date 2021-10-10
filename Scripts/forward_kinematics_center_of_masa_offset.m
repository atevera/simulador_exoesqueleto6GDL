%-- Traslación del referencial del eslabón al referencial virtual del
%   centro de masa mediante su matriz de transformación homogénea. 
% Parámetros:
%       - Lista de cinemática directa de cada referencial.
%       - Lista de centros de masa de cada eslabón en coordenadas del
%         referencial local xyz.
function HM_CM0_6 = forward_kinematics_center_of_masa_offset(forward_kinematics, offset)
    num_cm = size(offset, 2);
    HM_CM0_6 = sym(zeros(num_cm, 16));
    I = eye(3);
    for i = 1:num_cm
        HM_CM0_i = reshape(forward_kinematics(i,:),4,4)*[I offset(:,i); 0 0 0 1]; 
        HM_CM0_6(i,:) = reshape(HM_CM0_i, 1, 16);
    end
    HM_CM0_6 = simplify(HM_CM0_6);
end