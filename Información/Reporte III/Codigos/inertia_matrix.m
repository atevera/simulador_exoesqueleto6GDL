function H = inertia_matrix(mass,J_cm, FK, IT)
    N = length(mass);
    H = zeros(N);
    for i  = 1:N
        J_cm_i = reshape(J_cm(i,:), 6, N);
        Jv_cm_i = J_cm_i(1:3,:);
        Jw_cm_i = J_cm_i(4:6,:);
        FK_i = reshape(FK(i,:), 4, 4);
        IT_i = reshape(IT(i,:), 3, 3);
        rotmatrix0_i = FK_i(1:3,1:3);
        linear = mass(i)*(Jv_cm_i)'*Jv_cm_i;
        angular = (Jw_cm_i')*(rotmatrix0_i)*(IT_i)*(rotmatrix0_i')*(Jw_cm_i);
        H = H + linear + angular;
    end
end