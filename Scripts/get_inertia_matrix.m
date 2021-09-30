function H = get_inertia_matrix(mass,J_cm, forward_kinematics, tensor)
    n = length(mass);
    for i  = 1:n
        Jv_cm_i = J_cm(1:3, i);
        Jw_cm_i = J_cm(4:6, i);
        rotmatrix0_i = forward_kinematics{i}(1:3,1:3);
        linear = mass(i)*(Jv_cm_i)'*Jv_cm_i;
        angular = (Jw_cm_i)'(rotmatrix0_i)*(tensor{i})*(rotmatrix0_i)'*(Jw_cm_i)
        H = linear + angular;
    end
        H = (1/2)*H;
end