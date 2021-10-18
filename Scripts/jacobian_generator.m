function J = jacobian_generator(MTH, LAMBDA_R)
    n = size(LAMBDA_R,2);
    J = sym(zeros(n,(6*n)));
    j_w = sym(zeros(3, n));
    for i = 1:n
        j_i = sym(zeros(6, n));
        j_v = sym(zeros(3, n));
        
        mth0_i = reshape(MTH(i,:), 4, 4);
        mth_r_i = mth0_i(1:3,1:3);
        d_p = mth0_i(1:3,4);
        lambda_r_i = LAMBDA_R(:,i);

        %--Jacobiano de velocidad angular--
        j_w_i = mth_r_i*lambda_r_i;
        j_w(:,i) = j_w_i;
        
        %--Jacobiano de velocidad lineal--
        for j = 1:i
            mth0_j = reshape(MTH(j,:), 4, 4);
            d_j = mth0_j(1:3,4);
            j_w_j = j_w(:,j);
            j_v(:,j) = cross(j_w_j, (d_p-d_j));
        end

        j_i(1:3, :) = j_v;
        j_i(4:6, :) = j_w;
        
        J(i,:) = reshape(j_i, 1, (6*n));
    end
    J = simplify(J);
end
