%--Cálculo de Jacobiano Geométrico--
%   Necesario:
%       -Array de matrices de transformación en la forma MTH = {[mth0_1] [mth0_2]...
%       -Matriz de vectores directores LAMBDA_R = [lambda_r1 lambda_r2 lambda_r3 lambda_r4 lambda_r5 lambda_r6]
function J = jacobian_generator(MTH, LAMBDA_R)
    n = size(LAMBDA_R,2);
    J = zeros(n,(6*n));
    j_w = zeros(3, n);
    for i = 1:n
        j_i = zeros(6, n);
        j_v = zeros(3, n);
        
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
end
