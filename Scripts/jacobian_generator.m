%--Cálculo de Jacobiano Geométrico--
%   Necesario:
%       -Array de matrices de transformación en la forma MTH = {[mth0_1] [mth0_2]...
%       -Matriz de vectores directores LAMBDA_R = [lambda_r1 lambda_r2 lambda_r3 lambda_r4 lambda_r5 lambda_r6]
function J_ARR = jacobian_generator(MTH, LAMBDA_R)
    J_ARR = cell(1,length(LAMBDA_R));
    j_w = zeros(3, length(LAMBDA_R));
    for i = 1:length(LAMBDA_R)
        J = zeros(6, length(LAMBDA_R));
        j_v = zeros(3, length(LAMBDA_R));
        
        mth0_i = MTH{i};
        mth_r_i = mth0_i(1:3,1:3);
        d_p = mth0_i(1:3,4);
        lambda_r_i = LAMBDA_R{i};

        %--Jacobiano de velocidad angular--
        j_w_i = mth_r_i*lambda_r_i;
        j_w(:,i) = j_w_i;
        
        %--Jacobiano de velocidad lineal--
        for j = 1:i
            mth0_j = MTH{j};
            d_j = mth0_j(1:3,4);
            j_w_j = j_w(:,j);
            j_v(:,j) = cross(j_w_j, (d_p-d_j));
        end

        J(1:3, :) = j_v;
        J(4:6, :) = j_w;
        
        J_ARR{i} = J;
    end
end
