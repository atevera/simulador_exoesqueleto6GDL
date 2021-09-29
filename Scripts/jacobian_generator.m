%--Cálculo de Jacobiano Geométrico--
%   Necesario:
%       -Array de matrices de transformación en la forma MTH = {[mth0_1] [mth1_2]...
%       -Matriz de transformación homogenea de 0 a 6 mth0_6 = mth0_1*mth1_2*mth2_3*mth3_4*mth4_5*mth5_6
%       -Matriz de vectores directores LAMBDA_R = [lambda_r1 lambda_r2 lambda_r3 lambda_r4 lambda_r5 lambda_r6]
function J = jacobian_generator(MTH, mth0_6, LAMBDA_R)
    J = zeros(6, length(LAMBDA_R));
    mth0_i = eye(4);
    d_p = mth0_6(1:3,4);

    for i = 1:length(LAMBDA_R)
        mth0_i = mth0_i * MTH{i};
        mth_r_i = mth0_i(1:3,1:3);
        lambda_r_i = LAMBDA_R(:,i);
        d_i = mth0_i(1:3,4);

        %--Jacobiano de velocidad angular--
        j_w_i = mth_r_i*lambda_r_i;

        %--Jacobiano de velocidad lineal--
        j_v_i = cross(j_w_i, (d_p-d_i));

        J(1:3, i) = j_v_i;
        J(4:6, i) = j_w_i;
    end
end
