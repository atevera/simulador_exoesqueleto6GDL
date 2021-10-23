function U = potential_energy(m, fkcm, g0)
    n = length(m);
    U = 0;
    for i = 1:n
        mth_i = reshape(fkcm(i,:), 4, 4);
        d_i = mth_i(1:3,4)';
        U = U - m(i)*d_i*g0;
    end
end