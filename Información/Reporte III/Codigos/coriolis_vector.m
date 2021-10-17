function C = coriolis_vector(H, q, dq)
    n = size(H,1);
    C = sym(zeros(n,1));
    for k = 1:n
        cs = 0;
        for i = 1:n
            for j = 1:n
                cs_1 = diff(H(k,j),q(i));
                cs_2 = diff(H(i,k),q(j));
                cs_3 = diff(H(i,j),q(k));
                cs_ijk = (1/2)*(cs_1 + cs_2 - cs_3);
                cs = cs + (cs_ijk*dq(i)*dq(j));
            end
        end
        C(k,1) = cs;
    end
    close(f)
end