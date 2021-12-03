function P = potential_energy(m_n,g_0,FK0_6,CM_n)
    P = sym(0);
    n = size(m_n,2);

    for i = 1:n
        HTM_i = reshape(FK0_6(i,:),4,4);
        TM_i = [eye(3) CM_n(:,i);0 0 0 1];
        HTM_cmi = HTM_i*TM_i;
        r_cmi = HTM_cmi(1:3,4);
        P = m_n(i)*g_0.'*r_cmi;
    end
    
    P = simplify(P);
end