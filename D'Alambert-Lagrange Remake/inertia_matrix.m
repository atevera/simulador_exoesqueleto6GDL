function H = inertia_matrix(Jn, CM_n, m_n, inertial_tensor)
    f = waitbar(0, 'Starting');
    n = size(m_n,2);
    H = sym(zeros(n));

    for i = 1:n
        m_i = m_n(i);
        rc_i = CM_n(:,i);
        I_ci = reshape(inertial_tensor(i,:),3,3);
        J_i = reshape(Jn(i,:),6,n);
        M_i = [m_i*eye(3) -m_i*OPC(rc_i); m_i*OPC(rc_i) I_ci-(m_i*OPC(rc_i)^2)];
        H_i = J_i.'*M_i*J_i;
        H = H + H_i;
        H = simplify(H);
        waitbar(i/n, f, sprintf('Inertia Matrix: %d %%', floor(i/n*100)));
    end
    close(f)
end