function C = coriolis_matrix(H,q,dq)
    n = size(H,1);
    C = sym(zeros(n));
    f = waitbar(0, 'Starting');
    
    s = 1;
    t = n^2;

    for k = 1:n
        for j = 1:n
            c_kj = 0;
            for i = 1:n
                c_ijk = (1/2)*(diff(H(k,j),q(i))+diff(H(k,i),q(j))-diff(H(i,j),q(k)));
                c_kj = c_kj + c_ijk*dq(i);
            end
            C(k,j) = c_kj;
            waitbar(s/t, f, sprintf('Coriolis Matrix: %d %%', floor(s/t*100)));
            s = s + 1;
        end
    end
    close(f)
end