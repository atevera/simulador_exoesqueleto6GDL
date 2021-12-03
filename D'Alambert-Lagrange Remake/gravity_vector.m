function G = gravity_vector(P,q)
    n = size(q,2);
    G = sym(zeros(n,1));
    
    for i = 1:n
        G(i) = diff(P,q(i));
    end

    G = simplify(G);

end