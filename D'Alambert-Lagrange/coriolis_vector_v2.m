function tau_c = coriolis_vector_v2(H,q,dq)
    p = H*dq';
    A = simplify(jacobian(p,q));
    C = A - (1/2)*transpose(A);
    tau_c = C*dq';
end