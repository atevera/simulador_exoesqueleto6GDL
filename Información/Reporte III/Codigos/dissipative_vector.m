function D_q = dissipative_vector(b, dq)
    viscosity_matrix = sym(diag(b));
    D_q = viscosity_matrix*dq';  
end