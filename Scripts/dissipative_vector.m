%-- Construcción del vector de disipación.
%   Nota: Por ahora se considera una disipación simple con la matriz 
%         diagonal de los coeficientes de fricción viscosa de cada GLD. 
%         Sin embargo, sería adecuado considerar la función disipativa
%         generalizada de Rayleigh R(q,dq) para ser consistentes con el 
%         libro. Más información en MultiRigidBodies Pág. 92
% Parámetros:
%       - Vector 'b' de coeficientes de fricción viscosa
%       - Vector de velocidades generalizadas
function D_q = dissipative_vector(b, dq)
    viscosity_matrix = sym(diag(b));
    D_q = viscosity_matrix*dq';  
end