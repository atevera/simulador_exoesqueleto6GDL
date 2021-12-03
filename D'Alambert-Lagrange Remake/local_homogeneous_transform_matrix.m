function homogeneous_matrix = local_homogeneous_transform_matrix(D1_6, LAMBDA, q) 
    num_ref = size(LAMBDA, 2);
    homogeneous_matrix = sym(zeros(num_ref,16));
    for i = 1:num_ref
        rotation_matrix = expm(OPC(LAMBDA(4:6,i))*q(i));
        hm_i = [rotation_matrix D1_6(:,i); 0 0 0 1];
        homogeneous_matrix(i,:) = reshape(hm_i,1,16);
    end
    homogeneous_matrix = simplify(homogeneous_matrix);
end
