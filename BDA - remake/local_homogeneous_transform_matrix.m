function homogeneous_matrix = local_homogeneous_transform_matrix(translation, lambda, q) 
    num_ref = size(lambda, 2);
    homogeneous_matrix = sym(zeros(num_ref,16));
    for i = 1:num_ref
        lambda_i = lambda(:,i);
        rotation_matrix = sym(expm(OPC(lambda_i(4:6)*q(i))));
        hm_i = [rotation_matrix translation(:,i); 0 0 0 1];
        homogeneous_matrix(i,:) = reshape(hm_i,1,16);
    end
    homogeneous_matrix = simplify(homogeneous_matrix);
end
