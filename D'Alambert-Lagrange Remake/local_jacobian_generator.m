function Jn = local_jacobian_generator(FK0_6,J0_n)
    Jn = sym(zeros(size(J0_n)));
    n = size(J0_n,1);
    for i = 1:n
        HTM0_i = reshape(FK0_6(i,:),4,4);
        R0_i = HTM0_i(1:3,1:3);
        J0_i = reshape(J0_n(i,:),6,n);
        Ji = [R0_i.'*J0_i(1:3,:);R0_i.'*J0_i(4:6,:)];
        Jn(i,:) = reshape(Ji,1,6*n);
    end

    Jn = simplify(Jn);
end