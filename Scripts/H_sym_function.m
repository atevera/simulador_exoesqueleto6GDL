function H_eval = H_sym_function(q1,q2,q3,q4,q5,q6)
    H_test = load('H_q_test.mat');
    H_eval = double(H_test.H(q1,q2,q3,q4,q5,q6));
end
