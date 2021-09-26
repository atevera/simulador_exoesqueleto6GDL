function code_mov = MotionCodes(code,a,b)
    code_mov = zeros(3,length(code));
    for i = 1:length(code)
        switch code(i)
            case 4
                lambda = [1 0 0]';
            case 5
                lambda = [0 1 0]';
            case 6 
                lambda = [0 0 1]';
            otherwise
                lambda = [cos(a)*sin(b) sin(a)*sin(b) cos(b)]';
        end
        code_mov(:,i) = lambda; 
    end
end

