%-- Construcción del vector extendido de traslación 'T'
% Parámetros:
%       - Vector de traslación [Matriz 6x3]
%         -> Cada renglón corresponde a cada eslabón
% Salida: 
%       - Vector extendido de traslación [Matriz 6x36]
%         -> Cada renglón corresponde a cada eslabón
% function T = extended_translation(d)
%     n = size(d,1);
%     T = sym(zeros(n,36));
%     for i = 1:n
%         d_i = reshape(d(i,:),3,1);
%         d_i_x = OPC(d_i);
%         T_temp = [eye(3) -d_i_x; zeros(3) eye(3)];
%         T(i,:) = reshape(T_temp,1,36);
%     end
% end

function T = extended_translation(d)
    n = size(d,1);
    T = sym(zeros(n,36));
    for i = 1:n
        d_i = d(i,:)';
        d_i_x = OPC(d_i);
        T_temp = [eye(3) -d_i_x; zeros(3) eye(3)];
        T(i,:) = reshape(T_temp,1,36);
    end
end

