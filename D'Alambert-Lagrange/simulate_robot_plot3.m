% Código para simulación de movimiento de brazo robótico
% Necesario correr el modelo de simulink para que este código funcione :)

function simulate_robot_plot3(coord_out)
    coord = coord_out*100;

    x_min = min(min(coord(:,1,:)));
    x_max = max(max(coord(:,1,:)));
    
    y_min = min(min(coord(:,2,:)));
    y_max = max(max(coord(:,2,:)));
    
    z_min = min(min(coord(:,3,:)));
    z_max = max(max(coord(:,3,:)));
    
    line_w = 4;
    
    figure('Position', [300 100 900 600])
    
    ex_time = 'Execution time: ' + string(size(coord,3)*0.01) + 's';
    
    disp(ex_time)
    
    for i = 1:size(coord,3)
        u1 = coord(1,1,i); % Coordenadas finales eslabón 1
        v1 = coord(1,2,i); % Coordenadas finales eslabón 1
        w1 = coord(1,3,i); % Coordenadas finales eslabón 1
    
        u2 = coord(2,1,i); % Coordenadas finales eslabón 1
        v2 = coord(2,2,i); % Coordenadas finales eslabón 1
        w2 = coord(2,3,i); % Coordenadas finales eslabón 1
    
        u3 = coord(3,1,i); % Coordenadas finales eslabón 1
        v3 = coord(3,2,i); % Coordenadas finales eslabón 1
        w3 = coord(3,3,i); % Coordenadas finales eslabón 1
    
        u4 = coord(4,1,i); % Coordenadas finales eslabón 1
        v4 = coord(4,2,i); % Coordenadas finales eslabón 1
        w4 = coord(4,3,i); % Coordenadas finales eslabón 1
    
        u5 = coord(5,1,i); % Coordenadas finales eslabón 1
        v5 = coord(5,2,i); % Coordenadas finales eslabón 1
        w5 = coord(5,3,i); % Coordenadas finales eslabón 1
    
        u6 = coord(6,1,i); % Coordenadas finales eslabón 1
        v6 = coord(6,2,i); % Coordenadas finales eslabón 1
        w6 = coord(6,3,i); % Coordenadas finales eslabón 1
    
    
        X = [u1,u2,u3,u4,u5,u6];
        Y = [v1,v2,v3,v4,v5,v6];
        Z = [w1,w2,w3,w4,w5,w6];
    
        p1 = plot3(X(1:2),Y(1:2),Z(1:2));
        hold on
        p2 = plot3(X(2:3),Y(2:3),Z(2:3));
        p3 = plot3(X(3:4),Y(3:4),Z(3:4));
        p4 = plot3(X(4:5),Y(4:5),Z(4:5));
        p5 = plot3(X(5:6),Y(5:6),Z(5:6));
    
        hold off
    
        p1.LineWidth = line_w;
        p2.LineWidth = line_w;
        p3.LineWidth = line_w;
        p4.LineWidth = line_w;
        p5.LineWidth = line_w;
    
        axis equal
        xlim([x_min x_max])
        ylim([y_min y_max])
        zlim([z_min z_max])
        
        title('Robot arm visualization')
        xlabel('X: [cm]')
        ylabel('Y: [cm]')
        zlabel('Z: [cm]')
        grid on
        pause(0.01);
    end
    
    disp('Simulation finished :D')

end


