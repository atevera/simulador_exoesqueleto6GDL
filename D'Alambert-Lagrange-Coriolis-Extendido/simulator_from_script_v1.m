% Este script es una prueba para ejecutar el simulador desde aquí, obtener
% las salidas, y mostrarlas en gráficas desde el mismo script, así la
% interfaz gráfica tomaría los datos directamente desde aquí.

clc
clear All
close

t1 = 0;
t2 = 0;
t3 = 0;
t4 = 0;
t5 = 0;
t6 = 0;

torque = [1  2  3  4  5  6 ;
          t1 t2 t3 t4 t5 t6]';

sim_out = sim('simulator_prototype_v2.slx');

% position_array = reshape(sim_out.position,size(sim_out.position,1),size(sim_out.position,3));
% velocity_array = reshape(sim_out.velocity,size(sim_out.velocity,1),size(sim_out.velocity,3));

time = (sim_out.tout)';

simulate_robot_plot3(sim_out.coord_out);

% figure(10)
% plot(time, position_array)
% title('Joints position')
% legend('q1','q2','q3','q4','q5','q6')
% xlabel('time: [s]')
% ylabel('rad')
% 
% figure(11)
% plot(time, velocity_array)
% title('Joints velocity')
% legend('dq1','dq2','dq3','dq4','dq5','dq6')
% xlabel('time: [s]')
% ylabel('rad/s')



