% Cinemática directa con asignación de referenciales
% considerando el método de GRyMA. 
% Nota: Los valores de 'l' son propuestos y serán actualizados
% cuando el doctor lo indique. 

clear all
clc
format short 
%-- Longitudes entre cada referencial --%
l = [10 20 15.21 20.12 10 4.47 7.6]; % [mm]

%-- Array de ángulos --%
q = [0 0 0 0 0 0];
dq = [0.1 0.5 1 1.5 2 0.8]'; %Velocidad angular de los GDL [rad/s]
q = q*pi()/180; 

%-- Ajuste del tercer referencial --%
alpha = 0;
beta = -26.57*pi()/180; %Gamma offset: Obtenido de SolidWorks

%-- Vectores de rotación --%
lambda = rotation_vectors([5 5 8 6 4 5], alpha, beta);

translation_matrix = {[0 0 0],[l(1) 0 l(2)],[-l(7) 0 l(3)],[-l(6) 0 l(4)],[0 0 l(5)],[0 0 0]};

htm0_6 = homogeneous_transform_matrix(translation_matrix, lambda, q);

fk0_6 = forward_kinematics(htm0_6);



