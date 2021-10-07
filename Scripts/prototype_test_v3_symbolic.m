% Archivo de pruebas para la concentración de funciones y procesos
% para la construcción del simulador. 

clear All
clc

%-- Longitudes entre cada referencial, ajuste ángulo alpha y beta --%
l = [10 20 15.21 20.12 10 4.47 7.6 10 10]; % [cm] 
d = [0 0 16]; % Posición del dedal en coordenadas del referencial local

%-- Centros de masa de cada eslabón en coordenadas del referencial local xyz
cm_offset = [[5 0 10]', [-5 0 6]', [-3 0 10]', [0 0 5]', [0 0 5]', [0 0 5]'];

%-- Masas de cada eslabón
masas= [0.3 0.5 0.2 1 1 1];  % [kg]

%-- Vector de gravedad en coordenadas inerciales
g_0 = [0 0 -9.81]';

%-- Tensor de inercia test
inertial_tensor = [0.9844 0.5134 0.1339 0.8589 0.1776 0.0309 0.7856 0.3986 0.9391; %Aplanado por columnas 
                   0.9844 0.5134 0.1339 0.8589 0.1776 0.0309 0.7856 0.3986 0.9391;
                   0.9844 0.5134 0.1339 0.8589 0.1776 0.0309 0.7856 0.3986 0.9391;
                   0.9844 0.5134 0.1339 0.8589 0.1776 0.0309 0.7856 0.3986 0.9391;
                   0.9844 0.5134 0.1339 0.8589 0.1776 0.0309 0.7856 0.3986 0.9391;
                   0.9844 0.5134 0.1339 0.8589 0.1776 0.0309 0.7856 0.3986 0.9391];

%-- Array de ángulos --%
q = sym('q',[1 6]);
dq = sym('dq',[1 6]);

%-- Ajuste del tercer referencial --%
alpha = 0;
beta = -26.57*pi()/180; %Gamma offset: Obtenido de SolidWorks

%-- Vectores de rotación --%
motion_codes = [5 5 8 6 4 5 0 -26.57*pi/180]; %Codes, alpha, beta
lambda = rotation_vectors(motion_codes);

translation_ref = [[0 0 0]',[l(1) 0 l(2)]',[-l(7) 0 l(3)]',[-l(6) 0 l(4)]',[0 0 l(5)]',[0 0 l(8)]',[0 0 l(9)]'];

%-- Cinemática directa para los referenciales no inerciales
htm1_6 = local_homogeneous_transform_matrix(translation_ref, lambda, q);

disp('Homogeneous transformations: Done!')

fk0_6 = forward_kinematics(htm1_6);

%-- Cinemática directa para los centros de masa.
fk_cm0_6 = forward_kinematics_center_of_masa_offset(fk0_6, cm_offset);

disp('Forward kinematics: Done!')

%-- Jacobiano geométrico para referenciales en articulaciones
%J_Art = jacobian_generator(fk0_6, lambda);  * Nota: No es utilizado :)

%-- Jacobiano geométrico para centros de masas
J_CM = jacobian_generator(fk_cm0_6, lambda);

disp('Jacobians: Done!')

%-- Construcción de la matriz de Inercia H(q) %Notas, si pones solo H, te
% permite manipularla como matriz pero no evaluarla. Si pones H(q) te
% permite evaluarla, pero no manipularla como matriz.

%Recomendación de uso:
%   - H --> Operaciones matriciales.
%   - H(q) --> Salvar modelo e implementar en función para simulink.
H = inertia_matrix(masas, J_CM, fk_cm0_6, inertial_tensor);
H_save(q) = inertia_matrix(masas, J_CM, fk_cm0_6, inertial_tensor);

disp('Inertia Matrix: Done!')

%-- Construcción de vector de Coriolis.
C(q,dq) = coriolis_vector(H, q, dq);

disp('Coriolis vector: Done!')

%save('H_q_test.mat', 'H_save');
%save('C_q_dq_test.mat', 'C');

g(q) = gravity_vector(masas, J_CM, g_0);
% save('g_q.mat', 'g'); %Export to variable file
 disp('Gravity Vector: Done!')

%b = [0.1 0.1 0.1 0.1 0.1 0.1];
%d(dq) = dissipative_vector(b, dq); % Error: Check it
% save('d_dq.mat', 'd'); %Export to variable file

disp('Enjoy your robot! :)')