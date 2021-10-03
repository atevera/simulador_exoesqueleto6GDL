% Archivo de pruebas para la concentración de funciones y procesos
% para la construcción del simulador. 

clear All
clc

%-- Longitudes entre cada referencial, ajuste ángulo alpha y beta --%
l = [10 20 15.21 20.12 10 4.47 7.6 10 10]; % [cm] %Added non-official l8 and l9 lengths
d = [0 0 16]; % Posición del dedal en coordenadas del referencial local

%-- Centros de masa de cada eslabón en coordenadas del referencial local xyz
%cm_offset = {[5 0 10], [-5 0 6], [-3 0 10], [0 0 5], [0 0 5], [0 0 5]}; %Added coordinates for 2 extra joints
cm_offset = [[5 0 10]', [-5 0 6]', [-3 0 10]', [0 0 5]', [0 0 5]', [0 0 5]'];

%-- Masas de cada eslabón
masas= [0.3 0.5 0.2 1 1 1];  % [kg] %Added 2 extra masses

%-- Tensor de inercia test
%test_tensor = [0.9844 0.5134 0.1339; 0.8589 0.1776 0.0309; 0.7856 0.3986 0.9391];
%inertial_tensor = {test_tensor, test_tensor, test_tensor, test_tensor, test_tensor, test_tensor}; %Added 2 extra I_c
inertial_tensor = [0.9844 0.5134 0.1339 0.8589 0.1776 0.0309 0.7856 0.3986 0.9391; %Aplanado por columnas 
                   0.9844 0.5134 0.1339 0.8589 0.1776 0.0309 0.7856 0.3986 0.9391;
                   0.9844 0.5134 0.1339 0.8589 0.1776 0.0309 0.7856 0.3986 0.9391;
                   0.9844 0.5134 0.1339 0.8589 0.1776 0.0309 0.7856 0.3986 0.9391;
                   0.9844 0.5134 0.1339 0.8589 0.1776 0.0309 0.7856 0.3986 0.9391;
                   0.9844 0.5134 0.1339 0.8589 0.1776 0.0309 0.7856 0.3986 0.9391];

%-- Array de ángulos --%
q = [0 0 0 0 0 0]; %Removed 1 angle (corresponding to the end effector)
dq = [0.1 0.5 1 1.5 2 0.8]'; %Velocidad angular de los GDL [rad/s] %Removed 1 velocity
q = q*pi()/180; 

%-- Ajuste del tercer referencial --%
alpha = 0;
beta = -26.57*pi()/180; %Gamma offset: Obtenido de SolidWorks

%-- Vectores de rotación --%
motion_codes = [5 5 8 6 4 5 0 -26.57*pi/180]; %Codes, alpha, beta
lambda = rotation_vectors(motion_codes);

translation_ref = [[0 0 0]',[l(1) 0 l(2)]',[-l(7) 0 l(3)]',[-l(6) 0 l(4)]',[0 0 l(5)]',[0 0 l(8)]',[0 0 l(9)]']; %Modified last 2

%-- Cinemática directa para los referenciales no inerciales
htm1_6 = local_homogeneous_transform_matrix(translation_ref, lambda, q);
fk0_6 = forward_kinematics(htm1_6);

%-- Cinemática directa para los centros de masa.
fk_cm0_6 = forward_kinematics_center_of_masa_offset(fk0_6, cm_offset);

%-- Jacobiano geométrico para referenciales en articulaciones
J_Art = jacobian_generator(fk0_6, lambda);

%-- Jacobiano geométrico para centros de masas
J_CM = jacobian_generator(fk_cm0_6, lambda);

%-- Construcción de la matriz de Inercia H(q) 
H = inertia_matrix(masas, J_CM, fk_cm0_6, inertial_tensor);

