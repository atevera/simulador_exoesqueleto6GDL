% Constructor de parámetros dinámicos de un exoesqueleto de 6DGL mediante
% la asignación de referenciales GRyMA y el modelado dinámico considerando
% D'Alambert-Lagrange como principio de análisis. 

clear All
clc
format long eng

%-- Unidades:
%           | Todas se encuentran de acuerdo al Sistema Internacional. 

%-- Longitudes entre cada referencial de acuerdo al plano.
l = [0.03235525 0.10513390 0.02462267 0.02228474 0.04334075 0.00600000 0.01999013 0.02565247 0.01273194 0.03641522];

%-- Centros de masa de cada eslabón en coordenadas del referencial local xyz
cm1 = [0.01068092 0.00868293 0.05522912]';
cm2 = [-0.02243335 0 0.01580661]';
cm3 = [-0.01025512 0 0.00922495]';
cm4 = [-0.00584482 -6.47E-06 0.02311164]';
cm5 = [-6.48E-05 0.01247162 0.00568686]';
cm6 = [-0.00999471 0 0.01838876]';
center_of_mass = [cm1, cm2, cm3, cm4, cm5, cm6];

%-- Masas de cada eslabón
masas = [0.04496018 0.02038587 0.00333783 0.02889644 0.0108769 0.00726735];

%-- Vector de gravedad en coordenadas inerciales
g_0 = [0 0 -9.81]'; % No se contempla con el - de la fórmula?

%-- Tensores de inercia de cada eslabón respecto a su centro de masa
inertial_tensor = [8.74E-05 2.47E-06 2.42E-05 2.47E-06 9.19E-05 0 2.42E-05 0 1.61E-05;
                   1.71E-06 0 -1.36E-06 0 3.61E-06 0 -1.36E-06	0 2.76E-06;
                   1.60E-07 0 -5.00E-08 0 2.20E-07 0 -5.00E-08	0 1.10E-07;
                   7.83E-06 0 -3.24E-06 0 9.84E-06 0 -3.24E-06	0 3.11E-06;
                   6.20E-07 1.00E-08 0 1.00E-08 2.40E-07 4.00E-08 0 4.00E-08 5.70E-07;
                   9.30E-07 0 -9.00E-08 0 1.02E-06 0 -9.00E-08	0 1.02E-06];

%-- Array de ángulos --%
q = sym('q',[1 6]);
dq = sym('dq',[1 6]);

%-- Ajuste del tercer referencial --%
alpha = 0;
beta = -59.26721315*pi()/180; % Desviación del vector unitario de rotación respecto al eje Z

%-- Vectores de rotación --%
motion_codes = [5 5 8 6 4 5 0 beta];
lambda = rotation_vectors(motion_codes);

translation_ref = [[0 0 0]',[l(1) 0 l(2)]',[-l(10) 0 l(3)]',[-l(9) 0 l(4)]',[0 0 l(5)]',[0 0 l(6)]',[-l(8) 0 l(7)]'];

%-- Cinemática directa para los referenciales no inerciales
htm1_6 = local_homogeneous_transform_matrix(translation_ref, lambda, q);

disp('Homogeneous transformations: Done!')

fk0_6 = forward_kinematics(htm1_6);

matlabFunctionBlock('simulator_prototype/foward_kinematics',fk0_6)

%%

%-- Cinemática directa para los centros de masa.
fk_cm0_6 = forward_kinematics_center_of_masa_offset(fk0_6, center_of_mass);

disp('Forward kinematics: Done!')

%-- Jacobiano geométrico para referenciales en articulaciones
%J_Art = jacobian_generator(fk0_6, lambda);  * Nota: No es utilizado :)

%-- Jacobiano geométrico para centros de masas
J_CM = jacobian_generator(fk_cm0_6, lambda);

disp('Jacobians: Done!')

%-- Construcción de la matriz de Inercia H(q)
H = inertia_matrix(masas, J_CM, fk_cm0_6, inertial_tensor);
%matlabFunction(H, 'File', 'H_sym_function');
matlabFunctionBlock('simulator_prototype/Inertia_matrix',H)

disp('Inertia Matrix: Done!')

% -- Construcción de vector de Coriolis.
tau_c = coriolis_vector(H, q, dq);
%matlabFunction(tau_c, 'File', 'tau_c_sym_function');
matlabFunctionBlock('simulator_prototype/Coriolis_vector',tau_c)

disp('Coriolis vector: Done!')

tau_g = gravity_vector(masas, J_CM, g_0);
%matlabFunction(tau_g, 'File', 'tau_g_sym_function');
matlabFunctionBlock('simulator_prototype/Gravity_vector',tau_g)

disp('Gravity Vector: Done!')

b = [0.1 0.1 0.1 0.1 0.1 0.1];
tau_d = dissipative_vector(b, dq);
%matlabFunction(tau_d, 'File', 'tau_d_sym_function');
matlabFunctionBlock('simulator_prototype/Dissipative_vector',tau_d)

disp('Dissipative Vector: Done!')

disp('Enjoy your robot! :)')