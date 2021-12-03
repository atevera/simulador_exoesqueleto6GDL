% Dynamic parameters from remade version
clear all
clc
format long eng

%-- Longitudes entre cada referencial de acuerdo al plano.
l = 1e-3*[32.35524933 105.13390433 25.03848390 22.86892447 42.34075154 6.00000000 19.99012852 25.65247176 13.18958611 36.13755609];

%-- Centros de masa de cada eslabón en coordenadas del referencial local xyz
cm1 = [10.68092384 8.68292843 55.22911881]';
cm2 = [-22.43335471 0 15.80661314]';
cm3 = [-10.25512498 -0.00000517 9.22495247]';
cm4 = [-5.84481537 -0.00647382 23.11164304]';
cm5 = [-0.06482618 12.47161684 5.68685820]';
cm6 = [-9.99471073 -0.00000193 18.38876334]';

CM_n = 1e-3*[cm1, cm2, cm3, cm4, cm5, cm6];

%-- Masas de cada eslabón
m_n = [0.04496018 0.02038587 0.00333783 0.02889644 0.0108769 0.00726735];

%-- Vector de gravedad en coordenadas inerciales
g_0 = [0 0 9.81]';

%-- Tensores de inercia de cada eslabón respecto a su centro de masa
inertial_tensor = [87358.20798657 2465.66331565 24175.86376346 2465.66331565 91892.16758832 4.61681686 24175.86376346 4.61681686 16105.85112546;
                   1705.36305889 0.00014492 -1358.50435265 0.00014492 3612.62309834 0.00044415 -1358.50435265 0.00044415 2761.99031995;
                   163.77908527 0.00028743 -49.09401393 0.00028743 222.87315087 -0.00067942 -49.09401393 -0.00067942 113.22528713;
                   7832.40999638 -0.59913483 -3243.70459165 -0.59913483 9835.11139154 0.64356808 -3243.70459165	0.64356808 3111.08107433;
                   616.21453787 8.79383303 -1.48514248 8.79383303 236.03269139 42.29140167 -1.48514248 42.29140167 565.46784116;
                   925.87316996 -0.00017447 -88.80624626 -0.00017447 1017.52426404 0.00015471 -88.80624626	0.00015471 1018.20007511];

inertial_tensor = inertial_tensor * 1e-9;

%-- Array de ángulos --%
q = sym('q',[1 6],'real');
dq = sym('dq',[1 6],'real');
b = sym('b',[1 6],'real');

%-- Ajuste del tercer referencial --%
alpha = 0;
beta = -59.26721315*pi()/180; % Desviación del vector unitario de rotación respecto al eje Z

%-- Vectores directores--%
motion_codes = [5 5 8 6 4 5 0 beta];
LAMBDA = director_vectors(motion_codes);

D1_6 = [[0 0 0]',[l(1) 0 l(2)]',[-l(10) 0 l(3)]',[-l(9) 0 l(4)]',[0 0 l(5)]',[0 0 l(6)]',[-l(8) 0 l(7)]'];

%-- Cinemática directa para los referenciales no inerciales
A1_6 = local_homogeneous_transform_matrix(D1_6, LAMBDA, q);
FK0_6 = forward_kinematics(A1_6);

disp('Cinemática directa: Listo!')

%-- Jacobiano geométrico para referenciales en articulaciones
J0_n = jacobian_generator(FK0_6, LAMBDA(4:6,:));

Jn = local_jacobian_generator(FK0_6,J0_n);

disp('Jacobianos: Listo!')

%-- Cálculo de la matriz de inercia
%H = inertia_matrix(Jn, CM_n, m_n, inertial_tensor);

%-- Cálculo de matriz de Coriolis
%C = coriolis_matrix(H,q,dq);

%-- Cálculo de energía potencial
P = potential_energy(m_n,g_0,FK0_6,CM_n);

% -- Energía cinética
%K = (1/2)*dq*H*dq.';

%-- Cálculo de vector de gravedad
G = gravity_vector(P,q);

%-- Cálculo de matriz de disipación
%D = [b(1) 0 0 0 0 0
%      0 b(2) 0 0 0 0
%      0 0 b(3) 0 0 0
%      0 0 0 b(4) 0 0
%      0 0 0 0 b(5) 0
%      0 0 0 0 0 b(6)];

% -- Creación de vectores a partir de componentes dinámicos
%C_dq = C*dq.';
%D_dq = D*dq.';

disp('Components complete!, now creating matlab functions...')

% -- Creación de funciones en simulink
% matlabFunctionBlock('simulator_prototype_remade_v1/Dynamic Model/H_Matrix',H)
% clear H
% disp('Inertia matrix block, done!')

matlabFunctionBlock('simulator_prototype_remade_v1/Dynamic Model/Gq',G)
clear G
disp('Gravity vector block, done!')

% matlabFunctionBlock('simulator_prototype_remade_v1/Dynamic Model/Cdq',C_dq)
% clear C_dq
% disp('Coriolis vector block, done!')
% 
% matlabFunctionBlock('simulator_prototype_remade_v1/Dynamic Model/D_dq',D_dq)
% clear D_dq
% disp('Dissipation vector block, done!')

matlabFunctionBlock('simulator_prototype_remade_v1/Energy Analysis/P',P)
clear P
disp('Potential energy block, done!')

% matlabFunctionBlock('simulator_prototype_remade_v1/Energy Analysis/K',K)
% clear K
% disp('Kinetic energy block, done!')






