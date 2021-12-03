% Constructor de parámetros dinámicos de un exoesqueleto de 6DGL mediante
% la asignación de referenciales GRyMA y el modelado dinámico considerando
% BDA como principio de análisis. 

%% -- Section 1 to review -- 
% Modified:
% rotation_vectors --> lambda_vectors
% lambda_extended --> Modified to be more intuitive

clear all; close all; clc
format long eng

% IMPORTANTE: 
% Para el uso de algunas de las funciones, utiliza este comando desde la
% raíz del proyecto: 
% ---> addpath("D'Alambert-Lagrange")
% ---> addpath("BDA")

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
g_0 = [0 0 -9.81]';
%-- Vector extendido de gravedad --%
G0 = extended_gravity(g_0); % || Tony validation ||

%-- Tensores de inercia de cada eslabón respecto a su centro de masa
%   NOTA: Es necesario para la matriz de masas M pero se debe revisar que
%         tensor requiere, si en el centro de masa o respecto a su eje de
%         rotación.
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
lambda = lambda_vectors(motion_codes); % || Tony validation ||

%-- Matriz de vectores de rotación --%
LAMBDA = lambda_extended(lambda); % || Tony validation ||

translation_ref = [[0 0 0]',[l(1) 0 l(2)]',[-l(10) 0 l(3)]',[-l(9) 0 l(4)]',[0 0 l(5)]',[0 0 l(6)]',[-l(8) 0 l(7)]'];

%% -- Section 2 to review -- 

%-- Matrices de Transformación Homogénea --%
A = local_homogeneous_transform_matrix(translation_ref, lambda, q);

%-- Obtiene la matriz de rotación y el vector de traslación de la MTH ---%
[R, d]= get_rotation_translation(A);

%-- Matriz de masas --%
M = mass_matrix(masas, center_of_mass, inertial_tensor);

%--%--%--%--%--%--%--%--%--%--%--%--%--%--%--%--%--%--%--%--%
%--- Inicialización de variables ---%
H = sym(zeros(6,6));      % Inicialización de matriz de inercia
h = sym(zeros(6,1));      % Inicialización del "vector dinámico"
J = sym(zeros(6,36));     % Inicialización de Jacobianos locales
a = sym(zeros(6,6));      % Inicialización de aceleración residual
twist = sym(zeros(6,6));  % Inicialización del twist

% J_cm = sym(zeros(6,36));     % Inicialización de Jacobianos locales al centro de masa
% a_cm = sym(zeros(6,6));      % Inicialización de aceleración residual al centro de masa
% twist_cm = sym(zeros(6,6));  % Inicialización del twist al centro de masa

%--- Vectores extendidos de rotación^T y traslación ---%
T = extended_translation(d);
T_cm = extended_translation(center_of_mass');
R_T = extended_rotation_T(R);

%% -- Section 3 to review -- 

%--- INITIAL CONDITIONS ---%
a_0 = -G0;
g = zeros(6,1);
twist_0 = zeros(6,1);
J_0 = zeros(6,6);

%--- Recursividad para cada eslabón ---%
for i = 1:6
    T_i = reshape(T(i,:),6,6);
    R_i_T = reshape(R_T(i,:),6,6);
    X_i = R_i_T*T_i;

    if i == 1
        %twist_i = X_i*twist_0 + [zeros(3,1); lambda(:,1)]*dq(1);
        twist_i = X_i*twist_0 + lambda(:,1)*dq(1);
        % a_i = X_i*a_0 - dq(1)*Omega(lambda(:,1))*twist_i; 
        a_i = X_i*a_0 - Omega(lambda(:,1)*dq(1))*twist_i; 
        LAMBDA_1 = reshape(LAMBDA(1,:),6,6);
        J_i = X_i*J_0 + LAMBDA_1;   
    else
        % Notación --> j = i - 1
        twist_j = reshape(twist(i-1,:),6,1);
        %twist_i = X_i*twist_j + [zeros(3,1); lambda(:,i)]*dq(i);
        twist_i = X_i*twist_j + lambda(:,i)*dq(i);
        a_j = reshape(a(i-1,:),6,1);
        % a_i = X_i*a_j - dq(i)*Omega(lambda(:,i))*twist_i;
        a_i = X_i*a_j - Omega(lambda(:,i)*dq(i))*twist_i;
        J_j = reshape(J(i-1,:),6,6);
        LAMBDA_i = reshape(LAMBDA(i,:),6,6);
        J_i = X_i*J_j + LAMBDA_i;
    end
    
    T_cm_i = reshape(T_cm(i,:),6,6);

    a(i,:) = reshape(a_i, 1, 6);
    a_cm_i = T_cm_i*a_i;

    twist(i,:) = reshape(twist_i, 1, 6);
    twist_cm_i = T_cm_i*twist_i;

    J(i,:) = reshape(J_i,1,36);
    J_cm_i = T_cm_i*J_i;

    M_i = reshape(M(i,:),6,6);


    H = H + J_cm_i'*M_i*J_cm_i;

    h = h + J_cm_i'*( M_i*a_cm_i-(Omega(twist_cm_i))'*M_i*twist_cm_i); 

    g = g + masas(i)*J_cm_i'*R_i_T*G0;
end

%--- Vector de disipación con fricción viscosa simple
b = 0.01; % Puede ser simbólica para ser variable en la GUI
D = b*dq';

%--- Export to Simulink ---%
%matlabFunctionBlock('BDA_DynamicModel/h',h)
%matlabFunctionBlock('BDA_DynamicModel/H',H)
matlabFunctionBlock('BDA_DynamicModel/g',g)



