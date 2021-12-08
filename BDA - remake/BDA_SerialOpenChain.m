% Constructor de parámetros dinámicos de un exoesqueleto de 6DGL mediante
% la asignación de referenciales GRyMA y el modelado dinámico considerando
% BDA como principio de análisis. 

%% -- Section 1 to review -- 
% Modified:
% rotation_vectors --> lambda_vectors
% lambda_extended --> Modified to be more intuitive

clear all; close all; clc
format long eng

%-- Unidades:
%           | Todas se encuentran de acuerdo al Sistema Internacional. 

%-- Longitudes entre cada referencial de acuerdo al plano.
l = (1e-3)*[32.35524933 105.13390433 25.03848390 22.86892447 42.34075154 6.00000000 19.99012852 25.65247176 13.18958611 36.13755609];

%-- Centros de masa de cada eslabón en coordenadas del referencial local xyz
cm1 = [10.68092384 8.68292843 55.22911881]';
cm2 = [-22.43335471 0 15.80661314]';
cm3 = [-10.25512498 -0.00000517 9.22495247]';
cm4 = [-5.84481537 -0.00647382 23.11164304]';
cm5 = [-0.06482618 12.47161684 5.68685820]';
cm6 = [-9.99471073 -0.00000193 18.38876334]';
center_of_mass = (1e-3)*[cm1, cm2, cm3, cm4, cm5, cm6];

%-- Masas de cada eslabón
masas = (1e-3)*[44.96017690 20.38587093 3.33782865 28.89644336 10.87689656 7.26735103];

%-- Vector de gravedad en coordenadas inerciales
g_0 = [0 0 -9.81]';
%-- Vector extendido de gravedad --%
G0 = extended_gravity(g_0); % || Tony validation ||

%-- Tensores de inercia de cada eslabón respecto a su centro de masa
%   NOTA: Es necesario para la matriz de masas M pero se debe revisar que
%         tensor requiere, si en el centro de masa o respecto a su eje de
%         rotación.
inertial_tensor = [87358.20798657 2465.66331565 24175.86376346 2465.66331565 91892.16758832 4.61681686 24175.86376346 4.61681686 16105.85112546;
                   1705.36305889 0.00014492 -1358.50435265 0.00014492 3612.62309834 0.00044415 -1358.50435265 0.00044415 2761.99031995;
                   163.77908527 0.00028743 -49.09401393 0.00028743 222.87315087 -0.00067942 -49.09401393 -0.00067942 113.22528713;
                   7832.40999638 -0.59913483 -3243.70459165 -0.59913483 9835.11139154 0.64356808 -3243.70459165	0.64356808 3111.08107433;
                   616.21453787 8.79383303 -1.48514248 8.79383303 236.03269139 42.29140167 -1.48514248 42.29140167 565.46784116;
                   925.87316996 -0.00017447 -88.80624626 -0.00017447 1017.52426404 0.00015471 -88.80624626	0.00015471 1018.20007511];

inertial_tensor = (1e-9)*inertial_tensor;

%-- Array de ángulos --%
q = sym('q',[1 6],'real');
dq = sym('dq',[1 6],'real');

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
%Mc = mass_matrix(masas, center_of_mass, inertial_tensor);
M = mass_matrix_local_reference(masas, center_of_mass, inertial_tensor);

%--%--%--%--%--%--%--%--%--%--%--%--%--%--%--%--%--%--%--%--%
%--- Inicialización de variables ---%
H = sym(zeros(6,6),'real');      % Inicialización de matriz de inercia
h = sym(zeros(6,1),'real');      % Inicialización del "vector dinámico"
J = sym(zeros(6,36),'real');     % Inicialización de Jacobianos locales
a = sym(zeros(6,6),'real');      % Inicialización de aceleración residual
twist = sym(zeros(6,6),'real');  % Inicialización del twist

%--- Vectores extendidos de rotación^T y traslación ---%
T = extended_translation(d);
R_T = extended_rotation_T(R);

%% -- Section 3 to review -- 

%--- INITIAL CONDITIONS ---%
a_0 = -G0;
g = sym(zeros(6,1),'real');
twist_0 = zeros(6,1);
J_0 = zeros(6,6);

%--- Recursividad para cada eslabón ---%
n = 6; % Prueba de recursividad
for i = 1:n
    T_i = reshape(T(i,:),6,6);
    R_i_T = reshape(R_T(i,:),6,6);
    X_i = R_i_T*T_i;

    if i == 1
        twist_i = X_i*twist_0 + lambda(:,1)*dq(1);
        a_i = X_i*a_0 - dq(1)*Omega(lambda(:,1))*twist_i;
        LAMBDA_1 = reshape(LAMBDA(1,:),6,6);
        J_i = X_i*J_0 + LAMBDA_1;   
    else
        % Notación --> j = i - 1
        twist_j = reshape(twist(i-1,:),6,1);
        twist_i = X_i*twist_j + lambda(:,i)*dq(i);
        a_j = reshape(a(i-1,:),6,1);
        a_i = X_i*a_j - dq(i)*Omega(lambda(:,i))*twist_i;
        J_j = reshape(J(i-1,:),6,6);
        LAMBDA_i = reshape(LAMBDA(i,:),6,6);
        J_i = X_i*J_j + LAMBDA_i;
    end
  
    a(i,:) = reshape(a_i, 1, 6);
    twist(i,:) = reshape(twist_i, 1, 6);
    J(i,:) = reshape(J_i,1,36);


    M_i = reshape(M(i,:),6,6);


    H = H + J_i.'*M_i*J_i;

    h = h + J_i.'*(M_i*a_i-((Omega(twist_i)).')*M_i*twist_i); 

    g = g + J_i.'*M_i*R_i_T*G0;

end


%% Cálculo de cinemática directa
fk0_6 = forward_kinematics(A);
% Desplazamiento al centro de masa
fk_cm0_6 = forward_kinematics_center_of_masa_offset(fk0_6, center_of_mass);
%% Análsis energético
U = potential_energy(masas,fk_cm0_6,g_0);
K = (1/2)*(dq)*H*(dq.');

g = -g;

%% --- Export to Simulink ---
matlabFunctionBlock('model_BDA/h',h)
matlabFunctionBlock('model_BDA/H',H)
matlabFunctionBlock('model_BDA/g',g)
matlabFunctionBlock('model_BDA/K',K)
matlabFunctionBlock('model_BDA/U',U)