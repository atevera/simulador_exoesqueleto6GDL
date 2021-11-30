clear All
clc
format long eng

%-- Longitudes entre cada referencial de acuerdo al plano.
l = [0.03235525 0.10513390 0.02462267 0.02228474 0.04334075 0.00600000 0.01999013 0.02565247 0.01273194 0.03641522];

%-- Centros de masa de cada eslabón en coordenadas del referencial local xyz
cm1 = [0.01068092 0.00868293 0.05522912]';
cm2 = [-0.02243335 0 0.01580661]';
cm3 = [-0.01025512 0 0.00922495]';
cm4 = [-0.00584482 -6.47E-06 0.02311164]';
cm5 = [-6.48E-05 0.01247162 0.00568686]';
cm6 = [-0.00999471 0 0.01838876]';
CM_i = [cm1, cm2, cm3, cm4, cm5, cm6];

%-- Masas de cada eslabón
M = [0.04496018 0.02038587 0.00333783 0.02889644 0.0108769 0.00726735];

%-- Vector de gravedad en coordenadas inerciales
g_0 = [0 0 -9.81]';

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
LAMBDA = director_vectors(motion_codes);