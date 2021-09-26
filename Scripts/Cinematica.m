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
lambda = MotionCodes([5 5 8 6 4 5], alpha, beta);

%-- Construccion de matrices de transformación homogénea--%
ref1 = MTH([0 0 0], lambda(:,1), q(1));
ref2 = MTH([l(1) 0 l(2)], lambda(:,2), q(2));
ref3 = MTH([-l(7) 0 l(3)], lambda(:,3), q(3));
ref4 = MTH([-l(6) 0 l(4)], lambda(:,4), q(4));
ref5 = MTH([0 0 l(5)], lambda(:,5), q(5));
ref6 = MTH([0 0 0], lambda(:,6), q(6));

%-- Transformación de espacio entre cada referencial --%
m0_1 = ref1;
m0_2 = m0_1*ref2;
m0_3 = m0_2*ref3;
m0_4 = m0_3*ref4;
m0_5 = m0_4*ref5;
m0_6 = m0_5*ref6; %MTH que relaciona el referencial 6 con la base.

%-- Concatenación de matrices para iteraciones --%
mR_t = [m0_1(1:3,1:3),m0_2(1:3,1:3), m0_3(1:3,1:3), m0_4(1:3,1:3), m0_5(1:3,1:3), m0_6(1:3,1:3)];
mD_t = [m0_1(1:3,4), m0_2(1:3,4), m0_3(1:3,4), m0_4(1:3,4), m0_5(1:3,4), m0_6(1:3,4)];

%-- Jacobiano Geométrico Angular --%
Jw = zeros(3,6);
j = 1;
for i = 1:6
   Jw(:,i) = mR_t(1:3,j:2+j)*lambda(:,i);
   j = j + 3;
end

%-- Jacobiano Geométrico Lineal --%
Jv = zeros(3,6);
for k = 1:6
    Jv(:,k) = cross(Jw(1:3,k),mD_t(1:3,6)-mD_t(1:3,k));
end

%-- Cálculo de velocidades angulares y lineales --%
v0_6 = Jv*dq;          %[mm/s]
w0_6 = Jw*dq;          %[rad/s]
twist = [v0_6; w0_6]
