%% Parte 1 Transfer funcion

syms s;

Mb=0.267;  %masa vehiculo [kg]
Mw=0.0648; %masa rueda [kg]
r=0.033; %wheel radius [m]
L=0.07; %distancia rueda centro masa [m]
Ke=0.0259;	%DC motor back EMF constant [Vs/rad]
Jw=0.5*Mw*r^2; %wheel inertia [kgm^2]
h=0.15; %altura cuerpo [m]
w=0.1; %ancho cuerpo [m]
Jb=Mb*(h^2+w^2)/12; %body pitch inertia [kgm^2]
Km=0.0259;	%torque constant   [Nm/A]
R=4.4;	%DC motor resistance [Ohm]
g=9.78; %gravitional constant [m/s^2]
b=0.001; %coefficient friccion [Nm*s/rad]

alpha=2*(R*b-Ke*Km)*(Mb*L^2+Mb*r*L+Jb)/(R*(2*(Jb*Jw+Jw*L^2*Mb+Jb*Mw*r^2+L^2*Mb*Mw*r^2)+Jb*Mb*r^2));
beta=-L^2*Mb^2*g*r^2/(Jb*(2*Jw+Mb*r^2+2*Mw*r^2)+2*Jw*L^2*Mb+2*L^2*Mb*Mw*r^2);  
gamma=2*(R*b-Ke*Km)*(2*Jw+Mb*r^2+2*Mw*r^2+L*Mb*r)/(R*r*(2*(Jb*Jw+Jw*L^2*Mb+Jb*Mw*r^2+L^2*Mb*Mw*r^2)+Jb*Mb*r^2));  
sigma=L*Mb*g*(2*Jw+Mb*r^2+2*w*r^2)/(2*Jb*Jw+2*Jw*L^2*Mb+Jb*Mb*r^2+2*Jb*Mw*r^2+2*Jw*Mw*r^2+2*L^2*Mb*Mw*r^2);
epsilon=Km*r/(R*b-Ke*Km);

A=[0, 1, 0, 0;0, alpha, beta, -r*alpha;0, 0, 0, 1;0, gamma, sigma, -r*gamma];

C=[0, 0, 0, 0;0,0,0,1];

B=[0; alpha*epsilon; 0; gamma*epsilon];

D=[0;0];

I=[1,0,0,0;0,1,0,0;0,0,1,0;0,0,0,1];

G=C*(s*I-A)^(-1)*B;

%% Parte 2 PID control
s= tf('s');
P=- (91437362481614812850006314518996*s)/(- 633825300114114700748351602688*s^3 + 2565456747070833747539659849728*s^2 + 103976824631520494064629030249873*s - 1059578919836250568028746067908992) - (11584440332222154*s*(1125899906842624*s - 7893118688460203))/(- 633825300114114700748351602688*s^3 + 2565456747070833747539659849728*s^2 + 103976824631520494064629030249873*s - 1059578919836250568028746067908992)
  step(P)
%kp=50;
%ki=10;
%kd=200;
%C=pid(kp,ki,kd);
%T=feedback(C*P,1);
%t=0:0.1:100;
%step(T,t)

%% Parte 3 LQR
syms s;

Mb=0.3359;  %masa vehiculo [kg]
Mw=0.0648; %masa rueda [kg]
r=0.033; %wheel radius [m]
L=0.07; %distancia rueda centro masa [m]
Ke=0.0259;	%DC motor back EMF constant [Vs/rad]
Jw=0.5*Mw*r^2; %aproximado %wheel inertia [kgm^2]
h=0.15; %altura cuerpo [m]
w=0.1; %ancho cuerpo [m]
Jb=Mb*(h^2+w^2)/12; %aproximado %body pitch inertia [kgm^2]
Km=0.0259;	%torque constant   [Nm/A]
R=4.4;	%DC motor resistance [Ohm]
g=9.78; %gravitional constant [m/s^2]
b=0.001; %coefficient friccion [Nm*s/rad]
m=64.8/1000;  %wheel weight aproximado (sin nada 50g)

alpha=2*(R*b-Ke*Km)*(Mb*L^2+Mb*r*L+Jb)/(R*(2*(Jb*Jw+Jw*L^2*Mb+Jb*Mw*r^2+L^2*Mb*Mw*r^2)+Jb*Mb*r^2));
beta=-L^2*Mb^2*g*r^2/(Jb*(2*Jw+Mb*r^2+2*Mw*r^2)+2*Jw*L^2*Mb+2*L^2*Mb*Mw*r^2);  
gamma=2*(R*b-Ke*Km)*(2*Jw+Mb*r^2+2*Mw*r^2+L*Mb*r)/(R*r*(2*(Jb*Jw+Jw*L^2*Mb+Jb*Mw*r^2+L^2*Mb*Mw*r^2)+Jb*Mb*r^2));  
sigma=L*Mb*g*(2*Jw+Mb*r^2+2*w*r^2)/(2*Jb*Jw+2*Jw*L^2*Mb+Jb*Mb*r^2+2*Jb*Mw*r^2+2*Jw*Mw*r^2+2*L^2*Mb*Mw*r^2);
epsilon=Km*r/(R*b-Ke*Km);

A=[0, 1, 0, 0;0, alpha, beta, -r*alpha;0, 0, 0, 1;0, gamma, sigma, -r*gamma];

C=[0, 0, 0, 0;0,0,0,1];

B=[0; alpha*epsilon; 0; gamma*epsilon];

D=[0;0];

% estable:
I=zeros(4);
I(1,1)=1;
I(2,2)=1;
I(3,3)=1;
I(4,4)=1;
disp('determinante para estable');
det(s*I-A)          %es estable como los polos estan en 0

disp('rango para controlable');
co=[B A*B A^2*B A^3*B];  %controlabilidad
rank(co)        %es controlable como el rango es 4

Q=[200,0,0,0;0 30 0 0;0 0 1 0;0 0 0 1];
R=1;
[K,S,e] = lqr(A,B,Q,R);      %K ganancia
disp('ganancias K');
K


Ac = [(A-B*K)];
Bc = [B];
Cc = [C];
Dc = [D];

estados = {'x' 'x_p' 'phi' 'phi_p'};
entradas = {'r'};
salidas = {'x'; 'phi'};

sys_ctl = ss(Ac,Bc,Cc,Dc);

step(sys_ctl)

%% Parte 4 funcion de transferencia experimental 
Datos=importdata('-pastespecial');      %importar workspace
Angulo=Datos(:,1);                      %Angulo
Voltaje=Datos(:,2);                     %Voltaje
ident                                   %TF experimental
step(tf1)                               %dibujar TF
pidTuner                                %mejorar TF