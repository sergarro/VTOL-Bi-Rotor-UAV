%% TRABAJO FINAL DE GRADO
% DISEÑO E IMPLEMENTACIÓN DEL SISTEMA DE CONTROL MODAL DE UNA AERONAVE NO
% TRIPULADA TIPO VTOL.

% %% Arrancar FlightGear
% START_VSkye 

%% Centro de gravedad e inercias locales
clc;
clear all;

%CARACTERÍSTICAS CONSTRUCTIVAS

dm=0.35;                            %distancia motores a eje longitudinal
mM=0.071;                           %Masa motor
densCuerpo=1.9*0.45359237/0.3048^3; %kg/m3. Densidad EPO FOAM 
b=1.2;                              %largo cuerpo
h=.5;                               %alto cuerpo
e=3e-2;                             %ancho cuerpo
%mCuerpo=densCuerpo*1/2*b*h*e;      %Masa cuerpo
mCuerpo=190.43e-3;
mR=.416;                            %Masa resto
mTotal = 2*mM + mCuerpo +  mR;      %Masa total

%POSICIONES CENTRO DE GRAVEDAD INDIVIDUAL (x,y)==(punta2der,punta2base).

rM1      = [-dm h 0]; %(Motor right in global axis)
rM2      = [ dm h 0]; %(Motor left in global axis)
%rCuerpo  = [ 0  2/3*h 0];
rCuerpo  = [ 0  238.27 3.74]*1e-3;
rR       = [ 0  0.125 0];

%CENTRO DE MASAS TOTAL

CM       = (rM1*mM + rM2*mM + rCuerpo*mCuerpo + rR*mR)/mTotal;

%DISTANCIAS PIEZAS Y CM. Ejes (x,y)==(punta2der,punta2base)
r_M1CM      = rM1-CM; %posicion Motor 1 (Motor right in global axis) respecto CM del cnjunto
d_M1CM      = norm(r_M1CM,2);
r_M2CM      = rM2-CM;  %posicion Motor 2 (Motor left in global axis) respecto CM del cnjunto
d_M2CM      = norm(r_M2CM,2);
r_CuerpoCM  = rCuerpo-CM;  %posicion cuerpo respecto CM del cnjunto
d_CuerpoCM  = norm(r_CuerpoCM,2);
r_RCM       = rR-CM; %posicion resto respecto CM del cnjunto
d_RCM       = norm(r_RCM,2);

%INERCIAS INDIVIDUALES (x,y)==(C.G.Trian2der,C.G.Trian2abajo)
% IxxCuerpo=densCuerpo*e*b*h*(h^2/36 + e^2/24);
% IyyCuerpo=densCuerpo*e*b*h*(b^2/48 + e^2/24);
% IzzCuerpo=densCuerpo*e*b*h*(b^2/48 + h^2/36);

%INERCIAS INDIVIDUALES SOLIDWORKS (x,y)==(C.G.Trian2der,C.G.Trian2base)
IxxCuerpo= 1545984.20e-9;
IyyCuerpo=16023277.17e-9;
IzzCuerpo=17552680.51e-9;
PxzCuerpo=-14181.65e-9;

%% CONVERSION A EJES LOCALES AERONAVE (BODY FRAME)

%INERCIAS TOTALES EN EJES LOCALES AERONAVE (BODY FRAME)
Izz=IyyCuerpo + r_CuerpoCM(1)^2*mCuerpo + r_M1CM(1)^2*mM + ...
    r_M2CM(1)^2*mM + r_RCM(1)^2*mR;

Iyy=IxxCuerpo + r_CuerpoCM(2)^2*mCuerpo + r_M1CM(2)^2*mM + ...
    r_M2CM(2)^2*mM + r_RCM(2)^2*mR;

Ixx=IzzCuerpo + d_CuerpoCM^2*mCuerpo + d_M1CM^2*mM + ...
    d_M2CM^2*mM + d_RCM^2*mR;

Pxz=PxzCuerpo + (r_CuerpoCM(2)*r_CuerpoCM(3)*mCuerpo + ...
    r_M1CM(2)*r_M1CM(3)*mM + r_M2CM(2)*r_M2CM(3)*mM + ...
    r_RCM(2)*r_RCM(3)*mR);

Inertia=[[Ixx 0 -Pxz];[0 Iyy 0];[-Pxz 0 Izz]];

%POSICION MOTORES EN EJES LOCALES (BODY FRAME)

r_MotorRight=[r_M1CM(3) -r_M1CM(1) -r_M1CM(2)];
r_MotorLeft=[r_M2CM(3) -r_M2CM(1) -r_M2CM(2)];


%% Initial conditions
Vel_0=[0 0 0];
Pos_0=[0 0 0];
h0=0.281600391490859;
V_air_0=sqrt(Vel_0(1)^2 + Vel_0(2)^2 + Vel_0(1)^2);
pqr_0=[0 0 0];
euler_0=[0*pi/180 0*pi/180 0];
theta_0=0.110657221173896;
deflect_sat=[0.4 0.4 0.4];
alpha_0=0;
beta_0=0;
local_V_0 = [0 0 0];
altitude_0 = 200;
airspeed_0 = 0;

experimentN=5;
SimVars.chirp=false;
SimVars.wind=true;
SimVars.isVariable=true;
SimVars.windValue=[5 -20 180];

%% Lectura de la trayectoria
Posx=load('Posx.mat');
Posx=struct2cell(Posx);
Posx2=transpose(cell2mat(Posx));

Posy=load('Posy.mat');
Posy=struct2cell(Posy);
Posy2=transpose(cell2mat(Posy));

Posz=load('Posz.mat');
Posz=struct2cell(Posz);
Posz2=transpose(cell2mat(Posz));

t=load('tc.mat');
t=struct2cell(t);
t2=cell2mat(t);

% Montamos el vector
Posx3=timeseries(Posx2,t2);
Posy3=timeseries(Posy2,t2);
% Posz3=timeseries(Posz2,t2);
Posz3=timeseries(-200,t2);

%% Execute the Simulink model
simOut = sim("Comprobar_Posicion_PID_ISA.slx");