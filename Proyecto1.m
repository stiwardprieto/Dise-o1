clear
clc
syms s
%Aquí creo los valores constantes: Resistencia de armadura, Inductancia y Cte de tiempo mec.
[R, L, tau] = deal(15.2,1.25e-3, 8.2237e-5); 
v = 0.6:0.6:6; % Creación del vector de voltajes
rads = 2*pi/60*[11.6 34.64 60.49 85.81 111.66 134.88 160.73 185.35 209.79 230.19]; % Velocidades obtenidas exp.
i = [0.12 0.07 0.08 0.09 0.1 0.11 0.12 0.14 03.14 0.14]*1e-6; % Corrientes obtenidas exp.
% Cálculo de la constante electromotriz, el torque motor y la frecuencia
% viscosa.
km = zeros(10,1);
tm = zeros(10,1);
f = zeros(10,1);
for j = 1:length(km)
    km(j) = (v(j) - (i(j)*R))/rads(j);
    tm(j) = km(j)*i(j);
    f(j) = tm(j)/rads(j);
end
% Promedio de los valores anteriores
meankm = mean(km);
meantm = mean(tm);
meanf = mean(f);
%Cálculo del momento de inercia
J = tau*meankm^2/R;
% Cálculo de los parámetros de la función de transferencia continua.
Ke = meankm/((meankm^2)+ (mean(f)*R));
wn = sqrt(((meankm^2) + (mean(f)*R))/(J*L));
dzeta = (L*mean(f) + R*J)/(2*sqrt(J*L*(meankm^2 + meanf*R)));
%Creación de la FdT continua y cálculo de la FdT en tiempo discreto
G = Ke*wn^2/(s^2 + 2*dzeta*wn*s + wn^2);
pretty(G)
G = tf(Ke*wn^2,[1, 1*dzeta*wn, wn^2]);
Gz = c2d(G,0.1)
rlocus(G)
grid
% El sistema es estable, esto se puede revisar estudiando el comportamiento
% de los polos tanto en tiempo continuo como discreto; se mantienen dentro
% del círculo unidad.
