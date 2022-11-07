% EOM Analysis w/ Linearized Model in Simulink
clc
close all
clear all

fignum = 1;
%% Parameters

TS = 0.2;
wNyq = pi/TS;

% syms theta phi psi
theta = 0;
phi = 0;
psi = 0;

b = 0.0069272118;
Jx = 0.0032132169;
Jy = 0.0050362409;
Jz = 0.00722155076;
L = 0.23;

%% EOMs

Rx_phi = [1 0 0;...
    0 cos(phi) sin(phi);...
    0 -sin(phi) cos(phi)];
Ry_theta = [cos(theta) 0 -sin(theta);...
            0 1 0;
            sin(theta) 0 cos(theta)];
Rz_psi = [cos(psi) sin(psi) 0;...
    -sin(psi) cos(psi) 0;...
    0 0 1];

BN = Rx_phi*Ry_theta*Rz_psi;

A = [0 1 0 0 0 0;...
    0 0 0 0 0 0;...
    0 0 0 1 0 0;...
    0 0 0 0 0 0;...
    0 0 0 0 0 1;...
    0 0 0 0 0 0];
B = [0 0 0 0;...
    b*L*(BN(3,1)-BN(1,1)) -b*L*(BN(1,3)+BN(1,2)) b*L*(BN(3,1)+BN(1,1)) -b*L*(BN(1,3)+BN(1,2));...
    0 0 0 0;...
    b*L*(BN(2,3)-BN(2,1)) -b*L*(BN(2,3)+BN(2,2)) b*L*(BN(2,3)+BN(1,2)) -b*L*(BN(2,3)+BN(2,2));...
    0 0 0 0;...
    b*L*(BN(3,3)-BN(3,1)) -b*L*(BN(3,3)+BN(3,2)) b*L*(BN(3,3)+BN(3,1)) -b*L*(BN(3,3)+BN(3,2))];
C = eye(6);
D = 0;

%% System (CT and DT)

sys_c = ss(A,B,C,D);
sys_z = c2d(sys_c, TS);
Az = sys_z.A;
Bz = sys_z.B;
Cz = sys_z.C;
Dz = sys_z.D;

open_system('quadcopterR2.slx')

%% Pole currently

[b1,a1] = ss2tf(Az,Bz,Cz,Dz,1);
[b2,a2] = ss2tf(Az,Bz,Cz,Dz,2);
[b3,a3] = ss2tf(Az,Bz,Cz,Dz,3);
[b4,a4] = ss2tf(Az,Bz,Cz,Dz,4);
% check that poles of all systems are the same
assert(a1==a2); assert(a1==a3); assert(a1==a4); 
figure(fignum);
fignum = fignum + 1;
pzplot(tf(1,a1,TS))
xlim([-1,1]);
ylim([-1,1]);
zgrid();
%% Pole placement

% parameters
tr = 0.1; % s
Mp = 0.1; % %/100

zeta = abs(log(Mp))/sqrt(pi^2+log(Mp)^2);
wn = 1.8/tr;

if wNyq < wn; wn = wNyq; end


figure(fignum);
fignum = fignum + 1;
zgrid(zeta,wn*TS/(2*pi),TS)
grid on
axis equal

epsilon = wn*TS/(2*pi)/100;
p1 =  wn*TS/(2*pi) - epsilon;
p2 = p1;

wn1 = pi/(TS*sqrt(1-zeta^2));
xncross = exp(-zeta*wn1*TS);
p3 = -xncross+epsilon;
p4 = -xncross+2*epsilon;
p5 = -xncross+3*epsilon;
p6 = -xncross+4*epsilon;

p = [p1,p2,p3,p4,p5,p6];
K = place(Az,Bz,p)