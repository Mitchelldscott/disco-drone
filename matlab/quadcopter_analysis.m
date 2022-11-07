% EOM Analysis w/ Linearized Model in Simulink
clc
close all
clear all


%% Parameters

TS = 0.2;

% syms theta phi psi
theta = 0;
phi = 0;
psi = 0;

b = 0.0069272118;
Jx = 0.0032132169;
Jy = 0.0050362409;
Jz = 0.00722155076;
L = 0.23;

% n11 = cos(theta);
% n12 = -sin(theta)*sin(psi);
% n13 = -sin(theta)*cos(psi);
% n21 = -sin(phi)*sin(theta);
% n22 = cos(phi)*cos(psi)-sin(phi)*cos(theta)*sin(psi);
% n23 = -cos(phi)*sin(psi)-sin(phi)*cos(theta)*cos(psi);
% n31 = cos(phi)*sin(theta);
% n32 = sin(phi)*cos(psi)+cos(phi)*cos(theta)*sin(psi);
% n33 = -sin(theta)*sin(psi)+cos(phi)*cos(theta)*cos(psi);

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

sys_c = ss(A,B,C,D);
sys_z = c2d(sys_c, TS);
Az = sys_z.A;
Bz = sys_z.B;
Cz = sys_z.C;
Dz = sys_z.D;

open_system('quadcopterR2.slx')

%% Pole placement

% parameters
tr = 0.1; % s
Mp = 0.1; % %/100

zeta = abs(log(Mp))/sqrt(pi^2+log(Mp)^2);
wn = 1.8/tr;

figure(1);
zgrid(zeta,wn*TS/(2*pi),TS)
grid on
axis equal

epsilon = wn*TS/(2*pi)/100;
p1 =  wn*TS/(2*pi) - epsilon;
p2 = p1;

wn1 = pi/(TS*sqrt(1-zeta^2))
xncross = exp(-zeta*wn1*TS);
p3 = -xncross+epsilon;
p4 = p3;
p5 = -xncross+2*epsilon;
p6 = p5;

p = [p1,p2,p3,p4,p5,p6];
K = place(Az,Bz,p)