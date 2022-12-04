% EOM Analysis w/ Linearized Model in Simulink
clc
close all
clear all

fignum = 1;
figshow = 0;
%% Parameters

TS = 0.02; % 50 Hz
wNyq = pi/TS;
ref = [0; 0; 0; 0]; % Reference Input

% syms theta phi psi
phi = 0;
theta = 0;
psi = 0;

b = 0.0254478822; % 0.0069272118;
Jx = 0.0032132169;
Jy = 0.0050362409;
Jz = 0.00722155076;
L = 0.23;

%% EOMs

Rx_phi = [1 0 0;...
    0 cos(phi) sin(phi);...
    0 -sin(phi) cos(phi)];
Ry_theta = [cos(theta) 0 -sin(theta);...
            0 1 0;...
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
    b*L*(BN(1,3)-BN(1,1)) -b*L*(BN(1,3)+BN(1,2)) b*L*(BN(1,3)+BN(1,1)) b*L*(BN(1,2)-BN(1,3));...
    0 0 0 0;...
    b*L*(BN(2,3)-BN(2,1)) -b*L*(BN(2,3)+BN(2,2)) b*L*(BN(2,3)+BN(2,1)) b*L*(BN(2,2)-BN(2,3));...
    0 0 0 0;...
    b*L*(BN(3,3)-BN(3,1)) -b*L*(BN(3,3)+BN(3,2)) b*L*(BN(3,3)+BN(3,1)) b*L*(BN(3,2)-BN(3,3))];
C = eye(6);
D = 0;

%% System (CT and DT)

sys_c = ss(A,B,C,D);
sys_z = c2d(sys_c, TS);
Az = sys_z.A;
Bz = sys_z.B;
Cz = sys_z.C;
Dz = sys_z.D;

Cc = ctrb(sys_z);
assert(rank(Cc)==6);
%% Pole currently

[b1,a1] = ss2tf(Az,Bz,Cz,Dz,1);
[b2,a2] = ss2tf(Az,Bz,Cz,Dz,2);
[b3,a3] = ss2tf(Az,Bz,Cz,Dz,3);
[b4,a4] = ss2tf(Az,Bz,Cz,Dz,4);
% check that poles of all systems are the same
for i = 1:length(a1)
    assert(a1(i)==a2(i));
    assert(a1(i)==a3(i));
    assert(a1(i)==a4(i)); 
end
if(figshow)
    figure(fignum);
    fignum = fignum + 1;
    pzplot(tf(1,a1,TS))
    xlim([-1,1]);
    ylim([-1,1]);
    zgrid();
end

%% Pole placement

% parameters
tr = 0.1; % s
Mp = 0.2; % %/100

zeta = abs(log(Mp))/sqrt(pi^2+log(Mp)^2);
wn = 1.8/tr;

if wNyq < wn; disp('wn = '); disp(wn); disp('wn set to wNyq = '); disp(wNyq); wn = wNyq; end

if(figshow)
    figure(fignum);
    fignum = fignum + 1;
    zgrid(zeta,wn*TS/(2*pi),TS)
    grid on
    axis equal
end

epsilon = wn*TS/(2*pi)/10;
p1 =  1-wn*TS/(2*pi) - epsilon - 0.5;
p2 = p1;

wn1 = pi/(TS*sqrt(1-zeta^2));
xncross = exp(-zeta*wn1*TS);
p3 = -xncross+10*epsilon;
p4 = -xncross+20*epsilon;
p5 = -xncross+30*epsilon;
p6 = -xncross+40*epsilon;
% p3 = epsilon;
% p4 = 2*epsilon;
% p5 = 3*epsilon;
% p6 = 4*epsilon;

p = [p1,p2,p3,p4,p5,p6];
K = place(Az,Bz,p);
Ks = K*1e-6*2*pi;
disp('Feedback gain, K: '); disp(K);

z = tf('z',TS);
if(figshow)
    figure(fignum);
    fignum = fignum + 1;
    pzplot(1/((z-p1)*(z-p2)*(z-p3)*(z-p4)*(z-p5)*(z-p6)))
    xlim([-1,1]);
    ylim([-1,1]);
    zgrid();
end

%% lqr attempt
Q = [10 0 0 0 0 0;...
    0 100 0 0 0 0;...
    0 0 10 0 0 0;...
    0 0 0 100 0 0;...
    0 0 0 0 10 0;...
    0 0 0 0 0 100];
R = [1 0 0 0;...
    0 1 0 0;...
    0 0 1 0;...
    0 0 0 1];
[Kd,s,e] = dlqr(Az,Bz,Q,R);
Kds = 0.01*Kd;
%% Open Simulink
% initial conditions for Simulink
phi_init = 0;
theta_init = 0;
psi_init = 0.6;
% 
% open_system('quadcopterR2.slx')
open_system('quadcopterR2Feedback.slx')