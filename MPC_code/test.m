clc 
clear all

% xlb = [-3;-3;-50];
% xub = [3;3;50];
% ulb = [-20;-20];
% uub = [20;20];
% 
% para.Ld = 1.2e-3;
% para.Lq = 0.8e-3;
% para.R = 0.1;
% para.p = 4;
% para.J = 0.02;
% para.B = 0.01;
% para.mg = 0.15;
% para.Kt = 3*para.p*para.mg/2;
% para.T = 0.05; 
% 
% A = [1-para.T*para.R/para.Ld 0 0;0 1-para.T*para.R/para.Lq -para.T*para.mg/para.Lq;0 para.T*para.p*para.Kt/para.J 1-para.T*para.B/para.J];
% B = [para.T/para.Ld 0;0 para.T/para.Lq;0 0];
% C=[1 0 0;0 0 1];
% Q=diag([1,1,1]);
% R=eye(2);
% A1=[-para.R/para.Ld 0 30;0 -para.R/para.Lq -para.mg/para.Lq;-10 para.p*para.Kt/para.J -para.B/para.J];
% B1 = [1/para.Ld 0;0 1/para.Lq;0 0]; 
% C1=[1 0 0;0 0 1];
% D1=zeros(2,2);
% sysd=c2d(ss(A1,B1,C1,D1),para.T);
% 
% 
% a=rank(obsv(sysd.A,sysd.C));
% b=rank(ctrb(sysd.A,sysd.B));
% [V,D]=eig(sysd.A);
dime.nx=5; %state dimension
dime.nu=2; %input dimension
dime.ny=2; %output dimension
dime.N=5;  %horizon
con.dumax=1;
A=[];
for i=1:(dime.nu*dime.N-2)
    A=[A;zeros(1,i-1) 1 0 -1 zeros(1,dime.nu*dime.N-2-i)];
end
constraints.A=[A;-1.*A];
constraints.b= -con.dumax*ones(size(A,1),1);