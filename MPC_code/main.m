
% system parameters
para.Ld = 1.2e-3;
para.Lq = 0.8e-3;
para.R = 0.1;
para.p = 4;
para.J = 0.02;
para.B = 0.01;
para.mg = 0.15;
para.Kt = 3*para.p*para.mg/2;

para.T = 0.01; %sampling period


% system definition
sys.x0=[0.1; 0.1; 10];  %initial point
A1=[-para.R/para.Ld 0 0;0 -para.R/para.Lq -para.mg/para.Lq;0 para.p*para.Kt/para.J -para.B/para.J];
B1 = [1/para.Ld 0;0 1/para.Lq;0 0]; 
C1=[1 0 0;0 0 1];
D1=zeros(2,2);
sysd=c2d(ss(A1,B1,C1,D1),para.T); %discrete time system
sys.A=sysd.A;
sys.B=sysd.B;
sys.C=sysd.C;
sys.D=sysd.D;
sys.yref=[0;0];

rank1=rank(obsv(sysd.A,sysd.C));   %check observability
rank2=rank(ctrb(sysd.A,sysd.B));   %check controllability
[V,eiga]=eig(sysd.A);              %check eigenvalues of A


%Definition of system dimension
dim.nx=3;     %state dimension
dim.nu=2;     %input dimension
dim.ny=2;     %output dimension
dim.N=5;      %horizon
dim.nd=2;     %disturbance dimension


%Definition of quadratic cost function
N_sim=20;    %simulation horizon
alpha=1;
weight.Q=alpha*diag([1,1,0.1]);                %weight on state
weight.R=10*diag([1,1]);                         %weight on input
weight.P=dare(sys.A,sys.B,weight.Q,weight.R);  %terminal cost
[P,K]=idare(sys.A,sys.B,weight.Q,weight.R);
K=-K;

%bounds
xlb = [-1;-1;-30];
xub = [1;1;30];
ulb = [-3;-3];
uub = [3;3];

% %find Xf
Xn = struct();
V = struct();
Z = struct();
[Xn.('lqr'), V.('lqr'), Z.('lqr')] = findXn(sys.A, sys.B, K, dim.N, xlb, xub, ulb, uub, 'lqr');

%plot Xf
system = LTISystem('A', sys.A, 'B', sys.B);
system.x.min = xlb;
system.x.max = xub;
system.u.min = ulb;
system.u.max = uub;
InvSet = system.invariantSet()

figure()
InvSet.plot()
xlabel('x_1')
ylabel('x_2')
zlabel('x_3')
title('terminal set X_f')

% Generation of prediction model 
predmod=predmodgen(sys,dim,para);            
[H,h]=costgen(predmod,weight,dim);

%constraints
con.umin=-3;
con.umax=3;
con.imax=1;
con.imin=-1;
con.dumax=1;


% record state and optimal input of system
x_rec=zeros(dim.nx,N_sim+1);
u_rec=zeros(dim.nu,N_sim);
x_rec(:,1)=sys.x0;

% solve the optimization problem
for k=1:N_sim 

    eqconstraints=eqconstraintsgen(sys,dim,para);
    [xr,ur]=optimalss(sys,dim,weight,[],eqconstraints);  %update xr and ur in every loop
    sys.x=x_rec(:,k);
    constraints=constraintgen1(dim,con,predmod,sys);
    


    % Solve the constrained optimization problem (with YALMIP)
    u_con = sdpvar(dim.nu*dim.N,1);      %define optimization variable
    %Constraint=constraints.A*u_con>=constraints.b;
    Constraint=(constraints.A1*u_con>=constraints.b1)&(constraints.A3*u_con>=constraints.b3)&(constraints.A2*u_con>=constraints.b2);     %define constraints
    Objective = 0.5*u_con'*H*u_con+(h*[x_rec(:,k); xr; ur])'*u_con;               %define cost function
    optimize(Constraint,Objective);                             %solve the problem
    u_con=value(u_con);                                         %assign the solution
    
    % Select the first input only
    u_rec(:,k)=u_con(1:dim.nu);

    % Compute the state/output evolution
    x_rec(:,k+1)=sys.A*x_rec(:,k) + sys.B*u_rec(:,k);
    clear u_con
end

figure()
stairs(x_rec(3,:))
xlabel('step')
ylabel('\omega [rad/s]')
title('angular speed')

figure()
stairs(u_rec(1,:))
xlabel('step')
ylabel('u_d [V]')
title('input u_1')

figure()
stairs(u_rec(2,:))
xlabel('step')
ylabel('u_q[V]')
title('input u_2')


%% Lyapunov decrease

Vf=[];
lxu=[];
for i=1:N_sim-1
    Vf=[Vf;0.5*x_rec(:,i+1)'*weight.P*x_rec(:,i+1)-0.5*x_rec(:,i)'*weight.P*x_rec(:,i)];
    lxu=[lxu;-0.5*x_rec(:,i)'*weight.Q*x_rec(:,i)-0.5*u_rec(:,i+1)'*weight.R*u_rec(:,i+1)];
end

figure()
stairs(1:size(Vf),Vf);
hold on;
stairs(1:size(lxu),lxu);
legend('-l(x,u)','V_f(f(x,u))-V_f(x)')
title('Lyapunov decrease')


% output MPC with disturbance
sys.Cd=diag([0.5 2]);
sys.Bd=[1 0.5;0.4 0;0 1];
sys.x0d=[0.1;0.2;10];
sys.d=[0.1;1];
sys.yrefd=[0.4;20];

dime.nx=5; %state dimension
dime.nu=2; %input dimension
dime.ny=2; %output dimension
dime.N=5;  %horizon
Ne_sim=30; %simulation horizon


rank3=rank([eye(dim.nx)-sys.A -sys.Bd; sys.C sys.Cd]);

syse.A=[sys.A sys.Bd; zeros(dim.nd,dim.nx) eye(dim.nd)];
syse.B=[sys.B; zeros(dim.nd,dim.nu)];
syse.C=[sys.C sys.Cd];
syse.x0=[sys.x0d; sys.d];
syse.yref=sys.yrefd;

weighte.Q=blkdiag(weight.Q,zeros(dim.nd));            %weight on output
weighte.R=weight.R;                                   %weight on input
weighte.P=blkdiag(weight.P,zeros(dim.nd));            %terminal cost

predmode=predmodgen(syse,dime);  
[He,he]=costgen(predmode,weighte,dime);

% Receding horizon implementation
xe=zeros(dime.nx,Ne_sim+1);
ye=zeros(dime.ny,Ne_sim+1);
ue_rec=zeros(dime.nu,Ne_sim);
xehat=zeros(dime.nx,Ne_sim+1);

xe(:,1)=syse.x0;
xehat(:,1)=[0; 50; 0; 0; 0];
ye(:,1)=syse.C*syse.x0;

L=place(syse.A',syse.C',[0.5; 0.4; 0.45;0.6;0.65])';      %observer gain

constraintsd=constraintgend(dime,con);

for k=1:Ne_sim
    
    xe_0=xe(:,k);  
    dhat=xehat(end-dim.nd+1:end,k);
    
    %Compute optimal ss (online, at every iteration)
    eqconstraintd=eqconstraintsgend(sys,dim,dhat);
    [xrd,urd]=optimalss(sys,dim,weight,[],eqconstraintd); 
    xre=[xrd;dhat];
    
    u_d = sdpvar(dime.nu*dime.N,1);                                 %define optimization variable
    Constraint=constraintsd.A*u_d>=constraintsd.b;                         %define constraints
    Objective = 0.5*u_d'*He*u_d+(he*[xe_0; xre; urd])'*u_d;    %define cost function
    optimize(Constraint,Objective);                                    %solve the problem
    u_d=value(u_d);      

    % Select the first input only
    ue_rec(:,k)=u_d(1:dim.nu);

    % Compute the state/output evolution
    xe(:,k+1)=syse.A*xe_0 + syse.B*ue_rec(:,k);
    ye(:,k+1)=syse.C*xe(:,k+1);
    clear u_d
        
    % Update extended-state estimation
    xehat(:,k+1)=syse.A*xehat(:,k)+syse.B*ue_rec(:,k)+L*(ye(:,k)-syse.C*xehat(:,k));
    
end

figure()
stairs(syse.yref(2)*ones(1,size(ye,2))-ye(2,:))
xlabel('step')
ylabel('\omega [rad/s]')
title('tracking error')

figure()
stairs(ue_rec(1,:))
xlabel('step')
ylabel('U[V]')
title('input(u_1)')

figure()
stairs(ue_rec(2,:))
xlabel('step')
ylabel('U[V]')
title('input(u_1)')


%% adaptive MPC

%system definition
sysad.x0=[0; 0; 0];
sysad.A = [1-para.T*para.R/para.Ld 0 0;0 1-para.T*para.R/para.Lq -para.T*para.mg/para.Lq;0 para.T*para.p*para.Kt/para.J 1-para.T*para.B/para.J]; 
sysad.B = [para.T/para.Ld 0;0 para.T/para.Lq;0 0];               
sysad.C=[1 0 0;0 0 1];
sysad.D=zeros(2,2);
sysad.yref=[10;50];
%constraints
con.umin=0;
con.umax=100;
con.imax=3;
con.imin=0;
para.T=0.1;
sysa=ss(sysad.A,sysad.B,sysad.C,sysad.D,para.T);
mpcobj = mpc(minreal(sysa), para.T);
mpcobj.PredictionHorizon = 6;
mpcobj.ControlHorizon = 3;
mpcobj.Weights.OutputVariables = [1 10000];
mpcobj.Weights.ManipulatedVariablesRate = [10 10];
mpcobj.OutputVariables(2).Min = con.imin;
mpcobj.OutputVariables(2).Max = con.imax;
mpcobj.OutputVariables(1).Min = con.imin;
mpcobj.OutputVariables(1).Max = con.imax;
mpcobj.ManipulatedVariables(1).Min = con.umin;
mpcobj.ManipulatedVariables(1).Max = con.umax;
mpcobj.ManipulatedVariables(2).Min = con.umin;
mpcobj.ManipulatedVariables(2).Max = con.umax;

Nsima = 50;
t = linspace(0, Nsima*para.T, Nsima+1);
r = [3*ones(size(t));3*sin(2*pi*0.2*(t-1))+50*ones(size(t))];
mpcobj.Model.Nominal = struct('y', [0;0;0], 'u', [0;0]);
nominal=mpcobj.Model.Nominal;
xmpc = mpcstate(mpcobj);
x = xmpc.Plant;
YY = []; RR = []; UU = []; XX = [];

for k = 1:Nsima
    XX = [XX;x']; % store plant state
    y = sysad.C*x; % calculate plant output
    YY = [YY;y']; % store plant output
    RR = [RR;r(:,k)']; % store reference
    u = mpcmoveAdaptive(mpcobj,xmpc,sysa,nominal,y,r(:,k)); % calculate optimal mpc move
    UU = [UU;u']; % store plant input
    x = sysad.A*x+sysad.B*u; % update plant state
    % is the last line necessary since x=xmpc.Plant gets updated anyway
end

% r = [3*ones(size(t));3*ones(size(t));3*sin(2*pi*0.2*t)+50*ones(size(t))]; 
figure();
plot(t(1:end-1), YY(:,2));
hold on;
plot(t,r(2,:));
title('Adaptive MPC');
xlabel('t(s)');
ylabel('angular speed(rad/s)');
legend('output', 'reference');
figure();
plot(t(1:end-1), UU(:,1),t(1:end-1),UU(:,2));
legend('controled input 1', 'controled input 2');
title('controled input');
xlabel('t(s)');
ylabel('V');



