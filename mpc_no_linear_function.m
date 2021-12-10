function [p_o,v_o] = mpc_no_linear_function(p,v,n)
%function v_o = mpc_no_linear_function(p,v,n)

minimum_speed = 3;

% Initial state x(0)
X0=[p(1);p(2);p(3)];
%X0=[0;0;0];
% diff_now = sqrt((n(1)-p(1))^2 + (n(2)-p(2))^2);
% diff_ori = sqrt((n(1)-p_temp(1))^2 + (n(2)-p_temp(2))^2);


vk=v(1);
Ts=0.01;
%thetak=(120*pi)/180;
thetak=p(3); %% 안되면 0으로 바꿔보기
wk=0;
D=zeros(3,1);
%D=[-0.01;-0.01;-0.01];
N=5;
Xr=[n(1) n(2) 0]';
Xr(3)=(atan((n(2)-p(2))/(n(1)-p(1))));
Xr(3) = 0;
% Define cost functionx| and expected disturbances
Q=[50 0  0;
    0  50 0;
    0  0  1000000];
R=[1  0;
   0  1];
W=ones(1,N)';  % expected demand (this is just an example)

[A,B,C]=model_system_test(vk,thetak,Ts);
[A,B,D,Q]=increase_matrixDUQ(A,B,D,Q);
Q(4,4)=5;
Q(5,5)=5;

% Build R_hat
R_hat = kron(eye(N),R);
% Build Q_hat
Q_hat=kron(eye(N),Q);

% Constraints
Ax=[1  0  0;
    0  1  0;
    -1  0  0;
    0 -1  0];
bx=100 * [150 150 150 150]';
Au=[ 1  0;
    0  1;
    -1  0;
    0 -1];
bu=[10 4 10 4]';
Axaum=[Ax                             zeros(size(Ax,1),size(Au,2));
    zeros(size(Au,1),size(Ax,2))   Au                          ];
bxaum=[bx; bu];
Ax=Axaum;
bx=bxaum;

% Transform into U constraints
Au_hat=kron(eye(N),Au);
bu_hat=kron(ones(N,1),bu);
Ax_hat=kron(eye(N),Ax);
bx_hat=kron(ones(N,1),bx);

% Delta U
% Aggregated U constraints
%AU=[Ax_hat*Gu; Au_hat];
%bU=[bx_hat-Ax_hat*Gx*X0-Ax_hat*Gw*W;bu_hat];

% MPC into action
%Xhist=X0;
%Uhist=[];
%VK=vk;
%THK=thetak;
%Disturb= normrnd(0.5,1,Simlength+N,1); %Longer than simulation for prediction horizon

% Simulation loop
%XR=[];
u=[v(1);v(2)];
D=zeros(3,1);
path=createPath_test(n);
i=1;
delta=0.1;

% expected disturbances (force that they are different)
%W=0*Disturb(k:k+N-1)+0*normrnd(0,0.2,N,1); % == 0

% Update controller matrices for current state and disturbances (H and Au are constant)
[A,B,C]=model_system_test(vk,thetak,Ts);

[Xr,i]=createReferenceDU(path,i,X0,B,vk,Ts,N,delta);

UMPC=MPC_DU_test(A,B,D,N,W,X0,Xr,Q_hat,R_hat,Au_hat,bu_hat,Ax_hat,bx_hat,u);

% Apply only first component
u = UMPC(1:size(B,2)) + u;
%u = UMPC(3:4) + u;

v_o = u';
if v_o(1) < minimum_speed && v_o(1) > 0
    v_o(1) = minimum_speed;
    disp("mini!");
elseif v_o(1) > -minimum_speed && v_o(1) < 0
    v_o(1) = -minimum_speed;
    disp("mini!");
end

%     X1=linearModel(A,B,D,u,0,X0);
X1=nonlinearModel(0,u,W(1),X0,thetak,0,vk,Ts);
p_o = X1;

end