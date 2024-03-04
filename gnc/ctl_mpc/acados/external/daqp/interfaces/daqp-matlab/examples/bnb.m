%% Setup QP data
n = 250;
m = 400;
ms = 20;
me = 0;
M = randn(n,n);
H = M'*M;
min(eig(H))
%H(1:ms,1:ms) = 0.01*eye(ms);
%H(ms+1:end,1:ms) = 0; 
%H(1:ms,ms+1:end) = 0;
%H = eye(4);
f = 100*randn(n,1); 
f(1:ms) = -sign(f(1:ms)).*f(1:ms);
A = randn(m,n);
bupper = 20*rand(m,1);
blower = -20*rand(m,1);
bupper(ms+1:ms+me)=0;
blower(ms+1:ms+me)=0;

% Normalize
%M_norms = vecnorm(A*inv(chol(H)),2,2);
%A = A./M_norms;
%bupper = bupper./M_norms;
%blower = blower./M_norms;

bupper_tot = [ones(ms,1);bupper];
blower_tot = [zeros(ms,1);blower];
sense = int32(zeros(m+ms,1));
sense(1:ms) = sense(1:ms)+16;
sense(ms+1:ms+me) = sense(ms+1:ms+me)+5;

%% Solve QP (quadprog interface)
[xstar,fval,exitflag,info] = daqp.quadprog(H,f,A,bupper_tot,blower_tot,sense);
fval = 0.5*xstar'*H*xstar+f'*xstar;
exitflag
%% Gurobi
model.Q = 0.5*sparse(H);
model.A = sparse([A;-A(me+1:end,:)]);
model.rhs = [bupper;-blower(me+1:end)];
model.obj = f;
model.sense=repelem('=<',[me,2*(m-me)]);
model.vtype=repelem('BC',[ms,n-ms]);
model.lb = -inf(n,1);

params.Threads=1;
results = gurobi(model,params)

xgrb = results.x;
fgrb = 0.5*xgrb'*H*xgrb+f'*xgrb;

%% Compare 
norm(xstar-xgrb)
fgrb-fval
[info.solve_time results.runtime]
[info.iter results.itercount]
%% Slack
bupper(1:me)-A(1:me,:)*xstar
Rinv = inv(chol(H))
M = A*Rinv
v = Rinv'*f
dupper = bupper+M*v
dlower = blower+M*v

