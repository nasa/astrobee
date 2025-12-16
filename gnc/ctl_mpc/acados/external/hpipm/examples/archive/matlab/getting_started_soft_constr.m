% pyversion /usr/bin/python3
% py.sys.path

clear
close all
clc

%addpath('~/pytave') % or change with your pytave main folder path
addpath('../../interfaces/matlab/hpipm_matlab')

%octave_import hpipm_matlab.*
import hpipm_matlab.*



% dims
N = 20;
nx = 4;
nu = 2;
ng = 2;

tic
dims = hpipm_ocp_qp_dim(N);
tmp_time = toc;
fprintf('create dim time %e\n', tmp_time);

% set dimensions
tic
dims.set_nx(nx*ones(1,N+1));
tmp_time = toc;
fprintf('set nx time %e\n', tmp_time);
dims.set_nu(nu*ones(1,N));

dims.set_nbx(nx*ones(1,N+1));
dims.set_nbu(nu*ones(1,N));

dims.set_ng(ng*ones(1,N+1));
dims.set_ns(ng*ones(1,N+1));

dims.print_C_struct();


% Dynamics, double integrator in X and Y direction
A0 = [1, 1; 
      0, 1];
B0 = [0; 1];

A = blkdiag(A0,A0);
B = blkdiag(B0,B0);

% cost 
Q = eye(nx);
S = zeros(nu,nx);
R = eye(nu);
q = [1;0;1;0];

% bounds on all states and inputs
Jx = eye(nx); % can be used to only inforce bounds on certain states
Ju = eye(nu);
x0 = [1; 1; 1; 1];
xN = [0; 0; 0; 0];
ubx= [ 3; 3; 3; 3];
lbx= [-3;-3;-3;-3];
ubu= [ 1; 1];
lbu= [-1;-1];

% polytopic constraints
% -0.5 <= v_X + a_X <= 0.5
% -0.5 <= v_Y + a_Y <= 0.5
C = [0,1,0,0;
     0,0,0,1];
D = [1,0;
     0,1];
lg = [-1;-1];
ug = [ 1; 1];
Jsg = eye(2);
Z = 100*eye(ng);
z = 100*ones(ng,1);

% qp
tic
qp = hpipm_ocp_qp(dims);
tmp_time = toc;
fprintf('create qp time %e\n', tmp_time);

tic

% set dynamics
for i = 0:N-1
    qp.set('A', A,i);
    qp.set('B', B,i);
end
tmp_time = toc;
%alternative 
% qp.set('A', {A,A,A,A,...})

% set cost
for i = 0:N
    qp.set('Q', Q,i);
    qp.set('q', q,i);
    if i<N
       qp.set('R', R,i); 
%        qp.set('S', S,i);
    end
end

% set bounds and initial and terminal constraint
for i = 0:N
    qp.set('Jx', Jx,i)
    if i == 0
        qp.set('lx', x0,0)
        qp.set('ux', x0,0)
    elseif i == N
        qp.set('lx', xN,N)
        qp.set('ux', xN,N)
    else
        qp.set('lx', lbx,i)
        qp.set('ux', ubx,i)
    end
    
    if i<N
        qp.set('Ju', Ju,i)
        qp.set('lu', lbu,i)
        qp.set('uu', ubu,i) 
    end
end

% set polytopic soft constraints
for i = 0:N
    qp.set('C', C,i)
    if i<N
        qp.set('D', D,i)
    end
    qp.set('lg', lg,i)
    qp.set('ug', ug,i)
    qp.set('Jsg', Jsg,i)
    
    qp.set('Zl', Z,i);
    qp.set('zl', z,i);
    qp.set('Zu', Z,i);
    qp.set('zu', z,i);
end

qp.print_C_struct()



% qp sol
tic
qp_sol = hpipm_ocp_qp_sol(dims);
tmp_time = toc;
fprintf('create qp_sol time %e\n', tmp_time);



% set up solver arg
tic
arg = hpipm_ocp_qp_solver_arg(dims);
tmp_time = toc;
fprintf('create solver arg time %e\n', tmp_time);

arg.set_mu0(1e1);
arg.set_iter_max(200);
arg.set_tol_stat(1e-4);
arg.set_tol_eq(1e-5);
arg.set_tol_ineq(1e-5);
arg.set_tol_comp(1e-5);
arg.set_reg_prim(1e-12);


% set up solver
tic
solver = hpipm_ocp_qp_solver(dims, arg);
tmp_time = toc;
fprintf('create solver time %e\n', tmp_time);



% solve qp
tic
return_flag = solver.solve(qp, qp_sol);
tmp_time = toc;
fprintf('solve time %e\n', tmp_time);

fprintf('HPIPM returned with flag %d', return_flag);

if return_flag==0
    fprintf('-> QP solved! Solution:\n')
    qp_sol.print_C_struct()
else
    fprintf('-> Solver failed!')
end


u_opt = zeros(nu,N);
x_opt = zeros(nx,N+1);
for i=0:N-1
    u_opt(:,i+1) = qp_sol.get_u(i);
end
for i=0:N
	x_opt(:,i+1) = qp_sol.get_x(i);
end

figure(1)
subplot(3,2,1)
plot(x_opt(1,:))
subplot(3,2,3)
plot(x_opt(2,:))
subplot(3,2,5)
stairs(u_opt(1,:))

subplot(3,2,2)
plot(x_opt(3,:))
subplot(3,2,4)
plot(x_opt(4,:))
subplot(3,2,6)
stairs(u_opt(2,:))

