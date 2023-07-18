% pyversion /usr/bin/python3
% py.sys.path

clear all
close all
clc

%addpath('~/pytave') % or change with your pytave main folder path
addpath('../../interfaces/matlab/hpipm_matlab')

%octave_import hpipm_matlab.*
import hpipm_matlab.*



% dims
nx = 8;				% number of states
nu = 3;				% number of inputs (controls)
N = 30;				% horizon length

tic
dims = hpipm_ocp_qp_dim(N);
tmp_time = toc;
fprintf('create dim time %e\n', tmp_time);

tic
for ii=0:N
	dims.set('nx', nx, ii);
end
tmp_time = toc;
fprintf('set nx time %e\n', tmp_time);
for ii=0:N-1
	dims.set('nu', nu, ii);
end
% nbx
dims.set('nbx', nx, 0);
% nbu
for ii=0:N-1
	dims.set('nbu', nu, ii);
end

dims.print_C_struct();





% data

% mass sprint system
Ts = 0.5; % sampling time

Ac = [zeros(nx/2), eye(nx/2); diag(-2*ones(nx/2,1))+diag(ones(nx/2-1,1),-1)+diag(ones(nx/2-1,1),1), zeros(nx/2) ];
Bc = [zeros(nx/2,nu); eye(nu); zeros(nx/2-nu, nu)];

M = expm([Ts*Ac, Ts*Bc; zeros(nu, 2*nx/2+nu)]);

% dynamica system
A = M(1:nx,1:nx);
B = M(1:nx,nx+1:end);

% cost function
Q = eye(nx);
R = 2*eye(nu);

Jx0 = eye(nx);
x0 = zeros(nx, 1);
x0(1) = 3.5;
x0(2) = 3.5;

% bounds
Ju = eye(nu);
lb_u = -0.5*ones(nu,1);
ub_u =  0.5*ones(nu,1);



% qp
tic
qp = hpipm_ocp_qp(dims);
tmp_time = toc;
fprintf('create qp time %e\n', tmp_time);

tic
% A
if 1
	% faster
	tmp = {};
	for ii=1:N
		tmp{ii} = A;
	end
	qp.set('A', tmp);
else
	% slower
	for ii=0:N-1
		qp.set('A', A, ii);
	end
end
tmp_time = toc;
fprintf('set A time %e\n', tmp_time);
% B
tmp = {};
for ii=1:N
	tmp{ii} = B;
end
qp.set('B', tmp);
% Q
tmp = {};
for ii=1:N+1
	tmp{ii} = Q;
end
qp.set('Q', tmp);
% R
tmp = {};
for ii=1:N
	tmp{ii} = R;
end
qp.set('R', tmp);
% Jx
qp.set('Jx', Jx0, 0);
% lx
qp.set('lx', x0, 0);
% ux
qp.set('ux', x0, 0);
% Ju
tmp = {};
for ii=1:N
	tmp{ii} = Ju;
end
qp.set('Ju', tmp);
% lu
tmp = {};
for ii=1:N
	tmp{ii} = lb_u;
end
qp.set('lu', tmp);
% uu
tmp = {};
for ii=1:N
	tmp{ii} = ub_u;
end
qp.set('uu', tmp);

%qp.print_C_struct()



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

arg.set_mu0(1e4);
arg.set_iter_max(30);
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
%    qp_sol.print_C_struct()
else
    fprintf('-> Solver failed!')
end


% extract and print sol
u = qp_sol.get_u();
x = qp_sol.get_x();

figure()
uu = zeros(nu, N);
for ii=1:N-1
	uu(:,ii) = u{ii};
end
plot(uu')
title('u')
axis([0 N-1 -1 1])

figure()
xx = zeros(nx, N+1);
for ii=1:N
	xx(:,ii) = x{ii};
end
plot(xx')
title('x')
axis([0 N -5 5])


% print solver statistics
fprintf('\nsolver statistics:\n\n');
fprintf('ipm return = %d\n\n', return_flag);
res_stat = solver.get_res_stat();
fprintf('ipm max res stat = %e\n\n', res_stat);
res_eq = solver.get_res_eq();
fprintf('ipm max res eq   = %e\n\n', res_eq);
res_ineq = solver.get_res_ineq();
fprintf('ipm max res ineq = %e\n\n', res_ineq);
res_comp = solver.get_res_comp();
fprintf('ipm max res comp = %e\n\n', res_comp);
iters = solver.get_iter();
fprintf('ipm iter = %d\n\n', iters);
stat = solver.get_stat();
fprintf('stat =\n');
fprintf('\talpha_aff\tmu_aff\t\tsigma\t\talpha\t\tmu\n');
for ii=1:iters
	fprintf('\t%e\t%e\t%e\t%e\t%e\n', stat(ii,1), stat(ii,2), stat(ii,3), stat(ii,4), stat(ii,5));
end
fprintf('\n');




return

