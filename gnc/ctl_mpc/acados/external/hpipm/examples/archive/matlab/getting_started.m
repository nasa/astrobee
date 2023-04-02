% pyversion /usr/bin/python3
% py.sys.path

clearvars
close all
clc

addpath('../../interfaces/matlab/hpipm_matlab')

import hpipm_matlab.*



% dims
N = 5;

tic
dims = hpipm_ocp_qp_dim(N);
tmp_time = toc;
fprintf('create dim time %e\n', tmp_time);

tic
dims.set('nx', [2, 2, 2, 2, 2, 2]);
tmp_time = toc;
fprintf('set nx time %e\n', tmp_time);
dims.set('nu', [1, 1, 1, 1, 1]);
dims.set('nbx', 2, 0);
dims.set('nbx', 2, 5);

dims.print_C_struct();



% data
A = [1, 1; 0, 1];
B = [0; 1];
%b = [0; 0]

Q = [1, 1; 0, 1];
S = [0, 0];
R = [1];
q = [1; 1];
%r = [0];

Jx = [1, 0; 0, 1];
x0 = [1; 1];
Zu = [1e5, 0; 0, 1e5];
zl = [1e5; 1e5];
zu = [1e5; 1e5];



% qp
tic
qp = hpipm_ocp_qp(dims);
tmp_time = toc;
fprintf('create qp time %e\n', tmp_time);

tic
qp.set('A', {A, A, A, A, A});
tmp_time = toc;
fprintf('create set A time %e\n', tmp_time);
qp.set('B', {B, B, B, B, B});
%qp.set_b({b, b, b, b, b});

qp.set('Q', {Q, Q, Q, Q, Q, Q});
qp.set('S', {S, S, S, S, S});
qp.set('R', {R, R, R, R, R});
qp.set('q', {q, q, q, q, q, q});
qp.set('q', q, 0);
%qp.set_r({r, r, r, r, r});
qp.set('Jx', Jx, 0);
qp.set('lx', x0, 0);
qp.set('ux', x0, 0);
qp.set('Jx', Jx, 5);

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
    qp_sol.print_C_struct()
else
    fprintf('-> Solver failed!')
end


% extract and print sol
fprintf('u =\n');
u = qp_sol.get_u();
for i=1:N+1
	u{i}
end

fprintf('x =\n');
for i=0:N
	x_tmp = qp_sol.get_x(i);
	x_tmp
end


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

