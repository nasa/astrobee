%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                                                 %
% This file is part of HPIPM.                                                                     %
%                                                                                                 %
% HPIPM -- High-Performance Interior Point Method.                                                %
% Copyright (C) 2019 by Gianluca Frison.                                                          %
% Developed at IMTEK (University of Freiburg) under the supervision of Moritz Diehl.              %
% All rights reserved.                                                                            %
%                                                                                                 %
% The 2-Clause BSD License                                                                        %
%                                                                                                 %
% Redistribution and use in source and binary forms, with or without                              %
% modification, are permitted provided that the following conditions are met:                     %
%                                                                                                 %
% 1. Redistributions of source code must retain the above copyright notice, this                  %
%    list of conditions and the following disclaimer.                                             %
% 2. Redistributions in binary form must reproduce the above copyright notice,                    %
%    this list of conditions and the following disclaimer in the documentation                    %
%    and/or other materials provided with the distribution.                                       %
%                                                                                                 %
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND                 %
% ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED                   %
% WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE                          %
% DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR                 %
% ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES                  %
% (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;                    %
% LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND                     %
% ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT                      %
% (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS                   %
% SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                                    %
%                                                                                                 %
% Author: Gianluca Frison, gianluca.frison (at) imtek.uni-freiburg.de                             %
%                                                                                                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all
close all
clc



% check that env.sh has been run
env_run = getenv('ENV_RUN');
if (~strcmp(env_run, 'true'))
	disp('ERROR: env.sh has not been sourced! Before executing this example, run:');
	disp('source env.sh');
	return;
end

travis_run = getenv('TRAVIS_RUN');
%travis_run = 'true';



if (~strcmp(travis_run, 'true'))
	fprintf('\nHPIPM matlab interface: mass spring example\n');
end



% define flags
codegen_data = 1; % export qp data in the file ocp_qp_data.c for use from C examples



%%% data %%%
nx = 8;				% number of states
nu = 3;				% number of inputs (controls)
N = 30;				% horizon length

% mass sprint system
Ts = 0.5; % sampling time

Ac = [zeros(nx/2), eye(nx/2); diag(-2*ones(nx/2,1))+diag(ones(nx/2-1,1),-1)+diag(ones(nx/2-1,1),1), zeros(nx/2) ];
Bc = [zeros(nx/2,nu); eye(nu); zeros(nx/2-nu, nu)];

M = expm([Ts*Ac, Ts*Bc; zeros(nu, 2*nx/2+nu)]);

% dynamical system
A = M(1:nx,1:nx);
B = M(1:nx,nx+1:end);

% cost function
Q = eye(nx);
R = 2*eye(nu);
S = zeros(nx, nu);
q = zeros(nx, 1);
r = zeros(nu, 1);

% initial state
Jx0 = eye(nx);
x0 = zeros(nx, 1);
x0(1) = 3.5;
x0(2) = 3.5;

% input bounds
Jbu = eye(nu);
lbu = -0.5*ones(nu,1);
ubu =  0.5*ones(nu,1);



%%% dim %%%
dim = hpipm_ocp_qp_dim(N);

%% Note:
% The setters follow the following convention:
% obj.set('field', value, stage_index);
% or to set values for multiple consecutive stages:
% obj.set('field', value, first_stage_index, last_stage);
nbx = nx;
nbu = nu;
%ns = 0;
%ng = 0;
dim.set('nx', nx, 0, N);
dim.set('nu', nu, 0, N-1); % controls
dim.set('nbx', nbx, 0); % state bounds
dim.set('nbu', nbu, 0, N-1); % control bounds
%dim.set('ng', ng, 0, N); % general linear constraints
%dim.set('ns', ns, 0, N); % slacks

% print to shell
%dim.print_C_struct();
% codegen
if codegen_data
	dim.codegen('ocp_qp_data.c', 'w');
end



%%% qp %%%
qp = hpipm_ocp_qp(dim);

% dynamics
qp.set('A', A, 0, N-1);
qp.set('B', B, 0, N-1);

% cost
qp.set('Q', Q, 0, N);
qp.set('S', S, 0, N-1);
qp.set('R', R, 0, N-1);
qp.set('q', q, 0, N);
qp.set('r', r, 0, N-1);

%slack_L2 = 0;
%slack_L1 = 0;
%Z = slack_L2 * eye(ns);
%z = slack_L1 * ones(ns, 1);
%qp.set('Zl', Z, 0, N-1);
%qp.set('Zu', Z, 0, N-1);
%qp.set('zl', z, 0, N-1);
%qp.set('zu', z, 0, N-1);

% constraints
qp.set('Jbx', Jx0, 0);
qp.set('lbx', x0, 0);
qp.set('ubx', x0, 0);
qp.set('Jbu', Jbu, 0, N-1);
qp.set('lbu', lbu, 0, N-1);
qp.set('ubu', ubu, 0, N-1);

%Jsx = eye(nbx, ns);
%qp.set('Jsx', Jsx, 0, N);
%Jsu = zeros(nbu, ns);
%qp.set('Jsu', Jsu, 0, N-1);

%slack_bounds = zeros(ns,1);
%qp.set('lus', slack_bounds, 0, N);
%qp.set('lls', slack_bounds, 0, N);

% print to shell
%qp.print_C_struct();
% codegen
if codegen_data
	qp.codegen('ocp_qp_data.c', 'a');
end



%%% sol %%%
sol = hpipm_ocp_qp_sol(dim);



%%% solver arg %%%
%mode = 'speed_abs';
mode = 'speed';
%mode = 'balance';
%mode = 'robust';
% create and set default arg based on mode
arg = hpipm_ocp_qp_solver_arg(dim, mode);

% overwrite default argument values
arg.set('mu0', 1e4);
arg.set('iter_max', 20);
arg.set('tol_stat', 1e-4);
arg.set('tol_eq', 1e-5);
arg.set('tol_ineq', 1e-5);
arg.set('tol_comp', 1e-5);
arg.set('reg_prim', 1e-12);
%arg.set('warm_start', 1);

% codegen
if codegen_data
	arg.codegen('ocp_qp_data.c', 'a');
end



%%% solver %%%
solver = hpipm_ocp_qp_solver(dim, arg);

% arg which are allowed to be changed
solver.set('iter_max', 30);
solver.set('tol_stat', 1e-8);
solver.set('tol_eq', 1e-8);
solver.set('tol_ineq', 1e-8);
solver.set('tol_comp', 1e-8);

% set solution guess
%sol.set('x', zeros(nx,1), 0, N);
%sol.set('u', zeros(nu,1), 0, N-1);
%sol.set('sl', zeros((N+1)*ns,1), 0, N-1);
%sol.set('su', zeros((N+1)*ns,1), 0, N-1);

% solve qp
nrep = 100;
tic
for rep=1:nrep
	solver.solve(qp, sol);
end
solve_time = toc;

% get solution statistics
status = solver.get('status');
time_ext = solver.get('time_ext');
iter = solver.get('iter');
res_stat = solver.get('max_res_stat');
res_eq = solver.get('max_res_eq');
res_ineq = solver.get('max_res_ineq');
res_comp = solver.get('max_res_comp');
stat = solver.get('stat');
if (~strcmp(travis_run, 'true'))
	status
	iter
	res_stat
	res_eq
	res_ineq
	res_comp
	fprintf('\nprint solver statistics\n');
	fprintf('average solve time over %d runs: %e [s]\n', nrep, solve_time/nrep);
	fprintf('solve time of last run (measured in mex interface): %e [s]\n', time_ext);
	fprintf('iter\talpha_aff\tmu_aff\t\tsigma\t\talpha_prim\talpha_dual\tmu\t\tres_stat\tres_eq\t\tres_ineq\tres_comp\n');
	for ii=1:iter+1
		fprintf('%d\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\n', stat(ii,1), stat(ii,2), stat(ii,3), stat(ii,4), stat(ii,5), stat(ii,6), stat(ii,7), stat(ii,8), stat(ii,9), stat(ii,10), stat(ii,11));
	end
end



% get solution
% x
x = sol.get('x', 0, N);
x = reshape(x, nx, N+1);
% u
u = sol.get('u', 0, N-1);
u = reshape(u, nu, N);
% slack values
su = sol.get('su', 0, N);
sl = sol.get('sl', 0, N);

% print to shell
sol.print_C_struct();



% plot solution
if (~strcmp(travis_run, 'true'))
	figure()
	subplot(2, 1, 1)
	plot(0:N, x);
	grid on
	title('trajectory')
	ylabel('x')
	subplot(2, 1, 2)
	stairs(1:N, u');
	grid on
	ylabel('u')
	xlabel('sample')
end



if is_octave()
	% directly call destructor for octave 4.2.2 (ubuntu 18.04) + others ???
	if strcmp(version(), '4.2.2')
		delete(dim);
		delete(qp);
		delete(sol);
		delete(arg);
		delete(solver);
	end
end



if status==0
	fprintf('\nsuccess!\n\n');
else
	fprintf('\nSolution failed, solver returned status %d\n', status);
	exit(1);
end



if (~strcmp(travis_run, 'true'))

	waitforbuttonpress;

end


return


