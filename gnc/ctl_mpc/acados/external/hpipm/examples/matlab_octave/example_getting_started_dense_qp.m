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
	fprintf('\nHPIPM matlab interface: getting started dense QP example\n');
end



% define flags
%codegen_data = 1; % export qp data in the file dense_qp_data.c for use from C examples XXX not implemented yet !!!!!
constr_type = 0; % 0 box, 1 general



%%% data %%%
nv = 2;
ne = 1;

A = [2, -1];

H = 100*[1, 0; 0, 1];

Jb = [1, 0; 0, 1];
lb = [1; 1];
ub = [3; 3];



%%% dim %%%
dim = hpipm_dense_qp_dim();

%% Note:
% The setters follow the following convention:
% obj.set('field', value);

dim.set('nv', nv);
dim.set('ne', ne);
if(constr_type==0)
	dim.set('nb', nv);
else
	dim.set('ng', nv);
end

% print to shell
dim.print_C_struct();
% codegen
%if codegen_data
%	dim.codegen('dense_qp_data.c', 'w');
%end


%%% qp %%%
qp = hpipm_dense_qp(dim);

qp.set('H', H);
qp.set('A', A);
if(constr_type==0)
	qp.set('Jb', Jb);
	qp.set('lb', lb);
	qp.set('ub', ub);
else
	qp.set('C', Jb);
	qp.set('lg', lb);
	qp.set('ug', ub);
end

% print to shell
qp.print_C_struct();
% codegen
%if codegen_data
%	qp.codegen('dense_qp_data.c', 'a');
%end




%%% sol %%%
sol = hpipm_dense_qp_sol(dim);



%%% solver arg %%%
%mode = 'speed_abs';
mode = 'speed';
%mode = 'balance';
%mode = 'robust';
% create and set default arg based on mode
arg = hpipm_dense_qp_solver_arg(dim, mode);

% overwrite default argument values
arg.set('mu0', 1e4);
arg.set('iter_max', 20);
arg.set('tol_stat', 1e-4);
arg.set('tol_eq', 1e-5);
arg.set('tol_ineq', 1e-5);
arg.set('tol_comp', 1e-5);
arg.set('reg_prim', 1e-12);

% codegen
%if codegen_data
%	arg.codegen('dense_qp_data.c', 'a');
%end



%%% solver %%%
solver = hpipm_dense_qp_solver(dim, arg);

% arg which are allowed to be changed
solver.set('iter_max', 30);
arg.set('tol_stat', 1e-8);
arg.set('tol_eq', 1e-8);
arg.set('tol_ineq', 1e-8);
arg.set('tol_comp', 1e-8);

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
	fprintf('\n');
end



% get / print solution
% v
v = sol.get('v');

if (~strcmp(travis_run, 'true'))
	v
end

% print to shell
sol.print_C_struct();



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

