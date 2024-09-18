%
% Copyright 2019 Gianluca Frison, Dimitris Kouzoupis, Robin Verschueren,
% Andrea Zanelli, Niels van Duijkeren, Jonathan Frey, Tommaso Sartor,
% Branimir Novoselnik, Rien Quirynen, Rezart Qelibari, Dang Doan,
% Jonas Koenemann, Yutao Chen, Tobias Schöls, Jonas Schlagenhauf, Moritz Diehl
%
% This file is part of acados.
%
% The 2-Clause BSD License
%
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
%
% 1. Redistributions of source code must retain the above copyright notice,
% this list of conditions and the following disclaimer.
%
% 2. Redistributions in binary form must reproduce the above copyright notice,
% this list of conditions and the following disclaimer in the documentation
% and/or other materials provided with the distribution.
%
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
% POSSIBILITY OF SUCH DAMAGE.;
%

%% test of native matlab interface
clear all



%% arguments
compile_interface = 'true';
codgen_model = 'true';
param_scheme = 'multiple_shooting_unif_grid';
N = 20;

nlp_solver = 'sqp';
%nlp_solver = 'sqp_rti';
qp_solver = 'partial_condensing_hpipm';
%qp_solver = 'full_condensing_hpipm';
qp_solver_N_pcond = 5;
%sim_method = 'erk';
sim_method = 'irk';
sim_method_num_stages = 4;
sim_method_num_steps = 3;
cost_type = 'linear_ls';



%% create model entries
model = linear_mass_spring_model;
%model_funs = crane_model;

% dims
T = 10.0; % horizon length time
nx = model.nx;
nu = model.nu;
ny = nu+nx; % number of outputs in lagrange term
ny_e = nx; % number of outputs in mayer term
if 0
	nbx = nx/2;
	nbu = nu;
	ng = 0;
	nh = 0;
elseif 1
	nbx = 0;
	nbu = 0;
	ng = nu+nx/2;
	ng_e = nx;
	nh = 0;
else
	nbx = 0;
	nbu = 0;
	ng = 0;
	nh = nu+nx;
	nh_e = nx;
end
% cost
Vu = zeros(ny, nu); for ii=1:nu Vu(ii,ii)=1.0; end % input-to-output matrix in lagrange term
Vx = zeros(ny, nx); for ii=1:nx Vx(nu+ii,ii)=1.0; end % state-to-output matrix in lagrange term
Vx_e = zeros(ny_e, nx); for ii=1:nx Vx_e(ii,ii)=1.0; end % state-to-output matrix in mayer term
W = eye(ny); for ii=1:nu W(ii,ii)=2.0; end % weight matrix in lagrange term
W_e = eye(ny_e); % weight matrix in mayer term
yr = zeros(ny, 1); % output reference in lagrange term
yr_e = zeros(ny_e, 1); % output reference in mayer term
% constraints
x0 = zeros(nx, 1); x0(1)=2.5; x0(2)=2.5;
if (ng>0)
	D = zeros(ng, nu); for ii=1:nu D(ii,ii)=1.0; end
	C = zeros(ng, nx); for ii=1:ng-nu C(nu+ii,ii)=1.0; end
	lg = zeros(ng, 1); for ii=1:nu lg(ii)=-0.5; end; for ii=1:ng-nu lg(nu+ii)=-4.0; end
	ug = zeros(ng, 1); for ii=1:nu ug(ii)= 0.5; end; for ii=1:ng-nu ug(nu+ii)= 4.0; end
	C_e = zeros(ng_e, nx); for ii=1:ng_e C_e(ii,ii)=1.0; end
	lg_e = zeros(ng_e, 1); for ii=1:ng_e lg_e(ii)=-4.0; end
	ug_e = zeros(ng_e, 1); for ii=1:ng_e ug_e(ii)= 4.0; end
elseif (nh>0)
	lh = zeros(nh, 1); for ii=1:nu lh(ii)=-0.5; end; for ii=1:nx lh(nu+ii)=-4.0; end
	uh = zeros(nh, 1); for ii=1:nu uh(ii)= 0.5; end; for ii=1:nx uh(nu+ii)= 4.0; end
	lh_e = zeros(nh_e, 1); for ii=1:nh_e lh_e(ii)=-4.0; end
	uh_e = zeros(nh_e, 1); for ii=1:nh_e uh_e(ii)= 4.0; end
else
	Jbx = zeros(nbx, nx); for ii=1:nbx Jbx(ii,ii)=1.0; end
	lbx = -4*ones(nx, 1);
	ubx =  4*ones(nx, 1);
	Jbu = zeros(nbu, nu); for ii=1:nbu Jbu(ii,ii)=1.0; end
	lbu = -0.5*ones(nu, 1);
	ubu =  0.5*ones(nu, 1);
end




%% acados ocp model
ocp_model = acados_ocp_model();
% dims
ocp_model.set('T', T);
ocp_model.set('nx', nx);
ocp_model.set('nu', nu);
ocp_model.set('ny', ny);
ocp_model.set('ny_e', ny_e);
if (ng>0)
	ocp_model.set('ng', ng);
	ocp_model.set('ng_e', ng_e);
elseif (nh>0)
	ocp_model.set('nh', nh);
	ocp_model.set('nh_e', nh_e);
else
	ocp_model.set('nbx', nbx);
	ocp_model.set('nbu', nbu);
end
% symbolics
ocp_model.set('sym_x', model.sym_x);
if isfield(model, 'sym_u')
	ocp_model.set('sym_u', model.sym_u);
end
if isfield(model, 'sym_xdot')
	ocp_model.set('sym_xdot', model.sym_xdot);
end
% cost
ocp_model.set('cost_type', cost_type);
ocp_model.set('cost_e_type', cost_type);
ocp_model.set('Vu', Vu);
ocp_model.set('Vx', Vx);
ocp_model.set('Vx_e', Vx_e);
ocp_model.set('W', W);
ocp_model.set('W_e', W_e);
ocp_model.set('yr', yr);
ocp_model.set('yr_e', yr_e);
% dynamics
if (strcmp(sim_method, 'erk'))
	ocp_model.set('dyn_type', 'explicit');
	ocp_model.set('expr_f', model.expr_f_expl);
else % irk
	ocp_model.set('dyn_type', 'implicit');
	ocp_model.set('expr_f', model.expr_f_impl);
end
% constraints
ocp_model.set('x0', x0);
if (ng>0)
	ocp_model.set('C', C);
	ocp_model.set('D', D);
	ocp_model.set('lg', lg);
	ocp_model.set('ug', ug);
	ocp_model.set('C_e', C_e);
	ocp_model.set('lg_e', lg_e);
	ocp_model.set('ug_e', ug_e);
elseif (nh>0)
	ocp_model.set('expr_h', model.expr_h);
	ocp_model.set('lh', lh);
	ocp_model.set('uh', uh);
	ocp_model.set('expr_h_e', model.expr_h_e);
	ocp_model.set('lh_e', lh_e);
	ocp_model.set('uh_e', uh_e);
else
	ocp_model.set('Jbx', Jbx);
	ocp_model.set('lbx', lbx);
	ocp_model.set('ubx', ubx);
	ocp_model.set('Jbu', Jbu);
	ocp_model.set('lbu', lbu);
	ocp_model.set('ubu', ubu);
end

ocp_model.model_struct



%% acados ocp opts
ocp_opts = acados_ocp_opts();
ocp_opts.set('compile_interface', compile_interface);
ocp_opts.set('codgen_model', codgen_model);
ocp_opts.set('param_scheme', param_scheme);
ocp_opts.set('param_scheme_N', N);
ocp_opts.set('nlp_solver', nlp_solver);
ocp_opts.set('qp_solver', qp_solver);
if (strcmp(qp_solver, 'partial_condensing_hpipm'))
	ocp_opts.set('qp_solver_N_pcond', qp_solver_N_pcond);
end
ocp_opts.set('sim_method', sim_method);
ocp_opts.set('sim_method_num_stages', sim_method_num_stages);
ocp_opts.set('sim_method_num_steps', sim_method_num_steps);

ocp_opts.opts_struct



%% acados ocp
% create ocp
ocp = acados_ocp(ocp_model, ocp_opts);
ocp
ocp.C_ocp
ocp.C_ocp_ext_fun



% set trajectory initialization
x_traj_init = zeros(nx, N+1);
u_traj_init = zeros(nu, N);
ocp.set('x_init', x_traj_init);
ocp.set('u_init', u_traj_init);


% solve
tic;
ocp.solve();
time_solve = toc


x0(1) = 1.5;
ocp.set('x0', x0);


% if not set, the trajectory is initialized with the previous solution


tic;
ocp.solve();
time_solve = toc



% get solution
u = ocp.get('u');
x = ocp.get('x');



% plot result
figure()
subplot(2, 1, 1)
plot(0:N, x);
title('closed loop simulation')
ylabel('x')
subplot(2, 1, 2)
plot(1:N, u);
ylabel('u')
xlabel('sample')



fprintf('\nsuccess!\n\n');


return;
