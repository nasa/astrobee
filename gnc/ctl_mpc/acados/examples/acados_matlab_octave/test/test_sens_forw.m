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
clear VARIABLES

addpath('../linear_mass_spring_model/');

for integrator = {'irk_gnsf', 'irk', 'erk'}
    method = integrator{1}; %'irk'; 'irk_gnsf'; 'erk';

    %% arguments
    compile_interface = 'auto';
    codgen_model = 'true';
    sens_forw = 'true';
    jac_reuse = 'true';
    num_stages = 3;
    num_steps = 4;
    newton_iter = 3;
    gnsf_detect_struct = 'true';

    Ts = 0.1;
    FD_epsilon = 1e-6;
    
    %% model
    model = linear_mass_spring_model;
    
    model_name = ['lin_mass_' method];
    nx = model.nx;
    nu = model.nu;
    % x0 = [1e-1; 1e0; 2e-1; 2e0]; % pendulum
    % u = 0;

    % linear_mass_spring_
    x0 = ones(nx,1);
    u = ones(nu,1);

    %% acados sim model
    sim_model = acados_sim_model();
    sim_model.set('T', Ts);
    sim_model.set('name', model_name);

    sim_model.set('sym_x', model.sym_x);
    if isfield(model, 'sym_u')
        sim_model.set('sym_u', model.sym_u);
    end
    if isfield(model, 'sym_p')
        sim_model.set('sym_p', model.sym_p);
    end

    if (strcmp(method, 'erk'))
        sim_model.set('dyn_type', 'explicit');
        sim_model.set('dyn_expr_f', model.expr_f_expl);
    else % irk irk_gnsf
        sim_model.set('dyn_type', 'implicit');
        sim_model.set('dyn_expr_f', model.expr_f_impl);
        sim_model.set('sym_xdot', model.sym_xdot);
    %	if isfield(model, 'sym_z')
    %		sim_model.set('sym_z', model.sym_z);
    %	end
    end


    %% acados sim opts
    sim_opts = acados_sim_opts();
    sim_opts.set('compile_interface', compile_interface);
    sim_opts.set('codgen_model', codgen_model);
    sim_opts.set('num_stages', num_stages);
    sim_opts.set('num_steps', num_steps);
    sim_opts.set('newton_iter', newton_iter);
    sim_opts.set('method', method);
    sim_opts.set('sens_forw', sens_forw);
    sim_opts.set('jac_reuse', jac_reuse);
    if (strcmp(method, 'irk_gnsf'))
        sim_opts.set('gnsf_detect_struct', gnsf_detect_struct);
    end

    %% acados sim
    % create sim
    sim = acados_sim(sim_model, sim_opts);

    % Note: this does not work with gnsf, because it needs to be available
    % in the precomputation phase
    % 	sim.set('T', Ts);

    % set initial state
    sim.set('x', x0);
    sim.set('u', u);

    % initialize implicit integrator
    if (strcmp(method, 'irk'))
        sim.set('xdot', zeros(nx,1));
    elseif (strcmp(method, 'irk_gnsf'))
        n_out = sim.model_struct.dim_gnsf_nout;
        sim.set('phi_guess', zeros(n_out,1));
    end

    % solve
    sim.solve();

    xn = sim.get('xn');
    S_forw_ind = sim.get('S_forw');

    %% compute forward sensitivities using finite differences
    sim_opts.set('sens_forw', 'false');
    sim_opts.set('sens_adj', 'false');
    S_forw_fd = zeros(nx, nx+nu);

    %% asymmetric finite differences
    for ii=1:nx
        dx = zeros(nx, 1);
        dx(ii) = 1.0;

        sim.set('x', x0+FD_epsilon*dx);
        sim.set('u', u);
        sim.solve();

        xn_tmp = sim.get('xn');
        S_forw_fd(:,ii) = (xn_tmp - xn) / FD_epsilon;
    end

    for ii=1:nu
        du = zeros(nu, 1);
        du(ii) = 1.0;

        sim.set('x', x0);
        sim.set('u', u+FD_epsilon*du);
        sim.solve();

        xn_tmp = sim.get('xn');
        S_forw_fd(:,nx+ii) = (xn_tmp - xn) / FD_epsilon;
    end

    %% compute & check error
    error_abs = max(max(abs(S_forw_fd - S_forw_ind)));
    disp(' ')
    disp(['integrator:  ' method]);
    disp(['error forward sensitivities (wrt forward sens):   ' num2str(error_abs)])
    disp(' ')
    if error_abs > 1e-6
        disp(['forward sensitivities error too large'])
        error(strcat('test_sens_adj FAIL: forward sensitivities error too large: \n',...
            'for integrator:\t', method));
    end
end

fprintf('\nTEST_FORWARD_SENSITIVITIES: success!\n\n');
