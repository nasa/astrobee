close all
clear all
clc

addpath('./plotregion')
import casadi.*

% warning('off','all')

% B_STRATEGY      = 'MONOTONE';   % barrier strategy, possible values: {'MONOTONE', 'MEHROTRA'}
B_STRATEGY      = 'MEHROTRA';   % barrier strategy, possible values: {'MONOTONE', 'MEHROTRA'}
MAX_ITER        = 200;         % maximum solver iterations    
TAU0            = 1e1;          % initial barrier parameter value
MAX_LS_IT       = 100;          % maximum number of (positivity) line-search steps
PRINT_LEVEL     = 2;            % print level possible values {1, 2}
DTB             = 0.1;          % minimum distance to boundaries (used for initialization)
GAMMA           = 0.95;         % safeguard factor for solution update 
REG             = 1e-8;         % regularization of primal Hessian
TOL             = 1e-1;         % tolerance for monotone strategy
TERM_TOL        = 1e-8;         % termination tolerance
KAPPA           = 0.3;          % barrier parameter decrease factor 
REMOVE_AFF_C    = 0;            % remove affine constraints from problem
INIT_STRATEGY   = 1;            % initialization strategy (TODO(Andrea): fix)
MAX_TAU         = 1e10;         % maximum barrier parameter value
MIN_TAU         = 0.1*TERM_TOL; % maximum barrier parameter value
RES_NORM        = Inf;          % norm type used to compute residuals
ITER_REF        = 0;            % number of iterative refinement iterations
MIN_FTB         = 0.95;         % minimum fraction-to-the-boundaries
COND_MPC        = 0;            % conditional Mehrotra predictor-corrector
LIFT_AFF        = 0;            % lifted formulation for polytopic constraints (TODO(Andrea): buggy)
S_MAX           = 100;
REG_J           = 1e-8;

load benchmark
benchmark = [ 24 ];
% benchmark = [ 1:length(H) ];
solved = zeros(length(H), 1);

nQP = size(H,1);
C_full = A;
n_ieq = nc;
n_eq = ne;
num_pass = 0;
grad = g;
clear g A nc ne

for kk = 1: length(benchmark)

    if LIFT_AFF
        num_prob = benchmark(kk);
        
        nb = 2*nv{num_prob, 1};
        nc = 2*n_ieq{num_prob, 1};
        nx = nv{num_prob, 1};
        ne = n_eq{num_prob, 1};
        
        A = [];
        b = [];
        lbC = [];
        ubC = [];
        C_ieq = [];
        for i = 1: size(C_full{num_prob},1)
            if lbA{num_prob}(i) == ubA{num_prob}(i)
                A = [A; C_full{num_prob}(i,:)];
                b = [b; lbA{num_prob}(i)];
            else
                C_ieq = [C_ieq; C_full{num_prob}(i,:)];
                lbC = [lbC; lbA{num_prob}(i)];
                ubC = [ubC; ubA{num_prob}(i)];
            end
        end
        
        C  = [C_ieq; -C_ieq];
        
        if size(A) ~= [ne,nx]
            disp('Dimension of Ax are wrong');
        end
        
        if size(C,1) ~= [nc,nx]
            disp('Dimension of Cx are wrong');
        end
        
        e =   [ ubx{num_prob, 1}; -lbx{num_prob, 1} ];
        d =   [ ubC; -lbC ];
        
        nx_ = nx;
        nx = nx + nc;
        
        Q = blkdiag(H{num_prob, 1}, zeros(nc)) + REG*eye(nx);
        q = [grad{num_prob,1}; zeros(nc, 1)];
        
        nc_ = nc;
        nc  = nc + 2*nx_;
        
        ne = ne + nc_;
        
        x   = MX.sym('x',   nx, 1);
        l   = MX.sym('l',   ne, 1);
        mu  = MX.sym('mu',  nc, 1);
        s   = MX.sym('s',   nc, 1);
        tau = MX.sym('tau', 1,  1);
        
        f = 1/2*x.'*Q*x + q.'*x;
        
        if isempty(A)
            g = C*x(1:nx_) - x(nx_ + 1:end);
        else
            g = [   A*x - b; ...
                C*x(1:nx_) - x(nx_ + 1:end)];
        end
        
        h = [[ eye(nx_); -eye(nx_)]*x(1:nx_); [eye(nc_)]*x(nx_+1:end)]  - [e; d];
        
        if ne > 0
            Lag = f + l.'*g + mu.'*h;
        else
            Lag = f + mu.'*h;
        end
        rf_exp = [  jacobian(Lag,x).'; ...
            g; ...
            h + s; ...
            diag(s)*mu-ones(nc,1)*tau];
        
        rf = Function('rf', {x, l, mu, s, tau}, {rf_exp});
        j_rf = Function('j_rf', {x, l, mu, s}, {jacobian(rf_exp, [x; l; mu; s])});
        
        % Newton's method
        
        clear x l mu s tau
        max_iter = 100;
        
        p_alpha = 0;
        d_alpha = 0;
        step = 0;
        tau = TAU0;
        ls_acc = 0.0;
        
        iter.x  = zeros(nx,MAX_ITER);
        iter.l  = zeros(ne,MAX_ITER);
        iter.mu = sqrt(TAU0)*ones(nc,MAX_ITER);
        iter.s  = sqrt(TAU0)*ones(nc,MAX_ITER);
        
        iter.l(:,1) = 1;
        iter.x(1:nx_,1) = 0.5 * (lbx{num_prob,1} + ubx{num_prob,1});
        iter.x(nx_ + 1:end,1) = min(d - 1*ones(length(d),1), zeros(length(d),1));
        % iter.mu = 0.1*ones(nc,MAX_ITER);
        
        iter.l(:,1) = 0;
    else
        num_prob = benchmark(kk);
        if REMOVE_AFF_C == 1
            nc = 2*nv{num_prob,1};
        else
            nc = 2*(n_ieq{num_prob,1}+nv{num_prob,1});
        end
        nx = nv{num_prob,1};
        ne = n_eq{num_prob,1};
        
        A = [];
        b = [];
        lbC = [];
        ubC = [];
        C_ieq = [];
        for i = 1: size(C_full{num_prob},1)
            if lbA{num_prob}(i) == ubA{num_prob}(i)
                A = [A;C_full{num_prob}(i,:)];
                b = [b;lbA{num_prob}(i)];
            else
                C_ieq = [C_ieq;C_full{num_prob}(i,:)];
                lbC = [lbC;lbA{num_prob}(i)];
                ubC = [ubC;ubA{num_prob}(i)];
            end
        end
        
        if REMOVE_AFF_C == 1
            C  = [eye(nx);-eye(nx)];
        else
            C  = [C_ieq;-C_ieq;eye(nx);-eye(nx)];
        end
        if size(A) ~= [ne,nx]
            disp('Dimension of Ax are wrong');
        end
        
        if size(C,1) ~= [nc,nx]
            disp('Dimension of Cx are wrong');
        end
        
        shift = 0;
        
        if REMOVE_AFF_C == 1
            d = [ ubx{num_prob,1};-lbx{num_prob,1} ] + [ shift*ones(nx,1); -shift*ones(nx,1) ];
        else
            d = [ ubC;-lbC;ubx{num_prob,1};-lbx{num_prob,1} ] + [ shift*ones(n_ieq{num_prob,1},1); -shift*ones(n_ieq{num_prob,1},1);shift*ones(nx,1); -shift*ones(nx,1) ];
        end
        Q = H{num_prob,1} + REG*eye(nx);
        q = grad{num_prob,1};
        
        x   = MX.sym('x', nx, 1);
        l   = MX.sym('l', ne, 1);
        mu  = MX.sym('mu', nc, 1);
        s   = MX.sym('s', nc, 1);
        tau = MX.sym('tau', 1, 1);
        
        f = 1/2*x.'*Q*x + q.'*x;
        if isempty(A)
            g = [];
        else
            g = A*x - b;
        end
        h = (C*x - d);
        if ne > 0
            Lag = f + l.'*g + mu.'*h;
        else
            Lag = f + mu.'*h;
        end
        rf_exp = [  jacobian(Lag,x).'; ...
            g; ...
            h + s; ...
            diag(s)*mu-ones(nc,1)*tau];
        
        rf = Function('rf', {x, l, mu, s, tau}, {rf_exp});
        j_rf = Function('j_rf', {x, l, mu, s}, {jacobian(rf_exp, [x; l; mu; s])});
        
        % Newton's method
        
        clear x l mu s tau
        max_iter = 100;
        
        p_alpha = 0;
        d_alpha = 0;
        step = 0;
        tau = TAU0;
        ls_acc = 0.0;
        
        iter.x  = zeros(nx,MAX_ITER);
        iter.l  = zeros(ne,MAX_ITER);
        iter.mu = zeros(nc,MAX_ITER);
        iter.s  = zeros(nc,MAX_ITER);
        
        iter.l(:,1) = 10;
        %     iter.x(:,1) = 0.1*ones(length(iter.x(:,1)));
        iter.x(:,1) = 0.5 * (lbx{num_prob,1} + ubx{num_prob,1});
        iter.mu = 0.1*ones(nc,MAX_ITER);
        
        iter.l(:,1) = 0;
        
        if INIT_STRATEGY == 1
            if sum(-C*iter.x(:,1) + d > DTB) == nc
                iter.s(:,1)  = -C*iter.x(:,1) + d;
                iter.mu(:,1) = tau./iter.s(:,1);
            else
                iter.s(:,1)  = sqrt(tau)*ones(nc,1);
                iter.mu(:,1) = sqrt(tau)*ones(nc,1);
            end
        else
            iter.s(:,1)  = sqrt(tau)*ones(nc,1);
            iter.mu(:,1) = sqrt(tau)*ones(nc,1);
        end
    end

    for i = 1:MAX_ITER
        
        if rank(A) < size(A,1)
            fprintf('A is not full row rank for problem %i: skipping this problem.\n', benchmark(kk));
            break;
        end
        
        % compute step-szie
        x = iter.x(:,i);
        l = iter.l(:,i);
        mu = iter.mu(:,i);
        s = iter.s(:,i);
                       
        % compute search direction
        rhs = full(rf(x, l, mu, s, tau));
        
        s_c = max(S_MAX, norm(l, 1) + norm(mu, 1)/(nc + nx));
        s_d = max(S_MAX, norm(mu, 1)/nc);
        err_s = norm(rhs(1:nx), RES_NORM)/s_d;
        err_e = norm(rhs(nx+1:nx+ne), RES_NORM);
        err_i = norm(rhs(nx+ne+1:nx+ne+nc), RES_NORM);
        err_c = norm(rhs(nx+ne+nc+1:nx+ne+nc+nc), RES_NORM)/s_c;
        
        if PRINT_LEVEL == 2
            fprintf('it = %3.f   err_s = %5.e    err_e = %5.e    err_i = %5.e    err_c =    %5.e    tau = %5.e    p_alpha = %5.e    d_alpha = %5.e     ls_acc = %5.e\n', i, err_s,  err_e,  err_i,  err_c, tau, p_alpha, d_alpha, ls_acc);
        end
        
        err = norm(rhs, RES_NORM);
        
        if strcmp(B_STRATEGY, 'MONOTONE')
            if err_s < TOL && err_e < TOL && err_i < TOL && err_c < TOL
                tau = max(MIN_TAU, KAPPA*tau);
            end
        end
        
        if err_s < TERM_TOL && err_e < TERM_TOL && err_i < TERM_TOL && err_c < TERM_TOL && tau < TERM_TOL
            fprintf('\nsummary: num_QP = %3.f   it = %3.f   solved = 1   err_s = %5.e    err_e = %5.e    err_i = %5.e    err_c =    %5.e    tau = %5.e    p_alpha = %5.e    d_alpha = %5.e\n', num_prob, i, err_s,  err_e,  err_i,  err_c, tau, p_alpha, d_alpha);
            num_pass = num_pass + 1;
            solved(kk) = 1;
            break
        end
        
%         s(s < MIN_S) = MIN_S;
%         mu(mu < MIN_MU) = MIN_MU;
        
        J = full(j_rf(x, l, mu, s));
        J = J + REG_J*eye(size(J, 1));
%         cond(J)
        
        if strcmp(B_STRATEGY, 'MEHROTRA')
            % compute duality measure
            dm = s.'*mu/nc;
            
            % solve affine system
            aff_rhs = full(rf(x, l, mu, s, 0));
            aff_step = -J\aff_rhs;
            
            % extract mu and s step
            aff_mu_step = aff_step(nx+ne+1:nx+ne+nc);
            aff_s_step  = aff_step(nx+ne+nc+1:end);
            
            % do line-search for aff
            aff_alpha = 1;
            for ls_iter = 1:MAX_LS_IT
                t_mu  = mu  + aff_alpha*aff_mu_step;
                ftb = max(MIN_FTB, 1 - tau);
                if isempty(t_mu(t_mu < (1 - ftb)*mu)) 
                    break
                end
                if ls_iter == MAX_LS_IT
                    break;
                end
                aff_alpha = 0.5*aff_alpha;
            end
            
            aff_alpha = 1;
            for ls_iter = 1:MAX_LS_IT
                t_s  = s  + aff_alpha*aff_s_step;
                ftb = max(MIN_FTB, 1 - tau);
                if isempty(t_s(t_s < (1 - ftb)*s)) 
                    break
                end
                if ls_iter == MAX_LS_IT
                    break;
                end
                aff_alpha = 0.5*aff_alpha;
            end
            % compute affine duality measure
            aff_dm = t_s.'*t_mu/nc;
            
            % compute centering parameter
            sigma = (aff_dm/dm)^3;
            
            % update tau
            tau = sigma*dm;
            if tau > MAX_TAU
                tau = MAX_TAU;
            end
            
            if tau < MIN_TAU
                tau = MIN_TAU;
            end
            
            % update complementarity residuals (build aggregated rhs)
            rhs = full(rf(x, l, mu, s, tau));
            rhs((nx+ne+nc+1:end)) = rhs(nx+ne+nc+1:end) + diag(aff_s_step)*aff_mu_step;
        end

        step = linsolve(J, -rhs);
        
        for iter_ref = 1:ITER_REF
            % compute residuals
            lin_res = +rhs + J*step;
            
            % compute correction
            corr = linsolve(J, -lin_res);
            step = step + 1*corr;
            norm(J*step + rhs, Inf);
        end
        
        ls_acc = norm(J*step + rhs, Inf);
        
        % extract search direction componenents
        dx  = step(1:nx);
        dl  = step(nx+1:nx+ne);
        dmu = step(nx+ne+1:nx+ne+nc);
        ds  = step(nx+ne+nc+1:end);
        
        % do line-search
        p_alpha = 1;
        for ls_iter = 1:MAX_LS_IT
            t_s  = s  + p_alpha*ds;
            ftb = max(MIN_FTB, 1 - tau);
            if isempty(t_s(t_s < (1 - ftb)*s)) 
                break
            end
            if ls_iter == MAX_LS_IT
                fprintf(' num_QP = %5.f   it = %5.f   solved = 0   err_s = %5.e    err_e = %5.e    err_i = %5.e    err_c =    %5.e    tau = %5.e    alpha = %5.e\n', num_prob, i, err_s,  err_e,  err_i,  err_c, tau, p_alpha, d_alpha);
                break;
                i = MAX_ITER;
            end
            p_alpha = 0.5*p_alpha;
        end
        
        d_alpha = 1;
        for ls_iter = 1:MAX_LS_IT
            t_mu = mu + d_alpha*dmu;
            ftb = max(MIN_FTB, 1 - tau);
            if isempty(t_mu(t_mu < (1 - ftb)*mu))
                break
            end
            if ls_iter == MAX_LS_IT
                fprintf(' num_QP = %5.f   it = %5.f   solved = 0   err_s = %5.e    err_e = %5.e    err_i = %5.e    err_c =    %5.e    tau = %5.e    alpha = %5.e\n', num_prob, i, err_s,  err_e,  err_i,  err_c, tau, p_alpha, d_alpha);
                break;
                i = MAX_ITER;
            end
            d_alpha = 0.5*d_alpha;
        end
        
        if strcmp(B_STRATEGY, 'MEHROTRA') && COND_MPC
            if (mu + GAMMA*d_alpha*dmu).'*(s  + GAMMA*p_alpha*ds) > 2*dm
                rhs((nx+ne+nc+1:end)) = rhs(nx+ne+nc+1:end) - diag(aff_s_step)*aff_mu_step;
            end
            
            step = linsolve(J, -rhs);
            
            for iter_ref = 1:ITER_REF
                % compute residuals
                lin_res = +rhs + J*step;
                
                % compute correction
                corr = linsolve(J, -lin_res);
                step = step + 1*corr;
                norm(J*step + rhs, Inf);
            end
            
            ls_acc = norm(J*step + rhs, Inf);
            
            % extract search direction componenents
            dx  = step(1:nx);
            dl  = step(nx+1:nx+ne);
            dmu = step(nx+ne+1:nx+ne+nc);
            ds  = step(nx+ne+nc+1:end);
            
            % do line-search
            p_alpha = 1;
            for ls_iter = 1:MAX_LS_IT
                t_s  = s  + p_alpha*ds;
                ftb = max(MIN_FTB, 1 - tau);
                if isempty(t_s(t_s < (1 - ftb)*s))
                    break
                end
                if ls_iter == MAX_LS_IT
                    fprintf(' num_QP = %5.f   it = %5.f   solved = 0   err_s = %5.e    err_e = %5.e    err_i = %5.e    err_c =    %5.e    tau = %5.e    alpha = %5.e\n', num_prob, i, err_s,  err_e,  err_i,  err_c, tau, p_alpha, d_alpha);
                    break;
                    i = MAX_ITER;
                end
                p_alpha = 0.5*p_alpha;
            end
            
            d_alpha = 1;
            for ls_iter = 1:MAX_LS_IT
                t_mu = mu + d_alpha*dmu;
                ftb = max(MIN_FTB, 1 - tau);
                if isempty(t_mu(t_mu < (1 - ftb)*mu))
                    break
                end
                if ls_iter == MAX_LS_IT
                    fprintf(' num_QP = %5.f   it = %5.f   solved = 0   err_s = %5.e    err_e = %5.e    err_i = %5.e    err_c =    %5.e    tau = %5.e    p_alpha = %5.e    d_alpha = %5.e\n', num_prob, i, err_s,  err_e,  err_i,  err_c, tau, p_alpha, d_alpha);
                    break;
                    i = MAX_ITER;
                end
                d_alpha = 0.5*d_alpha;
            end
            
        end
        
        iter.x(:,i+1)  = x  + GAMMA*p_alpha*dx;
        iter.l(:,i+1)  = l  + GAMMA*p_alpha*dl;
        iter.mu(:,i+1) = mu + GAMMA*d_alpha*dmu;
        iter.s(:,i+1)  = s  + GAMMA*p_alpha*ds;
        
        if i == MAX_ITER
            format shortE
            fprintf(' num_QP = %5.f   it = %5.f   solved = 0   err_s = %5.e    err_e = %5.e    err_i = %5.e    err_c =    %5.e    tau = %5.e    p_alpha = %5.e    d_alpha = %5.e\n', num_prob, i, err_s,  err_e,  err_i,  err_c, tau, p_alpha, d_alpha);
        end
    end
        
end

fprintf('\n -> Number of QP = %d,  Number of solved QP  = %d, ratio = %5.e\n', length(benchmark), num_pass, num_pass/num_prob);

save('')