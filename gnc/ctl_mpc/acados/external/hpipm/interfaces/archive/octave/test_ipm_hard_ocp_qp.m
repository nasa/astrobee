% compile the C code
mex HPIPM_d_solve_ipm2_hard_ocp_qp.c /opt/hpipm/lib/libhpipm.a /opt/blasfeo/lib/libblasfeo.a -I/opt/hpipm/include -I/opt/blasfeo/include  % linear algebra in BLASFEO

% import cool graphic toolkit if in octave
if is_octave()
	graphics_toolkit('gnuplot')
end



% test problem

nx = 8;			% number of states
nu = 3;				% number of inputs (controls)
N = 30;				% horizon length

nb  = nu+nx/2;		% number of two-sided box constraints
ng  = 0;            % number of two-sided general constraints
ngN = nx;           % number of two-sided general constraints on last stage

time_invariant = 0; % time_invariant (0) vs time_variant (1) problems

nbu = min(nb, nu);

Ts = 0.5; % sampling time

Ac = [zeros(nx/2), eye(nx/2); diag(-2*ones(nx/2,1))+diag(ones(nx/2-1,1),-1)+diag(ones(nx/2-1,1),1), zeros(nx/2) ];
Bc = [zeros(nx/2,nu); eye(nu); zeros(nx/2-nu, nu)];

M = expm([Ts*Ac, Ts*Bc; zeros(nu, 2*nx/2+nu)]);

% dynamica system
A = M(1:nx,1:nx);
B = M(1:nx,nx+1:end);
b = 0.0*ones(nx,1);
x0 = zeros(nx, 1);
x0(1) = 3.5;
x0(2) = 3.5;
AA = repmat(A, 1, N);
BB = repmat(B, 1, N);
bb = repmat(b, 1, N);

% cost function
Q = eye(nx);
Qf = Q;
R = 2*eye(nu);
S = zeros(nx, nu);
q = zeros(nx,1);
%q = 1*Q*[ones(nx/2,1); zeros(nx/2,1)];
qf = q;
r = zeros(nu,1);
%r = 1*R*ones(nu,1);
QQ = repmat(Q, 1, N);
SS = repmat(S, 1, N);
RR = repmat(R, 1, N);
qq = repmat(q, 1, N);
rr = repmat(r, 1, N);

% general constraints
C  = zeros(ng, nx);
D  = zeros(ng, nu);
CN = zeros(ngN, nx);
%if(ng>0)
%	if(nb==0)
%		for ii=1:min(nu,ng)
%			D(ii,ii) = 1.0;
%		end
%		for ii=min(nu,ng)+1:ng
%			C(ii,ii-nu) = 1.0;
%		end
%	else
%		for ii=1:ng
%			C(ii,ii) = 1.0;
%		end
%	end
%end
if(ngN>0)
	for ii=1:ngN
		CN(ii,ii) = 1.0;
	end
end
CC = [zeros(ng,nx), repmat(C, 1, N-1)];
DD = [repmat(D, 1, N)];

% box constraints
lb = zeros(nb,1);
ub = zeros(nb,1);
for ii=1:nbu
	lb(ii) = -0.5; % lower bound
	ub(ii) =  0.5; % - upper bound
end
for ii=nbu+1:nb
	lb(ii) = -10;
	ub(ii) =  10;
end
llb = repmat(lb, 1, N+1);
uub = repmat(ub, 1, N+1);
% general constraints
lg = zeros(ng,1);
ug = zeros(ng,1);
%if(ng>0)
%	if(nb==0)
%		for ii=1:min(nu,ng)
%			lg(ii) = -2.5; % lower bound
%			ug(ii) = -0.1; % - upper bound
%		end
%		for ii=min(nu,ng)+1:ng
%			lg(ii) = -10;
%			ug(ii) =  10;
%		end
%	else
%		for ii=1:ng
%			lg(ii) = -10;
%			ug(ii) =  10;
%		end
%	end
%end
%db(2*nu+1:end) = -4;
llg = repmat(lg, 1, N);
uug = repmat(ug, 1, N);
% general constraints last stage
lgN = zeros(ngN,1);
ugN = zeros(ngN,1);
if(ngN>0)
	if(nb==0)
%		for ii=1:min(nu,ng)
%			lg(ii) = -2.5; % lower bound
%			ug(ii) = -0.1; % - upper bound
%		end
		for ii=min(nu,ng)+1:ng
			lgN(ii) =   0;
			ugN(ii) =   0;
		end
	else
		for ii=1:ngN
			lgN(ii) =   0;
			ugN(ii) =   0;
		end
	end
end
%if(ng>0)
%	if(nb==0)
%		uub(1:min(nu,ng),N+1) =  0.0;
%	end
%end

% initial guess for states and inputs
x = zeros(nx, N+1); x(:,1) = x0; % initial condition
u = zeros(nu, N);
mult_pi = zeros(nx,N);
mult_lam = zeros(2*(nb+ng)*N+2*(nb+ngN),1);
mult_t = zeros(2*(nb+ng)*N+2*(nb+ngN),1);

free_x0 = 0; % consider x0 as optimization variable
warm_start = 0; % read initial guess from x and u

mu0 = 2;        % max element in cost function as estimate of max multiplier
kk = -1;		% actual number of performed iterations
k_max = 20;		% maximim number of iterations
tol = 1e-8;		% tolerance in the duality measure
infos = zeros(5, k_max);
inf_norm_res = zeros(1, 4);

nrep = 1000; % number of calls to IPM solver for better timings


tic

if time_invariant==0 % time-variant interface

	for ii=1:nrep
		HPIPM_d_solve_ipm2_hard_ocp_qp(kk, k_max, mu0, tol, N, nx, nu, nb, ng, ngN, time_invariant, free_x0, warm_start, AA, BB, bb, QQ, Qf, RR, SS, qq, qf, rr, llb, uub, CC, DD, llg, uug, CN, lgN, ugN, x, u, mult_pi, mult_lam, inf_norm_res, infos);
	end

else % time-invariant interface

	for ii=1:nrep
		HPIPM_d_solve_ipm2_hard_ocp_qp(kk, k_max, mu0, tol, N, nx, nu, nb, ng, ngN, time_invariant, free_x0, warm_start, A, B, b, Q, Qf, R, S, q, qf, r, lb, ub, C, D, lg, ug, CN, lgN, ugN, x, u, mult_pi, mult_lam, inf_norm_res, infos);
	end

end

solution_time = toc/nrep

kk
fprintf('\n  alpha_aff     mu_aff       sigma        alpha        mu\n');
infos(:,1:kk)'
inf_norm_res

u
x


f1 = figure()
plot([0:N], x(:,:))
title('states')
xlabel('N')

% print figure to file
if is_octave()
	W = 4; H = 3;
	set(f1,'PaperUnits','inches')
	set(f1,'PaperOrientation','portrait');
	set(f1,'PaperSize',[H,W])
	set(f1,'PaperPosition',[0,0,W,H])
	FN = findall(f1,'-property','FontName');
	set(FN,'FontName','/usr/share/fonts/truetype/ttf-dejavu/DejaVuSerifCondensed.ttf');
	FS = findall(f1,'-property','FontSize');
	set(FS,'FontSize',10);
	file_name = ['states.eps'];
	print(f1, file_name, '-depsc') 
end


f1 = figure()
plot([1:N], u(:,:))
title('controls')
xlabel('N')

% print figure to file
if is_octave()
	W = 4; H = 3;
	set(f1,'PaperUnits','inches')
	set(f1,'PaperOrientation','portrait');
	set(f1,'PaperSize',[H,W])
	set(f1,'PaperPosition',[0,0,W,H])
	FN = findall(f1,'-property','FontName');
	set(FN,'FontName','/usr/share/fonts/truetype/ttf-dejavu/DejaVuSerifCondensed.ttf');
	FS = findall(f1,'-property','FontSize');
	set(FS,'FontSize',10);
	file_name = ['inputs.eps'];
	print(f1, file_name, '-depsc') 
end


