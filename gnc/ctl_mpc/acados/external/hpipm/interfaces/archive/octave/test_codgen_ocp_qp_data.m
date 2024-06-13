%

% horizon length
N = 5;

% dims
dims = create_ocp_qp_dims(N);

% number of states
for ii=1:N+1
	dims.nx(ii) = 2;
end
% number of inputs
for ii=1:N
	dims.nu(ii) = 1;
end
% number of state box constraints
for ii=1:N+1
	dims.nbx(ii) = dims.nx(ii);
end
% number of input box constraints
for ii=1:N
	dims.nbu(ii) = dims.nu(ii);
end
% number of general constraints
dims.ng(N+1) = 2;
% number of general constraints which are softed
dims.ns(N+1) = dims.ng(N+1);

%dims


% data
% 
A = [1 1; 0 1];
%
B = [0; 1];
%
b = [0; 0];
%
Q = [1 0; 0 1];
%
S = [0 0];
%
R = [1];
%
q = [1; 1];
%
r = [0];
%
Jx0 = [1 0; 0 1];
%
x0 = [1; 1];
%
Jx = [1 0; 0 1];
%
lx = [-5; -5];
%
ux = [5; 5];
%
Ju = [1];
%
lu = [-0.5];
%
uu = [0.5];
%
CN = [1 0; 0 1];
%
lgN = [0; 0];
%
ugN = [0; 0];
%
ZlN = [1e6 0; 0 1e6];
%
ZuN = [1e6 0; 0 1e6];
%
zlN = [0; 0];
%
zuN = [0; 0];
%
JsgN = [1 0; 0 1];

%
slN = [1; 1];
%
suN = [1; 1];


% qp
%
qp = create_ocp_qp(dims);

% A
for ii=1:N
	qp.A{ii} = A;
end
% B
for ii=1:N
	qp.B{ii} = B;
end
% b
for ii=1:N
	qp.b{ii} = b;
end
% Q
for ii=1:N+1
	qp.Q{ii} = Q;
end
% R
for ii=1:N
	qp.R{ii} = R;
end
% S
for ii=1:N
	qp.S{ii} = S;
end
% q
for ii=1:N+1
	qp.q{ii} = q;
end
% r
for ii=1:N
	qp.r{ii} = r;
end
% Jx
qp.Jx{1} = Jx0;
for ii=2:N+1
	qp.Jx{ii} = Jx;
end
% lx
qp.lx{1} = x0;
for ii=2:N+1
	qp.lx{ii} = lx;
end
% ux
qp.ux{1} = x0;
for ii=2:N+1
	qp.ux{ii} = ux;
end
% Ju
for ii=1:N
	qp.Ju{ii} = Ju;
end
% lu
for ii=1:N
	qp.lu{ii} = lu;
end
% uu
for ii=1:N
	qp.uu{ii} = uu;
end
%
qp.C{N+1} = CN;
%
qp.lg{N+1} = lgN;
%
qp.ug{N+1} = ugN;
%
qp.Zl{N+1} = ZlN;
%
qp.Zu{N+1} = ZuN;
%
qp.zl{N+1} = zlN;
%
qp.zu{N+1} = zuN;
%
qp.Jsg{N+1} = JsgN;

%qp


% sol_guess
%
sol_guess = create_ocp_qp_sol(dims);
%
sol_guess.sl{N+1} = slN;
%
sol_guess.su{N+1} = suN;


% codgen data
codegen_ocp_qp_data(dims, qp, sol_guess);

