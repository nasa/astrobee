% matrix size
n = 8;

% fix rand seed
rand('seed',0);

% general matrix A
A = rand(n,n);

% symmetric matrix H
H = rand(n,n);
H = H+H';
%eig(H)

% reference
R0 = A' * H * A

A
H
% recursive algorithm
T0 = zeros(n,1);
T1 = zeros(n,n);
R = zeros(n,n);
R(1,1) = A(1,1)' * H(1,1) * A(1,1);
for ii=2:n
	R(ii,1:ii) = A(1:ii-1,ii)' * H(1:ii-1,1:ii-1) * A(1:ii-1,1:ii);
	R(1:ii,ii) = R(ii,1:ii)';
	T0(1:ii) = A(1:ii-1,1:ii)' * H(1:ii-1,ii) + 0.5 * A(ii,1:ii)' * H(ii,ii);
	T1(1:ii,1:ii) = T0(1:ii) * A(ii,1:ii);
	R(1:ii,1:ii) += T1(1:ii,1:ii) + T1(1:ii,1:ii)';
end

R
R-R0
