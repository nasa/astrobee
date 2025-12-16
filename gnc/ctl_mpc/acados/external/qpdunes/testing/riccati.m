%% double integrator model:

nx = 2;
nu = 1;

Q = [
[	1.001e-04	0.000e+00	]
[	0.000e+00	1.001e-04	]
];
R = [
[	1.000e+00	]
];
S = [[ 0 0 ]];

A = [[ 1 1 ]
     [ 0 1 ]];
B = [[ 0 ]
     [ 1 ]];




%% simple single integrator model:

nx = 1;
nu = 1;

Q = [[3]];
R = [[2]];
S = [[0]];

A = [[1]];
B = [[1]];

%%


H = diag([diag(Q)' diag(R)']);
C = [A B];  
E = [eye(nx) zeros([nx,nu])];





%% solve riccati recursion

% start at P = 0
P = zeros(nx);
Pnew = P;

for ii = 1:100
    sBar = S' + A'*P*B;
    Pnew = Q + A'*P*A - sBar * ((R +B'*P*B) \ sBar');
    if max(max(Pnew-P)) < 1e-10
        disp(['reached accuracy in iteration ', num2str(ii)]);
        break; 
    end
    P = Pnew;
end

disp('P = ');
disp(P);


%% build up (unconstrained) dual hessian

% P = 1.6455;

PInv = inv(P);

nI = 10;

N = zeros(nI*nx);

HInv = inv(H);

for ii = 1:nI-1
    N((ii-1)*nx+1:ii*nx,(ii-1)*nx+1:ii*nx) = C*HInv*C' + E*HInv*E';
    N(ii*nx+1:(ii+1)*nx,(ii-1)*nx+1:ii*nx) = - C*HInv*E';
    N((ii-1)*nx+1:ii*nx,ii*nx+1:(ii+1)*nx) = - E*HInv*C';
end
N((nI-1)*nx+1:nI*nx,(nI-1)*nx+1:nI*nx) = C*HInv*C' + PInv;


% now play a bit with cholesky decompositions

nChol = chol(N);
rChol = reverseCholesky(N);

rChol