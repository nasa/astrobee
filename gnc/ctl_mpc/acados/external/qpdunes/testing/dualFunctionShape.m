%% matrices

nX = 2;
nU = 1;
nZ = 3;
nI = 2;

dt = 1.;
	
% Q = [[ 1.0e-0, 0.0     ]
%      [ 0.0   , 1.0e-0 ]];
% R = [[ 1.0e0 ]];
	
A = [[ 1.0, 1.0*dt ]
	 [ 0.0, 1.0    ]];	
B = [[ 0.0    ]
     [ 1.0*dt ]];

 
C = [A B];

C0 = zeros(size(C));

E = [[ 1 0 0 ]
     [ 0 1 0 ]];
 
H = eye(3);



% Afull = [[  C' C0' C0' ]
%          [  E'  C' C0' ]
%          [ C0'  E'  C' ]
%          [ C0' C0'  E' ]];
Afull = [[  C' C0' ]
         [ -E'  C' ]
         [ C0' -E' ]]';


Hfull = diag(ones(nZ*(nI+1),1));

zL = - ones(nZ*(nI+1),1);
zU = ones(nZ*(nI+1),1); 

zL(0*nZ+1) = 0; zU(0*nZ+1) = 0;      % initial position
zL(0*nZ+2) = 0; zU(0*nZ+2) = 0;      % initial velocity

a = 1e-8;
b = 1e-8;

zL(1*nZ+2) = 0.5-a; zU(1*nZ+2) = 0.5+b;
% zL(1*nZ+1) = 0.0-a; zU(1*nZ+1) = 0.0+b;
% zL(2*nZ+1) = 0.5-a;  % terminal position
% zU(2*nZ+1) = 0.5+b;  % terminal position


%% solve with quadprog

[zOpt, FVAL, E, O, multOpt] = ...
    quadprog( Hfull, [], [], [], Afull, zeros(nI*nX,1), zL, zU );

lambdaOpt = multOpt.eqlin
muOptL = multOpt.lower;
muOptU = multOpt.upper;

%% grid

nP = 100;     % nbr of grid points
lambdaL = -1.5;
lambdaU = 2;

nElem = nP^2; % nP^(nI*nX);

grid1D = linspace(lambdaL, lambdaU, nP);

%% get 2D plot

plotIdx = [1 2]

[l1_,l2_] = ndgrid( grid1D, grid1D );
l1_r = reshape(l1_,[1,nElem]);
l2_r = reshape(l2_,[1,nElem]);

if isequal(plotIdx, [1 2])
    l1r = l1_r;
    l2r = l2_r;
    l3r = lambdaOpt(3)*ones(1,nElem);  
    l4r = lambdaOpt(4)*ones(1,nElem);
elseif isequal(plotIdx, [1 3])
    l1r = l1_r;
    l2r = lambdaOpt(2)*ones(1,nElem);  
    l3r = l2_r;
    l4r = lambdaOpt(4)*ones(1,nElem);
elseif isequal(plotIdx, [1 4])
    l1r = l1_r;
    l2r = lambdaOpt(2)*ones(1,nElem);  
    l3r = lambdaOpt(3)*ones(1,nElem);
    l4r = l2_r;
elseif isequal(plotIdx, [2 3])
    l1r = lambdaOpt(1)*ones(1,nElem);  
    l2r = l1_r;
    l3r = l2_r;
    l4r = lambdaOpt(4)*ones(1,nElem);
elseif isequal(plotIdx, [2 4])
    l1r = lambdaOpt(1)*ones(1,nElem);  
    l2r = l1_r;
    l3r = lambdaOpt(3)*ones(1,nElem);
    l4r = l2_r;
elseif isequal(plotIdx, [3 4])
    l1r = lambdaOpt(1)*ones(1,nElem);  
    l2r = lambdaOpt(2)*ones(1,nElem);  
    l3r = l1_r;
    l4r = l2_r;
end

lambdagrid = [l1r; l2r; l3r; l4r];
fStar = dualFunction( Hfull, Afull, zL, zU, lambdagrid );
fS = reshape(fStar,[nP nP]);

mesh(l1_, ...
     l2_, ...
     fS );

%% get 1D plots
C = ['b','k','r','g'];
lambdagrid = [lambdaOpt(1)*ones(1,nP); 
              lambdaOpt(2)*ones(1,nP);
              lambdaOpt(3)*ones(1,nP);
              lambdaOpt(4)*ones(1,nP)];
figure; hold on;
for i = 1:  (nI*nX)
    lambdagrid(i,:) = grid1D;
    fStar = dualFunction( Hfull, Afull, zL, zU, lambdagrid );
    plot( grid1D, fStar, C(i), 'linewidth', 1+(nI*nX+1)/i*.25 );
    
    lambdagrid(i,:) = lambdaOpt(i)*ones(1,nP); 
end
legend('l12,x', 'l12,v', 'l23,x', 'l23,v');
hold off;








