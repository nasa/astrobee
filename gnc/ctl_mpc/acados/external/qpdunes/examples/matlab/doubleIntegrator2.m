%	This file is part of qpDUNES.
%
%	qpDUNES -- An Implementation of the Dual Nonsmooth Newton Strategy.
%	Copyright (C) 2012 by Janick Frasch, Hans Joachim Ferreau et al. 
%	All rights reserved.
%
%	qpDUNES is free software; you can redistribute it and/or
%	modify it under the terms of the GNU Lesser General Public
%	License as published by the Free Software Foundation; either
%	version 2.1 of the License, or (at your option) any later version.
%
%	qpDUNES is distributed in the hope that it will be useful,
%	but WITHOUT ANY WARRANTY; without even the implied warranty of
%	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
%	See the GNU Lesser General Public License for more details.
%
%	You should have received a copy of the GNU Lesser General Public
%	License along with qpDUNES; if not, write to the Free Software
%	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
%

%% Abstract
%
%  A simple double integrator model for a badminton robot
%


%%
% Problem dimensions

nI = 50;           % number of control intervals
nX = 2;             % number of states
nU = 1;             % number of controls
nZ = nX+nU;
nD = zeros(nI+1,1); % number of constraints
% nD(1:nI) = nX+nU;
% nD(nI+1) = nX;


%  Problem settings

INFTY = 1.0e12;
ENERGY_OPT = true;
QUADPROG = true;
dt = 0.01;              % discretization sampling time 10ms, needed for constraints
	

% Problem data

if ENERGY_OPT
    Q = [[ 1.001e-4, 0.0     ]
		 [ 0.0    , 1.001e-4 ]];
    R = [[ 1.0e0 ]];
else
	Q = [[ 1.0e1 , 0.0    ]
         [ 0.0   , 1.0e-4 ]];
    R = [[ 1.0e-4 ]];
end	
P = Q;
	
A = [[ 1.0, 1.0*dt ]
	 [ 0.0, 1.0    ]];	
B = [[ 0.0    ]
     [ 1.0*dt ]];
c = [ 0.0;
      0.0 ];

xLow = repmat( [ -1.9, -3.0 ], 1, nI+1 );
xUpp = repmat( [  1.9,  3.0 ], 1, nI+1 );
uLow = repmat( [ -30.0 ], 1, nI );
uUpp = repmat( [  30.0 ], 1, nI );
xRef = repmat( [  0.0,  0.0 ], 1, nI+1 );
uRef = repmat( [  0.0 ], 1, nI );

% arrival constraints
idxArrivalStart = 44;       % 43 and shorter is infeasible, confirmed by quadprog
% idxArrivalStart = 39;
idxArrivalEnd = idxArrivalStart + 1;

if (idxArrivalEnd < nI)
    k = idxArrivalStart;
    xLow(k*nX+1) = xRef(k*nX+1);
    xUpp(k*nX+1) = xRef(k*nX+1);
    % xLow(k*nX+2) = xRef(k*nX+2);
    % xUpp(k*nX+2) = xRef(k*nX+2);
    k = idxArrivalEnd;
    xLow(k*nX+1) = xRef(k*nX+1);
    xUpp(k*nX+1) = xRef(k*nX+1);
end


% build up data z-style
Hi = blkdiag(Q, R);
H = repmat( Hi, 1, nI );
Ci = [A B];
C = repmat( Ci, 1, nI );
cFull = repmat( c, nI, 1 );
% ziLow = [ xLow(1:nX) uLow(1:nU) ];
% ziUpp = [ xUpp(1:nX) uUpp(1:nU) ];
% ziRef = [ xRef(1:nX) uRef(1:nU) ];
zLow = [ reshape(xLow,[nX,nI+1]);
         [reshape(uLow,[nU,nI]), zeros(nU,1)] ];
zLow = reshape( zLow(1:nZ*nI+nX), [1,nZ*nI+nX] );
zUpp = [ reshape(xUpp,[nX,nI+1]);
         [reshape(uUpp,[nU,nI]), zeros(nU,1)] ];
zUpp = reshape( zUpp(1:nZ*nI+nX), [1,nZ*nI+nX] );
zRef = [ reshape(xRef,[nX,nI+1]);
         [reshape(uRef,[nU,nI]), zeros(nU,1)] ];
zRef = reshape( zRef(1:nZ*nI+nX), [1,nZ*nI+nX] );
% zLow = [ repmat( ziLow, 1, nI ) xLow(1:nX) ];
% zUpp = [ repmat( ziUpp, 1, nI ) xUpp(1:nX) ];
% zRef = [ repmat( ziRef, 1, nI ) xRef(1:nX) ];


% initial value and QP solution

x0 = [ -1, 0 ];
% x0 = [ -9.699999739663e-1,   1.500000000099e0 ];

%% 
% Options

% add qpDUNES path
qpDUNES_PATH = '../../interfaces/matlab';
addpath(qpDUNES_PATH);

qpOptions = qpDUNES_options( 'default', ...
                             'maxIter', 100, ...
                             'printLevel', 3, ...
                             'logLevel', 2, ...     % log all data
                             'lsType', 4, ...       % Accelerated gradient biscection LS
                             'maxNumLineSearchIterations', 25, ...
                             'maxNumLineSearchRefinementIterations', 150, ...
                             'lineSearchMaxStepSize', 1, ...
                             'stationarityTolerance', 1.e-6, ...
                             'regType', 2, ...       % regularize only singular directions; 1 is normalized Levenberg Marquardt
                             'newtonHessDiagRegTolerance', 1.e-8, ...
                             'regParam', .1e-6 ...
                             ...
                             );

	
disp( ['Solving double integrator [nI = ', num2str(nI), ', nX = ', num2str(nX), ', nU = ', num2str(nU), ']'] );

tStart = tic;

% -- INIT --
% mpcDUNES( 'init', nI, ...
%           Q, R, P, ...
%           A, B, c, ...
%           xLow, xUpp, uLow, uUpp, ...
%           xRef, uRef, ...
%           qpOptions );
% -- SOLVE --
% [uOpt, xOpt, stat, objFctnVal, qplog] = mpcDUNES( 'solve', x0 );
% -- CLEANUP --
mpcDUNES( 'cleanup' );


% -- INIT z-STYLE --
mpcDUNES( 'init', nI, ...
          ... %H, P, [], C, cFull, ...
          H, P, zeros(nZ*nI+nX,1), C, cFull, ...
          zLow, zUpp, zRef, ...
          qpOptions );
% -- UPDATE --
% mpcDUNES( 'update', ...
%           H, P, [], C, c, ...
%           [], zLow, zUpp, zRef );
% -- SOLVE --
[uOpt, xOpt, stat, objFctnVal, qplog] = mpcDUNES( 'solve', x0 );
% -- CLEANUP --
mpcDUNES( 'cleanup' );
%       mpcDUNES( 'init', nI, ...
%           Q, R, P, ...
%           A, B, c, ...
%           xLow, xUpp, uLow, uUpp, ...
%           xRef, uRef, ...
%           x0, ...
%           qpOptions );
tEnd = toc(tStart);

fprintf( 'Elapsed time is %f ms\n', tEnd*1e3 );

keyboard;

%% plotting

t = dt * (1:nI+1);
linewidth = 2;

figure; 
subplot(3,1,1); hold on;
plot(t,xOpt(1:2:end),'b', 'linewidth', linewidth); 
plot([t(1) t(end)],[xLow(1) xLow(1)], 'm-', 'linewidth', linewidth);     % lb
plot([t(1) t(end)],[xUpp(1) xUpp(1)], 'm-', 'linewidth', linewidth);     % ub
subplot(3,1,2); hold on;
plot(t,xOpt(2:2:end),'b', 'linewidth', linewidth); 
plot([t(1) t(end)],[xLow(2) xLow(2)], 'm-', 'linewidth', linewidth);     % lb
plot([t(1) t(end)],[xUpp(2) xUpp(2)], 'm-', 'linewidth', linewidth);     % ub
subplot(3,1,3); hold on;
plot(t(1:end-1),uOpt(1:end),'b', 'linewidth', linewidth);
plot([t(1) t(end)],[uLow(1) uLow(1)], 'm-', 'linewidth', linewidth);     % lb
plot([t(1) t(end)],[uUpp(1) uUpp(1)], 'm-', 'linewidth', linewidth);     % ub

% Arrival constraint
subplot(3,1,1); 

pos = [t(idxArrivalStart) ...                   % x
       xUpp(nX*idxArrivalStart+1) ...           % y
       t(idxArrivalEnd)-t(idxArrivalStart) ...  % w
       xUpp(nX*(idxArrivalStart-1)+1)-xUpp(nX*idxArrivalStart+1) ...    % h
       ];
rectangle('pos',pos ,'facecolor',[.8 .8 .8],'edgecolor','none');
pos = [t(idxArrivalStart) ...                   % x
       xLow(nX*(idxArrivalStart-1)+1) ...       % y
       t(idxArrivalEnd)-t(idxArrivalStart) ...  % w
       xLow(nX*idxArrivalStart+1)-xLow(nX*(idxArrivalStart-1)+1) ...    % h
       ];
rectangle('pos',pos ,'facecolor',[.8 .8 .8],'edgecolor','none');
 
hold off;


%% quadprog for comparison
Hfull = [];
for i=1:nI
    Hfull = blkdiag( Hfull, Q, R );
end
Hfull = blkdiag( Hfull, Q );

C = [A B];
E = eye(nX);
Afull = zeros( nX*nI, nZ*nI+nX );
for i=1:nI
    Afull( (i-1)*nX+1:i*nX, (i-1)*nZ+1:i*nZ+nX ) = [C -E];
end
cfull = repmat(c, nI,1);

zLow = zeros( 1, nZ*nI+nX );
zUpp = zeros( 1, nZ*nI+nX );
for i=1:nI
    zLow( (i-1)*nZ+1:(i-1)*nZ+nX ) = xLow( (i-1)*nX+1:i*nX );
    zLow( (i-1)*nZ+nX+1:i*nZ ) = uLow( (i-1)*nU+1:i*nU );
    zUpp( (i-1)*nZ+1:(i-1)*nZ+nX ) = xUpp( (i-1)*nX+1:i*nX );
    zUpp( (i-1)*nZ+nX+1:i*nZ ) = uUpp( (i-1)*nU+1:i*nU );
end
zLow( nI*nZ+1:nI*nZ+nX ) = xLow( nI*nX+1:(nI+1)*nX );
zUpp( nI*nZ+1:nI*nZ+nX ) = xUpp( nI*nX+1:(nI+1)*nX );
% initial value
zLow( 1:nX ) = x0;
zUpp( 1:nX ) = x0;


% QPoptions = optimset('LargeScale','on','Display','iter', 'Algorithm', 'active-set');
QPoptions = optimset('LargeScale','on','Display','iter', 'Algorithm', 'interior-point-convex');
tic
[zOpt_quadprog, fval_quadprog, E, O, multOpt] = ...
    quadprog( Hfull, [], [], [], Afull, cfull, zLow, zUpp, [], QPoptions );
toc

lambdaOpt_quadprog = multOpt.eqlin;
muOptL_quadprog = multOpt.lower;
muOptU_quadprog = multOpt.upper;


% plot quadprog

t = dt * (1:nI+1);
linewidth = 2;

figure;  title( 'quadprog solution' );
subplot(3,1,1); hold on;
plot(t,zOpt_quadprog(1:3:end),'b', 'linewidth', linewidth); 
plot([t(1) t(end)],[xLow(1) xLow(1)], 'm-', 'linewidth', linewidth);     % lb
plot([t(1) t(end)],[xUpp(1) xUpp(1)], 'm-', 'linewidth', linewidth);     % ub
subplot(3,1,2); hold on;
plot(t,zOpt_quadprog(2:3:end),'b', 'linewidth', linewidth); 
plot([t(1) t(end)],[xLow(2) xLow(2)], 'm-', 'linewidth', linewidth);     % lb
plot([t(1) t(end)],[xUpp(2) xUpp(2)], 'm-', 'linewidth', linewidth);     % ub
subplot(3,1,3); hold on;
plot(t(1:end-1),zOpt_quadprog(3:3:end),'b', 'linewidth', linewidth);
plot([t(1) t(end)],[uLow(1) uLow(1)], 'm-', 'linewidth', linewidth);     % lb
plot([t(1) t(end)],[uUpp(1) uUpp(1)], 'm-', 'linewidth', linewidth);     % ub

% Arrival constraint
subplot(3,1,1); 

pos = [t(idxArrivalStart) ...                   % x
       xUpp(nX*idxArrivalStart+1) ...           % y
       t(idxArrivalEnd)-t(idxArrivalStart) ...  % w
       xUpp(nX*(idxArrivalStart-1)+1)-xUpp(nX*idxArrivalStart+1) ...    % h
       ];
rectangle('pos',pos ,'facecolor',[.8 .8 .8],'edgecolor','none');
pos = [t(idxArrivalStart) ...                   % x
       xLow(nX*(idxArrivalStart-1)+1) ...       % y
       t(idxArrivalEnd)-t(idxArrivalStart) ...  % w
       xLow(nX*idxArrivalStart+1)-xLow(nX*(idxArrivalStart-1)+1) ...    % h
       ];
rectangle('pos',pos ,'facecolor',[.8 .8 .8],'edgecolor','none');
 
hold off;


%% end of file



 
 