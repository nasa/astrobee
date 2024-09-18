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

nI = 1;             % number of control intervals
nX = 1;             % number of states
nU = 0;             % number of controls
nZ = nX+nU;
nD = zeros(nI+1,1); % number of affine constraints
% nD(1:nI) = nX+nU;
% nD(nI+1) = nX;


%  Problem settings

QUADPROG = true;
	


% build up data z-style
Hi = [1];
H = repmat( Hi, 1, nI );
P = Hi;
Ci = [1];
C = repmat( Ci, 1, nI );
cFull = [0];

zLow = [ 0 ; 1 ];
zUpp = [ 0 ; 1e12];
zRef = [];


% initial value and QP solution

x0 = [ 2 ];
% x0 = [ 0 ];


%% 
% Options

% add qpDUNES path
qpDUNES_PATH = '../../interfaces/matlab';
addpath(qpDUNES_PATH);

qpOptions = qpDUNES_options( 'default', ...
                             'maxIter', 10, ...
                             'printLevel', 3, ...
                             'logLevel', 2, ...     % log all data
                             'lsType', 4, ...       % Accelerated gradient biscection LS
                             'maxNumLineSearchIterations', 25, ...
                             'maxNumLineSearchRefinementIterations', 150, ...
                             'lineSearchMaxStepSize', 1, ...
                             'stationarityTolerance', 1.e-6, ...
                             'regType', 2, ...       % 2 regularize only singular directions; 1 is normalized Levenberg Marquardt
                             'newtonHessDiagRegTolerance', 1.e-8, ...
                             'regParam', .1e-6 ...
                             ...
                             );

	
disp( ['Solving problem [nI = ', num2str(nI), ', nX = ', num2str(nX), ', nU = ', num2str(nU), ']'] );

tStart = tic;

% -- INIT z-STYLE --
mpcDUNES( 'init', nI, ...
          H, P, [], C, cFull, ...
          zLow, zUpp, zRef, ...
          qpOptions );
% -- SOLVE --
[uOpt, xOpt, stat, objFctnVal, qplog] = mpcDUNES( 'solve', x0 );
% -- CLEANUP --
mpcDUNES( 'cleanup' );

tEnd = toc(tStart);

fprintf( 'Elapsed time is %f ms\n', tEnd*1e3 );



%% quadprog for comparison
Hfull = [];
for i=1:nI
    Hfull = blkdiag( Hfull, Hi );
end
Hfull = blkdiag( Hfull, Hi );

E = eye(nX);
Afull = zeros( nX*nI, nZ*nI+nX );
for i=1:nI
    Afull( (i-1)*nX+1:i*nX, (i-1)*nZ+1:i*nZ+nX ) = [Ci -E];
end
bfull = cFull;

% initial value embedding
zLowPassed = zLow;
zLowPassed(1:nX) = x0;
zUppPassed = zUpp;
zUppPassed(1:nX) = x0;


% QPoptions = optimset('LargeScale','on','Display','iter', 'Algorithm', 'active-set');
QPoptions = optimset('LargeScale','on','Display','iter', 'Algorithm', 'interior-point-convex');
tic
[zOpt_quadprog, fval_quadprog, E, O, multOpt] = ...
    quadprog( Hfull, [], [], [], Afull, bfull, zLowPassed, zUppPassed, [], QPoptions );
toc

lambdaOpt_quadprog = multOpt.eqlin;
muOptL_quadprog = multOpt.lower;
muOptU_quadprog = multOpt.upper;



%% end of file



 
 