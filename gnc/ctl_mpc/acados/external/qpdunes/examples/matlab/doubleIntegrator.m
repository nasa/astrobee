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


%% Problem dimensions

nI = 200;           % number of control intervals
nX = 2;             % number of states
nU = 1;             % number of controls
nD = zeros(nI+1,1); % number of constraints
nD(1:nI) = nX+nU;
nD(nI+1) = nX;


%% Problem settings

INFTY = 1.0e12;
ENERGY_OPT = false;
dt = 0.01;              % discretization sampling time 10ms, needed for constraints
	

%% Problem data

if ENERGY_OPT
    Q = [[ 1.01e-0, 0.0     ]
		 [ 0.0    , 1.01e-0 ]];
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
idxArrivalStart = 50;

k = idxArrivalStart+0;
xLow(k*nX+0) = xRef(k*nX+0)-1e-04;
xUpp(k*nX+0) = xRef(k*nX+0)+1e-04;
k = idxArrivalStart+2;
xLow(k*nX+0) = xRef(k*nX+0)-1e-04;
xUpp(k*nX+0) = xRef(k*nX+0)+1e-04;


%% initial value and QP solution

x0 = [ -1, 0 ];
	
	
% mpcProblem.qEpData.options.maxIter    = 10000;
% % mpcProblem.qpData.options.lsType			= QP42_LS_BACKTRACKING_LS;
% % mpcProblem.qpData.options.lsType			= QP42_LS_BACKTRACKING_LS_WITH_AS_CHANGE;
% % mpcProblem.qpData.options.lsType			= QP42_LS_GOLDEN_SECTION_IS;
% % mpcProblem.qpData.options.lsType			= QP42_LS_GRADIENT_BISECTION_IS;
% mpcProblem.qpData.options.lsType			= QP42_LS_ACCELERATED_GRADIENT_BISECTION_IS;
% 	mpcProblem.qpData.options.lineSearchStationarityTolerance = 1.e-3;
	
disp( ['Solving double integrator [nI = ', num2str(nI), ', nX = ', num2str(nX), ', nU = ', num2str(nU), ']'] );

tic
[uOpt, xOpt, stat, objVal] = ...
mpcDUNES( nI, Q, R, P, A, B, c, xLow, xUpp, uLow, uUpp, xRef, uRef, x0 );
toc



%% end of file

 
 