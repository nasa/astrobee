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
%  A run script to compare the computation times of qpDUNES against other
%  solvers, for a Badminton robot/double integrator benchmark problem
%


%%
% Paths and settings

qpDUNES_PATH = '../../interfaces/matlab';
cplex_PATH = '~/Applications/IBM/ILOG/CPLEX_Studio1251/cplex/matlab';
% cplex_PATH = '/opt/ibm/ILOG/CPLEX_Studio1251/cplex/matlab';
forces_PATH = './FORCES';

enablePlotting = false;     % true, false

nRuns = 10;               % number of repeated benchmark runs


% time logging
tLog.qpdunes = zeros(nRuns,1);
tLog.forces = zeros(nRuns,1);
tLog.cplex = zeros(nRuns,1);
tLog.quadprog = zeros(nRuns,1);

tLog.nI = zeros(nRuns,1);


% Problem dimensions
nI = 200;           % number of control intervals
nX = 2;             % number of states
nU = 1;             % number of controls
nZ = nX+nU;


%  Problem settings
INFTY = 1.0e12;
dt = 0.01;              % discretization sampling time 10ms, needed for dynamics

recompileFORCES = true;     % true, false
if ( exist('./forcesMPC', 'dir') == 7 )
    rmdir('forcesMPC', 's')
end


% Problem data
Q = [[ 1.0e-4, 0.0     ]
     [ 0.0    , 1.0e-4 ]];
R = [[ 1.0e0 ]];

P = Q;
	
A = [[ 1.0, 1.0*dt ]
	 [ 0.0, 1.0    ]];	
B = [[ 0.0    ]
     [ 1.0*dt ]];
c = [ 0.0;
      0.0 ];

% arrival constraints
idxArrivalStart = 45;               % 43 and shorter is infeasible, confirmed by quadprog
idxArrivalEnd = idxArrivalStart + 1;



% initial value and QP solution
x0 = [ -1, 0 ];



%% NUMBER OF RUNS LOOP
for runIdx = 1:nRuns
    

% adapt data
nD = zeros(nI+1,1); % number of constraints

xLow = repmat( [ -1.9, -3.0 ], 1, nI+1 );
xUpp = repmat( [  1.9,  3.0 ], 1, nI+1 );
uLow = repmat( [ -30.0 ], 1, nI );
uUpp = repmat( [  30.0 ], 1, nI );
xRef = repmat( [  0.0,  0.0 ], 1, nI+1 );
uRef = repmat( [  0.0 ], 1, nI );


if (idxArrivalEnd < nI)
    k = idxArrivalStart;
    xLow(k*nX+1) = xRef(k*nX+1);
    xUpp(k*nX+1) = xRef(k*nX+1);
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
zLow = [ reshape(xLow,[nX,nI+1]);
         [reshape(uLow,[nU,nI]), zeros(nU,1)] ];
zLow = reshape( zLow(1:nZ*nI+nX), [1,nZ*nI+nX] );
zUpp = [ reshape(xUpp,[nX,nI+1]);
         [reshape(uUpp,[nU,nI]), zeros(nU,1)] ];
zUpp = reshape( zUpp(1:nZ*nI+nX), [1,nZ*nI+nX] );
zRef = [ reshape(xRef,[nX,nI+1]);
         [reshape(uRef,[nU,nI]), zeros(nU,1)] ];
zRef = reshape( zRef(1:nZ*nI+nX), [1,nZ*nI+nX] );




tLog.nI(runIdx) = nI;
fprintf('+ run %d of %d +\n', runIdx, nRuns);

%% FOR ALL SOLVERS LOOP
for SOLVER = {'QPDUNES', 'FORCES', 'CPLEX', 'QUADPROG'}
% for SOLVER = {'QPDUNES'}

if strcmp( SOLVER, 'QPDUNES')
    %% Q P D U N E S

    % add qpDUNES path
    addpath(qpDUNES_PATH);

    % Options
    qpOptions = qpDUNES_options( 'default', ...
                                 'maxIter', 100, ...
                                 'printLevel', 0, ...
                                 'logLevel', 0, ...     % log all data
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


    % -- CLEANUP and INITIAL SETUP --
    mpcDUNES( 'cleanup' );
    mpcDUNES( 'init', nI, ...
              H, P, [], C, cFull, ...
              zLow, zUpp, zRef, ...
              qpOptions );
    
    tic;
    % -- SOLVE --
    [uOpt, xOpt, stat, objFctnVal] = mpcDUNES( 'solve', x0 );
    tLog.qpdunes(runIdx) = toc*1e3;
    
    
    % -- CLEANUP --
    mpcDUNES( 'cleanup' ); 
    

    % plotting
    if (enablePlotting)
        t = dt * (0:nI);
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
    end
    
end


if strcmp( SOLVER, 'FORCES')
    %% F O R C E S

    addpath(forces_PATH);

    % FORCES multistage form - zi = [xi, ui] for i=1...N-1 and zN = xN
    
    if (recompileFORCES)
        recompileFORCES = false;
        
        stages = MultistageProblem(nI+1);

        for i = 1:(nI+1)
            % initial stage
            if( i==1 )

                % dimension
                stages(i).dims.n = nX+nU; % number of stage variables
                stages(i).dims.r = 2*nX;  % number of equality constraints        
                stages(i).dims.l = nX+nU; % number of lower bounds
                stages(i).dims.u = nX+nU; % number of upper bounds
                stages(i).dims.p = 0;     % number of polytopic constraints
                stages(i).dims.q = 0;     % number of quadratic constraints

                % cost
                stages(i).cost.H = blkdiag(Q,R);
                stages(i).cost.f = zeros(stages(i).dims.n,1);

                % lower bounds
                stages(i).ineq.b.lbidx = 1:stages(i).dims.n; % lower bound acts on these indices
                stages(i).ineq.b.lb = zLow(1:nZ); % lower bound for this stage variable

                % upper bounds
                stages(i).ineq.b.ubidx = 1:stages(i).dims.n; % upper bound acts on these indices
                stages(i).ineq.b.ub = zUpp(1:nZ); % upper bound for this stage variable

                % equality constraints
                stages(i).eq.C = [eye(nX), zeros(nX,nU); A, B];
                params(1) = newParam('z1',1,'eq.c'); % RHS of first eq. constr. is a parameter: [x0, 0]

            end

            % stages along horizon
            if( i>1 && i<=nI )       

                % dimension
                stages(i).dims.n = nX+nU; % number of stage variables
                stages(i).dims.r = nX;    % number of equality constraints        
                stages(i).dims.l = nX+nU; % number of lower bounds
                stages(i).dims.u = nX+nU; % number of upper bounds
                stages(i).dims.p = 0;     % number of polytopic constraints
                stages(i).dims.q = 0;     % number of quadratic constraints

                % cost
                stages(i).cost.H = blkdiag(Q,R);
                stages(i).cost.f = zeros(stages(i).dims.n,1);

                % lower bounds
                stages(i).ineq.b.lbidx = 1:stages(i).dims.n; % lower bound acts on these indices
                stages(i).ineq.b.lb = zLow((i-1)*nZ+1:i*nZ); % lower bound for this stage variable

                % upper bounds
                stages(i).ineq.b.ubidx = 1:stages(i).dims.n; % upper bound acts on these indices
                stages(i).ineq.b.ub = zUpp((i-1)*nZ+1:i*nZ); % upper bound for this stage variable

                % equality constraints
                stages(i).eq.C = [A, B];
                stages(i).eq.c = c;
                if( i==2 )
                    stages(i).eq.D = [zeros(nX,nX+nU); -eye(nX), zeros(nX,nU)];
                else
                    stages(i).eq.D = [-eye(nX), zeros(nX,nU)];
                end

            end

            % final stage
            if( i==nI+1 )

                % dimension
                stages(i).dims.n = nX;    % number of stage variables
                stages(i).dims.r = 0;     % number of equality constraints        
                stages(i).dims.l = nX;    % number of lower bounds
                stages(i).dims.u = nX;    % number of upper bounds
                stages(i).dims.p = 0;     % number of polytopic constraints
                stages(i).dims.q = 0;     % number of quadratic constraints

                % cost
                stages(i).cost.H = P;
                stages(i).cost.f = zeros(stages(i).dims.n,1);

                % lower bounds
                stages(i).ineq.b.lbidx = 1:stages(i).dims.n; % lower bound acts on these indices
                stages(i).ineq.b.lb = zLow(nI*nZ+1:nI*nZ+nX); % lower bound for this stage variable

                % upper bounds
                stages(i).ineq.b.ubidx = 1:stages(i).dims.n; % upper bound acts on these indices
                stages(i).ineq.b.ub = zUpp(nI*nZ+1:nI*nZ+nX); % upper bound for this stage variable

                % equality constraints        
                stages(i).eq.D = -eye(nX);

            end
        end

        % solver settings
        codeoptions = getOptions('forcesMPC');

        % printlevel
        codeoptions.printlevel = 0;
        codeoptions.parallel = 0;
        
        % generate code
        generateCode(stages,params,codeoptions);
    end
    
    
    % R U N    F O R C E S
    problem.z1 = zeros(2*nX,1);
    problem.z1(1:nX) = x0;
    tic;
    [solverout,exitflag,info] = forcesMPC(problem);
    tLog.forces(runIdx) = toc*1e3;
    

    

    if (enablePlotting)
        % get solver output
        uOpt = [[]];
        xOpt = [[]];
        for k = 1:9
            a = solverout.(['y', num2str(0), num2str(k)]);
            xOpt = [xOpt; a(1:nX)];
            uOpt = [uOpt; a(nX+1:nX+nU)];
        end
        for k = 10:nI
            a = solverout.(['y', num2str(k)]);
            xOpt = [xOpt; a(1:nX)];
            uOpt = [uOpt; a(nX+1:nX+nU)];
        end
        a = solverout.(['y', num2str(nI+1)]);
        xOpt = [xOpt; a(1:nX)];

        tvec = 0:dt:(nI)*dt;
        for jj=1:2
            subplot(3,1,jj); hold on;
            plot(tvec,xOpt(jj:2:end), 'g');
            axis([0,nI*dt,1.1*xLow(jj),1.1*xUpp(jj)]); ylabel(['x_',num2str(jj)]);
        end
        for jj=1:1
            subplot(3,1,jj+2); hold on;
            stairs(tvec(1:end-1),uOpt(jj:1:end), 'r');
            axis([0,nI*dt,1.1*uLow(1),1.1*uUpp(1)]); ylabel(['u_',num2str(jj)]);
        end
        xlabel('t (forces)');
    else 
        uOpt = solverout.y01(nX+1:nX+nU);
    end

end


if strcmp( SOLVER, 'CPLEX')
    %% C P L E X
    
    Hfull = [];
    for i=1:nI
        Hfull = blkdiag( Hfull, Q, R );
    end
    Hfull = blkdiag( Hfull, Q );

    Ci = [A B];
    E = eye(nX);
    Afull = zeros( nX*nI, nZ*nI+nX );
    for i=1:nI
        Afull( (i-1)*nX+1:i*nX, (i-1)*nZ+1:i*nZ+nX ) = [Ci -E];
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
    
    
    addpath(cplex_PATH);

    %                 cplexOptions = cplexoptimset('cplex');
    %                 cplexOptions = cplexoptimset('Diagnostics', 'on');
    cplexOptions = cplexoptimset();
    %                 cplexOptions.qpmethod = 0;  % 0 = auto, 1 = primal AS, 2 = dual AS, 4 = IP
    %                 cplexOptions.barrier.display = 0;
    %                 cplexOptions.threads = 0;   
    %                 cplexOptions.diagnostics = 'off';
    
    
    tic;
    [zOpt_cplex,fval_cplex,exitflag,output,multOpt] = ...
        cplexqp( Hfull, zeros(length(Hfull),1), [], [], Afull, cfull, zLow, zUpp, [], cplexOptions);
    tLog.cplex(runIdx) = toc*1e3;
    
    uOpt = zOpt_cplex(nX+1:nZ);
    
end


if strcmp( SOLVER, 'QUADPROG')
    %% Q U A D P R O G
    Hfull = [];
    for i=1:nI
        Hfull = blkdiag( Hfull, Q, R );
    end
    Hfull = blkdiag( Hfull, Q );

    Ci = [A B];
    E = eye(nX);
    Afull = zeros( nX*nI, nZ*nI+nX );
    for i=1:nI
        Afull( (i-1)*nX+1:i*nX, (i-1)*nZ+1:i*nZ+nX ) = [Ci -E];
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
    QPoptions = optimset('LargeScale','on','Display','off', 'Algorithm', 'interior-point-convex');
    tic
    [zOpt_quadprog, fval_quadprog, E, O, multOpt] = ...
        quadprog( Hfull, [], [], [], Afull, cfull, zLow, zUpp, [], QPoptions );
    tLog.quadprog(runIdx) = toc*1e3;


    lambdaOpt_quadprog = multOpt.eqlin;
    muOptL_quadprog = multOpt.lower;
    muOptU_quadprog = multOpt.upper;


    if (enablePlotting)
        % plot quadprog

        t = dt * (0:nI);
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
    end
end
    
end % for all solvers

end % for nRuns




%% end of file



 
 