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
%  solvers, for a Badminton robot/double integrator MPC benchmark problem
%



%%
% Paths and settings
clear all

qpDUNES_PATH = '../../interfaces/matlab';
cplex_PATH = '~/Applications/IBM/ILOG/CPLEX_Studio1251/cplex/matlab';
forces_PATH = './FORCES';

enablePlotting = false;      % true, false


nRuns = 1;
nMPC = 20;


tLog.qpdunes = zeros(nRuns,nMPC);
tLog.forces = zeros(nRuns,nMPC);
tLog.cplex = zeros(nRuns,nMPC);
tLog.quadprog = zeros(nRuns,nMPC);

tLog.nI = zeros(nRuns,1);


% Problem dimensions

nI = 100;           % number of control intervals
nX = 2;             % number of states
nU = 1;             % number of controls
nZ = nX+nU;
nD = zeros(nI+1,1); % number of constraints

recompileFORCES = true;     % true, false


%  Problem settings

INFTY = 1.0e12;
dt = 0.01;              % discretization sampling time 10ms, needed for constraints
	

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
idxArrivalStart = 49;       
idxArrivalEnd = idxArrivalStart + 1;

xLow = repmat( [ -1.9, -3.0 ], 1, nI+1 );
xUpp = repmat( [  1.9,  3.0 ], 1, nI+1 );
uLow = repmat( [ -30.0 ], 1, nI );
uUpp = repmat( [  30.0 ], 1, nI );
xRef = repmat( [  0.0,  0.0 ], 1, nI+1 );
uRef = repmat( [  0.0 ], 1, nI );


if (idxArrivalEnd <= nI)
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



% initial value and QP solution

x0 = [ -1, 0 ];



% log for plotting
xLog = zeros(nMPC*nX,1);
uLog = zeros(nMPC*nU,1);


%% NUMBER OF RUNS LOOP
runIdx = 1;
while runIdx <= nRuns

    % try

    tLog.nI(runIdx) = nI;

    fprintf('+ run %d of %d +\n', runIdx, nRuns);

    % generate noise
    w = 0.05*(rand(nX,nMPC)-0.5);
    % w = w*0;




    %% FOR ALL SOLVERS LOOP
%     for SOLVER = {'QPDUNES', 'FORCES', 'CPLEX', 'QUADPROG'}
    for SOLVER = {'QPDUNES'}


        % reset bounds
        zLow = [ reshape(xLow,[nX,nI+1]);
                 [reshape(uLow,[nU,nI]), zeros(nU,1)] ];
        zLow = reshape( zLow(1:nZ*nI+nX), [1,nZ*nI+nX] );
        zUpp = [ reshape(xUpp,[nX,nI+1]);
                 [reshape(uUpp,[nU,nI]), zeros(nU,1)] ];
        zUpp = reshape( zUpp(1:nZ*nI+nX), [1,nZ*nI+nX] );
        zRef = [ reshape(xRef,[nX,nI+1]);
                 [reshape(uRef,[nU,nI]), zeros(nU,1)] ];
        zRef = reshape( zRef(1:nZ*nI+nX), [1,nZ*nI+nX] );

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


            % -- CLEANUP --
            mpcDUNES( 'cleanup' );
            % -- INIT z-STYLE --
            mpcDUNES( 'init', nI, ...
                      H, P, [], C, cFull, ...
                      zLow, zUpp, zRef, ...
                      qpOptions );


            % -- SOLVE --
            x = x0';
            for mpcIter = 1:nMPC
                % update and solve new QP
                tic;
                mpcDUNES( 'update', ...
                  [], [], [], [], [], ...
                  [], zLow, zUpp, [] );
                [uOpt, xOpt, stat, objFctnVal] = mpcDUNES( 'solve', x );

                tLog.qpdunes(runIdx,mpcIter) = toc*1e3;

                u = uOpt(1:nU);

                % log
                xLog((mpcIter-1)*nX+1:mpcIter*nX) = x;
                uLog((mpcIter-1)*nU+1:mpcIter*nU) = u;

                % simulate system
                x = A*x+B*u;
                x = x.*(1+w(:,mpcIter));

                % prepare new bounds
                zLow = [zLow(nZ+1:end) uLow( 1:nU) xLow(1:nX)];
                zUpp = [zUpp(nZ+1:end) uUpp( 1:nU) xUpp(1:nX)];
            end
            xLog(mpcIter*nX+1:(mpcIter+1)*nX) = x;

            % -- CLEANUP --
            mpcDUNES( 'cleanup' );


            % plotting
            if (enablePlotting)
                t = dt * (0:nMPC);
                linewidth = 2;

                figure; 
                subplot(3,1,1); hold on;
                plot(t,xLog(1:2:end),'b', 'linewidth', linewidth); 
                plot([t(1) t(end)],[xLow(1) xLow(1)], 'm-', 'linewidth', linewidth);     % lb
                plot([t(1) t(end)],[xUpp(1) xUpp(1)], 'm-', 'linewidth', linewidth);     % ub
                subplot(3,1,2); hold on;
                plot(t,xLog(2:2:end),'b', 'linewidth', linewidth); 
                plot([t(1) t(end)],[xLow(2) xLow(2)], 'm-', 'linewidth', linewidth);     % lb
                plot([t(1) t(end)],[xUpp(2) xUpp(2)], 'm-', 'linewidth', linewidth);     % ub
                subplot(3,1,3); hold on;
                plot(t(1:end-1),uLog(1:end),'b', 'linewidth', linewidth);
                plot([t(1) t(end)],[uLow(1) uLow(1)], 'm-', 'linewidth', linewidth);     % lb
                plot([t(1) t(end)],[uUpp(1) uUpp(1)], 'm-', 'linewidth', linewidth);     % ub

                % Arrival constraint
                subplot(3,1,1); 


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

                        % equality constraints
                        stages(i).eq.C = [eye(nX), zeros(nX,nU); A, B];
                        params(1) = newParam('z1',1,'eq.c'); % RHS of first eq. constr. is a parameter: [x0, 0]

                        % cost
                        stages(i).cost.H = blkdiag(Q,R);
                        stages(i).cost.f = zeros(stages(i).dims.n,1);

                        % lower bounds
                        stages(i).ineq.b.lbidx = 1:stages(i).dims.n; % lower bound acts on these indices
                        params(end+1) = newParam(['zLow',num2str(i)],i,'ineq.b.lb');

                        % upper bounds
                        stages(i).ineq.b.ubidx = 1:stages(i).dims.n; % upper bound acts on these indices
                        params(end+1) = newParam(['zUpp',num2str(i)],i,'ineq.b.ub');

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
                        params(end+1) = newParam(['zLow',num2str(i)],i,'ineq.b.lb');

                        % upper bounds
                        stages(i).ineq.b.ubidx = 1:stages(i).dims.n; % upper bound acts on these indices
                        params(end+1) = newParam(['zUpp',num2str(i)],i,'ineq.b.ub');

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
                        params(end+1) = newParam(['zLow',num2str(i)],i,'ineq.b.lb');

                        % upper bounds
                        stages(i).ineq.b.ubidx = 1:stages(i).dims.n; % upper bound acts on these indices
                        params(end+1) = newParam(['zUpp',num2str(i)],i,'ineq.b.ub');

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
            x = x0';
            for mpcIter = 1:nMPC
                % update and solve new QP
                problem.z1 = zeros(2*nX,1);
                problem.z1(1:nX) = x;
                for i = 1:nI
                    problem.(['zLow',num2str(i)]) = zLow((i-1)*nZ+1:i*nZ)';
                    problem.(['zUpp',num2str(i)]) = zUpp((i-1)*nZ+1:i*nZ)';
                end
                problem.(['zLow',num2str(nI+1)]) = zLow(nI*nZ+1:nI*nZ+nX)';
                problem.(['zUpp',num2str(nI+1)]) = zUpp(nI*nZ+1:nI*nZ+nX)';

                tic;
                [solverout,exitflag,info] = forcesMPC(problem);
                tLog.forces(runIdx,mpcIter) = toc*1e3;

                % get u
                a = solverout.(['y', num2str(0), num2str(1)]);
                u = a(nX+1:nX+nU);

                % log
                xLog((mpcIter-1)*nX+1:mpcIter*nX) = x;
                uLog((mpcIter-1)*nU+1:mpcIter*nU) = u;

                % simulate system
                x = A*x+B*u;
                x = x.*(1+w(:,mpcIter));

                % prepare new bounds
                zLow = [zLow(nZ+1:end) uLow( 1:nU) xLow(1:nX)];
                zUpp = [zUpp(nZ+1:end) uUpp( 1:nU) xUpp(1:nX)];
            end
            xLog(mpcIter*nX+1:(mpcIter+1)*nX) = x;


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

                tvec = dt * (0:nMPC);
                for jj=1:2
                    subplot(3,1,jj); hold on;
                    plot(tvec,xLog(jj:2:end), 'g');
                    axis([0,nMPC*dt,1.1*xLow(jj),1.1*xUpp(jj)]); ylabel(['x_',num2str(jj)]);
                end
                for jj=1:1
                    subplot(3,1,jj+2); hold on;
                    stairs(tvec(1:end-1),uLog(jj:1:end), 'r');
                    axis([0,nMPC*dt,1.1*uLow(1),1.1*uUpp(1)]); ylabel(['u_',num2str(jj)]);
                end
                xlabel('t (forces)');
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



            addpath(cplex_PATH);

            cplexOptions = cplexoptimset('cplex');
            cplexOptions.threads = 1;   
            %                 cplexOptions = cplexoptimset('Diagnostics', 'on');
            %                 cplexOptions.qpmethod = 0;  % 0 = auto, 1 = primal AS, 2 = dual AS, 4 = IP
            %                 cplexOptions.barrier.display = 0;
            %                 cplexOptions.diagnostics = 'off';

            x = x0';
            for mpcIter = 1:nMPC
                % initial value
                zLow( 1:nX ) = x;
                zUpp( 1:nX ) = x;
                % update and solve new QP
                tic;
                [zOpt_cplex,fval_cplex,exitflag,output,multOpt] = ...
                    cplexqp( Hfull, zeros(length(Hfull),1), [], [], Afull, cfull, zLow, zUpp, [], cplexOptions);
                tLog.cplex(runIdx,mpcIter) = toc*1e3;



                % get u
                u = zOpt_cplex(nX+1:nZ);

                % log
                xLog((mpcIter-1)*nX+1:mpcIter*nX) = x;
                uLog((mpcIter-1)*nU+1:mpcIter*nU) = u;

                % simulate system
                x = A*x+B*u;
                x = x.*(1+w(:,mpcIter));

                % prepare new bounds
                zLow = [zLow(nZ+1:end) uLow( 1:nU) xLow(1:nX)];
                zUpp = [zUpp(nZ+1:end) uUpp( 1:nU) xUpp(1:nX)];
            end
            xLog(mpcIter*nX+1:(mpcIter+1)*nX) = x;



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


            QPoptions = optimset('LargeScale','on','Display','off', 'Algorithm', 'interior-point-convex');

            x = x0';
            for mpcIter = 1:nMPC
                % initial value
                zLow( 1:nX ) = x;
                zUpp( 1:nX ) = x;

                % update and solve new QP
                tic; 
                [zOpt_quadprog, fval_quadprog, E, O, multOpt] = ...
                    quadprog( Hfull, [], [], [], Afull, cfull, zLow, zUpp, [], QPoptions );
                tLog.quadprog(runIdx,mpcIter) = toc*1e3;


                % get u
                u = zOpt_quadprog(nX+1:nZ);

                % log
                xLog((mpcIter-1)*nX+1:mpcIter*nX) = x;
                uLog((mpcIter-1)*nU+1:mpcIter*nU) = u;

                % simulate system
                x = A*x+B*u;
                x = x.*(1+w(:,mpcIter));

                % prepare new bounds
                zLow = [zLow(nZ+1:end) uLow( 1:nU) xLow(1:nX)];
                zUpp = [zUpp(nZ+1:end) uUpp( 1:nU) xUpp(1:nX)];
            end
            xLog(mpcIter*nX+1:(mpcIter+1)*nX) = x;



            lambdaOpt_quadprog = multOpt.eqlin;
            muOptL_quadprog = multOpt.lower;
            muOptU_quadprog = multOpt.upper;


            if (enablePlotting)
                % plot quadprog

                t = dt * (0:nMPC);
                linewidth = 2;

                figure;  title( 'quadprog solution' );
                subplot(3,1,1); hold on;
                plot(t,xLog(1:nX:end),'b', 'linewidth', linewidth); 
                plot([t(1) t(end)],[xLow(1) xLow(1)], 'm-', 'linewidth', linewidth);     % lb
                plot([t(1) t(end)],[xUpp(1) xUpp(1)], 'm-', 'linewidth', linewidth);     % ub
                subplot(3,1,2); hold on;
                plot(t,xLog(2:nX:end),'b', 'linewidth', linewidth); 
                plot([t(1) t(end)],[xLow(2) xLow(2)], 'm-', 'linewidth', linewidth);     % lb
                plot([t(1) t(end)],[xUpp(2) xUpp(2)], 'm-', 'linewidth', linewidth);     % ub
                subplot(3,1,3); hold on;
                plot(t(1:end-1),uLog(1:nU:end),'b', 'linewidth', linewidth);
                plot([t(1) t(end)],[uLow(1) uLow(1)], 'm-', 'linewidth', linewidth);     % lb
                plot([t(1) t(end)],[uUpp(1) uUpp(1)], 'm-', 'linewidth', linewidth);     % ub


                hold off;
            end
        end

    end % for all solvers

    runIdx = runIdx + 1;

    % catch % catch possibly infeasible problems and redo for new noise vector
    %     
    %     % do not update counter
    %     continue;
    %     
    % end

end % for nRuns




%% end of file



 
 