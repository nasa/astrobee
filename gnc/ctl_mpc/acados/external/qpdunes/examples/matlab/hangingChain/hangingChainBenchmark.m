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
%  solvers, for a "Control of hanging chain" benchmark problem
%


%% hanging chain benchmark

%% clear data

clear all;
close all;


%% Paths and settings

qpDUNES_PATH = '../../../interfaces/matlab';
cplex_PATH = '~/Applications/IBM/ILOG/CPLEX_Studio1251/cplex/matlab';
cplex_PATH = '/opt/ibm/ILOG/CPLEX_Studio1251/cplex/matlab';
forces_PATH = './FORCES';


nRuns = 100;      % number of benchmark runs
nMPC = 50;        % number of MPC iterations


nMasses = 5;
nI = 50;                % number of Interval
nX = 3*(2*nMasses+1);   % number of states
nU = 3;                 % number of controls
nZ = nX+nU;             % number of stage variables


% prepare tLog
tLog.qpdunes = zeros(nRuns,nMPC);
tLog.forces = zeros(nRuns,nMPC);
tLog.cplex = zeros(nRuns,nMPC);
tLog.quadprog = zeros(nRuns,nMPC);


%% load hanging chain data

hangingChain_M5_N50_dataSteadyState;
hangingChain_M5_N50_dataInit;


% refactor data (LTI system)
H = repmat( H, 1, nI/50);
g = zeros(nI*nZ+nX,1);
C = repmat( C, 1, nI/50);
c = zeros(nI*nX,1);

zLb = repmat( zLb, 1, nI/50);
zUb = repmat( zUb, 1, nI/50);


% for simulation
Ci = C(:,1:nZ);
x0 = z0Lb(1:nX);


% recompile forces?
recompileFORCES = true;
resetInstance = false;


%% MAIN RUN LOOP
for runIdx = 1:nRuns
    display(['run ', num2str(runIdx)]);


    % generate noise
    w = zeros(nX,nMPC);
    w(3*nMasses+1:6*nMasses,:) = 0.005*(rand(3*nMasses,nMPC)+1); % noise on velocities
    % w = w*0;


    % for SOLVER = {'CPLEX', 'FORCES', 'QPDUNES', 'QUADPROG'}
    % for SOLVER = {'FORCES', 'QPDUNES', 'QUADPROG'}
    % for SOLVER = {'QUADPROG', 'CPLEX'}
    for SOLVER = {'QPDUNES'} 

        if ( resetInstance )
            break;
            resetInstance = false;
        end


        %%  Q P D U N E S
        if strcmp( SOLVER, 'QPDUNES')
        %%

        % data setup
        nD = zeros(nI+1,1); % number of constraints


        % add qpDUNES path
        addpath(qpDUNES_PATH);

        % Options
        qpOptions = qpDUNES_options( 'default', ...
                                     'maxIter', 100, ...
                                     'printLevel', 0, ...
                                     'logLevel', 0, ...     % log all data
                                     'stationarityTolerance', 1.e-6, ...
                                     'regType', 0 ...       % regularize only singular directions; 0 is Levenberg Marquardt
                                     ...
                                     );


        % tic;
        % -- CLEANUP --
        mpcDUNES( 'cleanup' );
        % -- INIT z-STYLE --
        mpcDUNES( 'init', nI, ...
                  H, P, [], C, [], ...
                  zLb, zUb, [], ...
                  qpOptions );
        % tLog.qpdunes_setup(runIdx) = toc*1e3;
        % toc

        zLog_qpdunes = zeros(nZ,nMPC+1);
        x = x0';

        % -- MPC LOOP --
        for mpcIter = 1:nMPC    
            % update and solve new QP
            tic;
            [uOpt, xOpt, stat, objFctnVal] = mpcDUNES( 'solve', x );
            tLog.qpdunes(runIdx,mpcIter) = toc*1e3;

            if (stat ~= 1)
                warning(['qpdunes had an error in mpc iteration ', num2str(mpcIter)]);
        %         runIdx = runIdx - 1;
        %         break;
            end

            u = uOpt(1:nU);

            % log
            zLog_qpdunes(:,mpcIter) = [x;u];


            % simulate system
            x = Ci * [x;u];
            x = x + w(:,mpcIter);
        end

        zLog_qpdunes(1:nX,mpcIter+1) = x;


        tic
        % -- CLEANUP --
        mpcDUNES( 'cleanup' );
        tCleanup = toc;


        zOpt_qpdunes = [ reshape(xOpt,nX,nI+1);[reshape(uOpt,nU,nI) zeros(nU,1)] ];

        % zOpt = zOpt_qpdunes;
        zOpt = zLog_qpdunes;

        display('qpDUNES done.');

        end  % QPDUNES


        %% F O R C E S
        if strcmp( SOLVER, 'FORCES')

        % add FORCES path
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
        %                 stages(i).dims.l = nX+nU; % number of lower bounds
        %                 stages(i).dims.u = nX+nU; % number of upper bounds
                    stages(i).dims.l = nMasses+1+nU; % number of lower bounds
                    stages(i).dims.u = 3; % number of upper bounds
                    stages(i).dims.p = 0;     % number of polytopic constraints
                    stages(i).dims.q = 0;     % number of quadratic constraints

                    % cost
                    stages(i).cost.H = H(:,(i-1)*nZ+1:i*nZ);
                    stages(i).cost.f = g((i-1)*nZ+1:i*nZ)';

                    % lower bounds
        %                 stages(i).ineq.b.lbidx = 1:stages(i).dims.n; % lower bound acts on these indices
        %                 stages(i).ineq.b.lb = zLb_forces(1:nZ)'; % lower bound for this stage variable
                    stages(i).ineq.b.lbidx = [2:3:3*nMasses 6*nMasses+2 nX+1:nX+nU]; % lower bound acts on these indices
                    stages(i).ineq.b.lb = zLb(stages(i).ineq.b.lbidx)'; % lower bound for this stage variable

                    % upper bounds
        %                 stages(i).ineq.b.ubidx = 1:stages(i).dims.n; % upper bound acts on these indices
        %                 stages(i).ineq.b.ub = zUb_forces(1:nZ)'; % upper bound for this stage variable
                    stages(i).ineq.b.ubidx = nX+1:nX+nU; % upper bound acts on these indices
                    stages(i).ineq.b.ub = zUb(stages(i).ineq.b.ubidx)'; % upper bound for this stage variable

                    % equality constraints
                    stages(i).eq.C = [eye(nX), zeros(nX,nU); C(:,(i-1)*nZ+1:i*nZ)];
                    params(1) = newParam('z1',1,'eq.c'); % RHS of first eq. constr. is a parameter: [x0, 0]

                end

                % stages along horizon
                if( i>1 && i<=nI )       

                    % dimension
                    stages(i).dims.n = nX+nU; % number of stage variables
                    stages(i).dims.r = nX;    % number of equality constraints        
        %                 stages(i).dims.l = nX+nU; % number of lower bounds
        %                 stages(i).dims.u = nX+nU; % number of upper bounds
                    stages(i).dims.l = nMasses+1+nU; % number of lower bounds
                    stages(i).dims.u = nU; % number of upper bounds 
                    stages(i).dims.p = 0;     % number of polytopic constraints
                    stages(i).dims.q = 0;     % number of quadratic constraints

                    % cost
                    stages(i).cost.H = H(:,(i-1)*nZ+1:i*nZ);
                    stages(i).cost.f = g((i-1)*nZ+1:i*nZ)';

                    % lower bounds
        %                 stages(i).ineq.b.lbidx = 1:stages(i).dims.n; % lower bound acts on these indices
        %                 stages(i).ineq.b.lb = zLb_forces((i-1)*nZ+1:i*nZ)'; % lower bound for this stage variable
                    stages(i).ineq.b.lbidx = [2:3:3*nMasses 6*nMasses+2 nX+1:nX+nU]; % lower bound acts on these indices
                    stages(i).ineq.b.lb = zLb(stages(i).ineq.b.lbidx)'; % lower bound for this stage variable

                    % upper bounds
        %                 stages(i).ineq.b.ubidx = 1:stages(i).dims.n; % upper bound acts on these indices
        %                 stages(i).ineq.b.ub = zUb_forces((i-1)*nZ+1:i*nZ)'; % upper bound for this stage variable
                    stages(i).ineq.b.ubidx = nX+1:nX+nU; % upper bound acts on these indices
                    stages(i).ineq.b.ub = zUb(stages(i).ineq.b.ubidx)'; % upper bound for this stage variable


                    % equality constraints
                    stages(i).eq.C = C(:,(i-1)*nZ+1:i*nZ);
        %                 stages(i).eq.c = c((i-1)*nX+1:i*nX)';
                    stages(i).eq.c = zeros(nX,1);
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
        %                 stages(i).dims.l = nX;    % number of lower bounds
        %                 stages(i).dims.u = nX;    % number of upper bounds
                    stages(i).dims.l = nMasses+1+nU;    % number of lower bounds
                    stages(i).dims.u = nU;    % number of upper bounds
                    stages(i).dims.p = 0;     % number of polytopic constraints
                    stages(i).dims.q = 0;     % number of quadratic constraints

                    % cost
                    stages(i).cost.H = P;
                    stages(i).cost.f = g(nI*nZ+1:nI*nZ+nX)';

                    % lower bounds
        %                 stages(i).ineq.b.lbidx = 1:stages(i).dims.n; % lower bound acts on these indices
        %                 stages(i).ineq.b.lb = zLb_forces(nI*nZ+1:nI*nZ+nX)'; % lower bound for this stage variable
                    stages(i).ineq.b.lbidx = [2:3:3*nMasses 6*nMasses+2 nX+1:nX+nU]; % lower bound acts on these indices
                    stages(i).ineq.b.lb = zLb(stages(i).ineq.b.lbidx)'; % lower bound for this stage variable


                    % upper bounds
        %                 stages(i).ineq.b.ubidx = 1:stages(i).dims.n; % upper bound acts on these indices
        %                 stages(i).ineq.b.ub = zUb_forces(nI*nZ+1:nI*nZ+nX)'; % upper bound for this stage variable
                    stages(i).ineq.b.ubidx = nX+1:nX+nU; % upper bound acts on these indices
                    stages(i).ineq.b.ub = zUb(stages(i).ineq.b.ubidx)'; % upper bound for this stage variable

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


        %% R U N    F O R C E S


        % -- MPC LOOP --
        x = x0';
        zLog_forces = zeros(nZ,nMPC+1);

        for mpcIter = 1:nMPC
            % update problem data
            problem.z1 = zeros(2*nX,1);
            problem.z1(1:nX) = x;


            % solve
            tic;
            [solverout,exitflag,info] = forcesMPC(problem);
            tLog.forces(runIdx,mpcIter) = toc*1e3;
            if (exitflag ~= 1)
                warning(['forces had an error in mpc iteration ', num2str(mpcIter)]);
                runIdx = runIdx - 1;
                resetInstance = true;
                break;
            end


            % get solver output
            zOpt_forces = zeros(nZ,nI+1);
            for k = 1:9
                a = solverout.(['y', num2str(0), num2str(k)]);
                zOpt_forces(:,k) = a(1:nX+nU);
            end
            for k = 10:nI
                a = solverout.(['y', num2str(k)]);
                zOpt_forces(:,k) = a(1:nX+nU);
            end
            a = solverout.(['y', num2str(nI+1)]);
            zOpt_forces(1:nX,k) = a(1:nX);


            % get first control inputs
            u = solverout.y01(nX+1:nX+nU);

            % log
            zLog_forces(:,mpcIter) = [x;u];


            % simulate system
            x = Ci * [x;u];
            x = x + w(:,mpcIter);
        end

        % for plotting
        % zOpt = zOpt_forces;
        zOpt = zLog_forces;

        display('FORCES done.');

        end % FORCES


        %% C P L E X
        if strcmp( SOLVER, 'CPLEX')
        %%


        Hfull = zeros(nI*nZ+nX,nI*nZ+nX);
        for i=1:nI
            Hfull((i-1)*nZ+1:i*nZ,(i-1)*nZ+1:i*nZ) = ...
                H(:,(i-1)*nZ+1:i*nZ);
        end
        Hfull(nI*nZ+1:nI*nZ+nX,nI*nZ+1:nI*nZ+nX) = P;


        E = eye(nX);
        Afull = zeros( nX*nI, nZ*nI+nX );
        for i=1:nI
            Cii = C(:,(i-1)*nZ+1:i*nZ);
            Afull( (i-1)*nX+1:i*nX, (i-1)*nZ+1:i*nZ+nX ) = [Cii -E];
        end

        zLbFull = zLb;
        zUbFull = zUb;


        addpath(cplex_PATH);

        cplexOptions = cplexoptimset('cplex');
        cplexOptions.threads = 1;   
        %                 cplexOptions = cplexoptimset('Diagnostics', 'on');
        %                 cplexOptions.qpmethod = 0;  % 0 = auto, 1 = primal AS, 2 = dual AS, 4 = IP
        %                 cplexOptions.barrier.display = 0;
        % cplexOptions.diagnostics = 'on';


        % -- MPC LOOP --
        x = x0';
        zLog_cplex = zeros(nZ,nMPC+1);

        for mpcIter = 1:nMPC
            % initial value
            zLbFull( 1:nX ) = x;
            zUbFull( 1:nX ) = x;
            % update and solve new QP
            tic;
            [zOpt_cplex,fval_cplex,exitflag,output,multOpt] = ...
                cplexqp( Hfull, g, [], [], Afull, c, zLbFull, zUbFull, [], cplexOptions);
            tLog.cplex(runIdx,mpcIter) = toc*1e3;
            if (exitflag ~= 1)
                warning(['cplex had an error in mpc iteration ', num2str(mpcIter)]);
                runIdx = runIdx - 1;
                break;
            end


            % get u
            u = zOpt_cplex(nX+1:nZ);


            % log
            zLog_cplex(:,mpcIter) = [x;u];

            % simulate system
            x = Ci * [x;u];
            x = x + w(:,mpcIter);
        end

        zLog_cplex(1:nX,mpcIter+1) = x;

        zOpt = reshape( [zOpt_cplex; zeros(nU,1)], nZ, nI+1 );


        % zOpt = zLog_cplex;

        display('CPLEX done.');


        end


        %%  Q U A D P R O G
        if strcmp( SOLVER, 'QUADPROG')
        %%


        Hfull = zeros(nI*nZ+nX,nI*nZ+nX);
        for i=1:nI
            Hfull((i-1)*nZ+1:i*nZ,(i-1)*nZ+1:i*nZ) = ...
                H(:,(i-1)*nZ+1:i*nZ);
        end
        Hfull(nI*nZ+1:nI*nZ+nX,nI*nZ+1:nI*nZ+nX) = P;


        E = eye(nX);
        Afull = zeros( nX*nI, nZ*nI+nX );
        for i=1:nI
            Cii = C(:,(i-1)*nZ+1:i*nZ);
            Afull( (i-1)*nX+1:i*nX, (i-1)*nZ+1:i*nZ+nX ) = [Cii -E];
        end

        zLbFull = zLb;
        zUbFull = zUb;


        QPoptions = optimset('LargeScale','on','Display','off', 'Algorithm', 'interior-point-convex');


        % -- MPC LOOP --
        x = x0';
        zLog_quadprog = zeros(nZ,nMPC+1);

        for mpcIter = 1:nMPC
            % update and solve new QP
            % initial value embedding
            zLbFull(1:nX) = x';
            zUbFull(1:nX) = x';

            % solve
            tic;
            [zOpt_quadprog, fval_quadprog, flag, ~, multOpt] = ...
                quadprog( Hfull, g, [], [], Afull, zeros(nI*nX,1), zLbFull, zUbFull, [], QPoptions );
            tLog.quadprog(runIdx,mpcIter) = toc*1e3;
            if (flag ~= 1)
                warning(['quadprog had an error in mpc iteration ', num2str(mpcIter)]);
                runIdx = runIdx - 1;
                break;
            end

            u = zOpt_quadprog(nX+1:nX+nU);

            % log
            zLog_quadprog(:,mpcIter) = [x;u];

            % simulate system
            x = Ci * [x;u];
            x = x + w(:,mpcIter);
        end

        zLog_quadprog(1:nX,mpcIter+1) = x;

        % zOpt = reshape( [zOpt_quadprog; zeros(nU,1)], nZ, nI+1 );

        zOpt = zLog_quadprog;

        display('QUADPROG done.');


        end;



        %% do some visualization
        if strcmp( SOLVER, 'PLOTTING')
        %%

        % lengthPlotSequence = nI+1;
        lengthPlotSequence = nMPC+1;
        t = 1:lengthPlotSequence;

        % first M*3 variables are positions

        for mm = 1:nMasses
            figure; 
            subplot(3,1,1); plot( t, zOpt((mm-1)*3+1,:), 'bx-' );   % x of mass mm
            subplot(3,1,2); hold on;
            plot( t, zOpt((mm-1)*3+2,:), 'bx-' );   % y of mass mm
            plot( t, zLb((mm-1)*3+2)*ones(lengthPlotSequence,1), 'r-');
            hold off;
            subplot(3,1,3); plot( t, zOpt((mm-1)*3+3,:), 'bx-' );   % z of mass mm
        end


        figure;
        subplot(3,1,1); plot( t, zOpt(nMasses*6+1,:), 'bx-' );   % x of mass nMasses+1
        subplot(3,1,2); hold on;
        plot( t, zOpt(nMasses*6+2,:), 'bx-' );   % y of mass nMasses+1
        plot( t, zLb(nMasses*6+2)*ones(lengthPlotSequence,1), 'r-');
        hold off;
        subplot(3,1,3); plot( t, zOpt(nMasses*6+3,:), 'bx-' );   % z of mass nMasses+1


        figure; 
        subplot(3,1,1); plot( t(1:end-1), zOpt(nX+1,1:end-1), 'gx-' );  % u 1
        subplot(3,1,2); plot( t(1:end-1), zOpt(nX+2,1:end-1), 'gx-' );  % u 1
        subplot(3,1,3); plot( t(1:end-1), zOpt(nX+2,1:end-1), 'gx-' );  % u 1

        end  %plotting


    end % solvers
end % benchmark runs


% process times

tLogAvg.qpdunes = mean(tLog.qpdunes,1);
tLogAvg.cplex = mean(tLog.cplex,1);
tLogAvg.quadprog = mean(tLog.quadprog,1);
tLogAvg.forces = mean(tLog.forces,1);



