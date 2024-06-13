% run script for solving oscillator example from Wang2010 with qpDUNES,
% quadprog, CPLEX, and FORCES


function [ tLog ] = runOscillator( )
	clc
    
    enablePlotting = false;  % true, false
    
    qpDUNES_PATH = '../../../interfaces/matlab';
    cplex_PATH = '~/Applications/IBM/ILOG/CPLEX_Studio1251/cplex/matlab';
    forces_PATH = './FORCES';
    
    
    nTestRuns = 1000;
%     nTestRuns = 1;
    
    
 	nsteps = 100;
	%nsteps = 5;

	%  generate_oscillator;
	setupOscillator;


	[nX,nU] = size(B);
    nZ = nX+nU;
	nPH = 30;


	x0 = 1.0*ones(nX,1);
    % x0 = x0 * 0;
    

	fprintf('Number of states: %d\n', nX);
	fprintf('Number of controls: %d\n', nU);
	fprintf('Number of vars: %d\n', nX * (nPH+1) + nU * nPH);

    
    tLog.qpdunes = zeros(nTestRuns,1);
    tLog.quadprog = zeros(nTestRuns,1);
    tLog.cplex = zeros(nTestRuns,1);
    tLog.forces = zeros(nTestRuns,1);
       
    %%
    for runIter = 1:nTestRuns
    
        % process noise trajectory
        w = 2*rand(nX,nsteps)-1; 
        w(1:nX/2,:) = 0; 
        w = 0.1*w;
        % w = w*0;
        

        for SOLVER = {'QPDUNES', 'QUADPROG', 'CPLEX', 'FORCES'}
    %     for SOLVER = {'QPDUNES', 'CPLEX'} %, 'FORCES'}
%         for SOLVER = {'QPDUNES', 'FORCES'}
%         for SOLVER = {'QPDUNES'}
%         for SOLVER = {'FORCES'}

            if strcmp( SOLVER, 'QPDUNES')

                % bring data in z-style
                Hi = blkdiag(Q, R);
                H = repmat( Hi, 1, nPH );
                P = Q;
                Ci = [A B];
                C = repmat( Ci, 1, nPH );
                c = zeros(nX, 1);
                cFull = repmat( c, nPH, 1 );
                ziLow = [ xmin(1:nX); umin(1:nU) ];
                ziUpp = [ xmax(1:nX); umax(1:nU) ];
                zRef = zeros( nX*(nPH+1)+nU*nPH,1 );
                zLow = [ repmat(ziLow, nPH, 1); xmin ];
                zUpp = [ repmat(ziUpp, nPH, 1); xmax ];
                
                
                % set up qpDUNES

                addpath(qpDUNES_PATH);

                qpOptions = qpDUNES_options( 'default', ...
                                             'maxIter', 20, ...
                                             'printLevel', 0, ...
                                             'logLevel', 0, ...     
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
                mpcDUNES( 'init', nPH, ...
                          H, P, [], C, cFull, ...
                          zLow, zUpp, zRef, ...
                          qpOptions );

            elseif ( strcmp( SOLVER, 'QUADPROG' ) || strcmp( SOLVER, 'CPLEX' ) )

                Hfull = [];
                for i=1:nPH
                    Hfull = blkdiag( Hfull, Q, R );
                end
                Hfull = blkdiag( Hfull, Q );
                

                C = [A B];
                E = eye(nX);
                Afull = zeros( nX*nPH, nZ*nPH+nX );
                for i=1:nPH
                    Afull( (i-1)*nX+1:i*nX, (i-1)*nZ+1:i*nZ+nX ) = [C -E];
                end
                cfull = repmat(c, nPH,1);

                ziLow = [ xmin(1:nX); umin(1:nU) ];
                ziUpp = [ xmax(1:nX); umax(1:nU) ];
                zLow = [ repmat(ziLow, nPH, 1); xmin ];
                zUpp = [ repmat(ziUpp, nPH, 1); xmax ];

                if strcmp( SOLVER, 'QUADPROG' ) 
        %             quadprogOptions = optimset('LargeScale','on','Display','off', 'Algorithm', 'active-set');
                    quadprogOptions = optimset('LargeScale','on','Display','off', 'Algorithm', 'interior-point-convex');
                elseif strcmp( SOLVER, 'CPLEX' )

                    addpath(cplex_PATH);

    %                 cplexOptions = cplexoptimset('cplex');
    %                 cplexOptions = cplexoptimset('Diagnostics', 'on');
                    cplexOptions = cplexoptimset();
    %                 cplexOptions.qpmethod = 0;  % 0 = auto, 1 = primal AS, 2 = dual AS, 4 = IP
    %                 cplexOptions.barrier.display = 0;
    %                 cplexOptions.threads = 0;   
    %                 cplexOptions.diagnostics = 'off';
                end

            elseif strcmp( SOLVER, 'FORCES' )
                addpath(forces_PATH);

                % FORCES multistage form - zi = [xi, ui] for i=1...N-1 and zN = xN
                

                stages = MultistageProblem(nPH+1);

                for i = 1:(nPH+1)
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
                        stages(i).ineq.b.lb = [xmin; umin]; % lower bound for this stage variable

                        % upper bounds
                        stages(i).ineq.b.ubidx = 1:stages(i).dims.n; % upper bound acts on these indices
                        stages(i).ineq.b.ub = [xmax; umax]; % upper bound for this stage variable

                        % equality constraints
                        stages(i).eq.C = [eye(nX), zeros(nX,nU); A, B];
                        params(1) = newParam('z1',1,'eq.c'); % RHS of first eq. constr. is a parameter: [x0, 0]

                    end

                    % stages along horizon
                    if( i>1 && i<=nPH )       

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
                        stages(i).ineq.b.lb = [xmin(1:nX); umin(1:nU)]; % lower bound for this stage variable

                        % upper bounds
                        stages(i).ineq.b.ubidx = 1:stages(i).dims.n; % upper bound acts on these indices
                        stages(i).ineq.b.ub = [xmax(1:nX); umax(1:nU)]; % upper bound for this stage variable

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
                    if( i==nPH+1 )

                        % dimension
                        stages(i).dims.n = nX;    % number of stage variables
                        stages(i).dims.r = 0;     % number of equality constraints        
                        stages(i).dims.l = nX;    % number of lower bounds
                        stages(i).dims.u = nX;    % number of upper bounds
                        stages(i).dims.p = 0;     % number of polytopic constraints
                        stages(i).dims.q = 0;     % number of quadratic constraints

                        % cost
                        stages(i).cost.H = Q;
                        stages(i).cost.f = zeros(stages(i).dims.n,1);

                        % lower bounds
                        stages(i).ineq.b.lbidx = 1:stages(i).dims.n; % lower bound acts on these indices
                        stages(i).ineq.b.lb = xmin; % lower bound for this stage variable

                        % upper bounds
                        stages(i).ineq.b.ubidx = 1:stages(i).dims.n; % upper bound acts on these indices
                        stages(i).ineq.b.ub = xmax; % upper bound for this stage variable

                        % equality constraints        
                        stages(i).eq.D = -eye(nX);

                    end
                end

                %% solver settings
                codeoptions = getOptions('forcesMPC');

                % printlevel
                codeoptions.printlevel = 0;
                codeoptions.parallel = 0;

                %% generate code
                %generateCode(stages,params,codeoptions);

            end


            %%

            if (enablePlotting)
                figure;
            end

            % allocate history matrices
            Xhist = zeros(nX,nsteps);  % state
            Uhist = zeros(nU,nsteps);  % input
            thist = zeros(1,nsteps);  % fmpc run time

            % set initial state
            x = x0;

            for i = 1:nsteps

                % disp(strcat(['--- ', num2str(i), ' ---']));

                % run QP solver
                if strcmp( SOLVER, 'QPDUNES')
                    tic;
                    [uOpt, xOpt, stat, objFctnVal] = mpcDUNES( 'solve', x );
                    thist(i) = toc;

                    if (enablePlotting)
                        tvec = 0:ts:(nPH-1)*ts;
                        for jj=1:3
                            subplot(3,1,jj); hold on;
                            stairs(tvec+i*ts,uOpt(jj:3:end), 'r');
                            axis([0,70,1.1*umin(jj),1.1*umax(jj)]); ylabel(['u_',num2str(jj)]);
                        end
                        xlabel('t (qpd)');
                    end

                elseif strcmp( SOLVER, 'QUADPROG')
                    % initial value
                    zLow( 1:nX ) = x;
                    zUpp( 1:nX ) = x;
                    tic;
                    [zOpt_quadprog, fval_quadprog, ~, ~, multOpt] = ...
                        quadprog( Hfull, [], [], [], Afull, cfull, zLow, zUpp, [], quadprogOptions );
                    thist(i) = toc;
                    uOpt = zOpt_quadprog(nX+1:nZ);
                elseif strcmp( SOLVER, 'CPLEX')
                    tic;
                    [zOpt_cplex,fval_cplex,exitflag,output,multOpt] = ...
                        cplexqp( Hfull, zeros(length(Hfull),1), [], [], Afull, cfull, zLow, zUpp, [], cplexOptions);
                    thist(i) = toc;
                    uOpt = zOpt_cplex(nX+1:nZ);
                elseif strcmp( SOLVER, 'FORCES')
                    problem.z1 = zeros(2*nX,1);
                    problem.z1(1:nX) = x;
                    tic;
                    [solverout,exitflag,info] = forcesMPC(problem);
                    thist(i) = toc;

                    if (enablePlotting)
                        % get solver output
                        uOpt = [[]];
                        for k = 1:9
                            a = solverout.(['y', num2str(0), num2str(k)]);
                            uOpt = [uOpt; a(nX+1:nX+nU)];
                        end
                        for k = 10:nPH
                            a = solverout.(['y', num2str(k)]);
                            uOpt = [uOpt; a(nX+1:nX+nU)];
                        end

                        tvec = 0:ts:(nPH-1)*ts;
                        for jj=1:3
                            subplot(3,1,jj); hold on;
                            stairs(tvec+i*ts,uOpt(jj:3:end), 'r');
                            axis([0,70,1.1*umin(jj),1.1*umax(jj)]); ylabel(['u_',num2str(jj)]);
                        end
                        xlabel('t (forces)');
                    else 
                        uOpt = solverout.y01(nX+1:nX+nU);
                    end
                end

                % record state, input, stage cost, and fmpc run time
                u = uOpt(1:nU);
                Xhist(:,i) = x; Uhist(:,i) = u;


                % state update
                x = A*x+B*u+w(:,i);

                % shift previous state and input trajectories 
                % for warm start in next step
                %X = [X(:,2:T),zeros(n,1)];
                %U = [U(:,2:T),zeros(m,1)];
            end

            if strcmp( SOLVER, 'QPDUNES')
                % -- CLEANUP --
                mpcDUNES( 'cleanup' );

                Xhist_qpdunes = Xhist;
                Uhist_qpdunes = Uhist;
                str = sprintf('qpDUNES: The average time is: %f ms',mean(thist)*1e3);
                
                tLog.qpdunes(runIter) = mean(thist)*1e3;
            elseif strcmp( SOLVER, 'QUADPROG')
                Xhist_quadprog = Xhist;
                Uhist_quadprog = Uhist;
                str = sprintf('quadprog: The average time is: %f ms',mean(thist)*1e3);
                
                tLog.quadprog(runIter) = mean(thist)*1e3;
            elseif strcmp( SOLVER, 'CPLEX')
                Xhist_cplex = Xhist;
                Uhist_cplex = Uhist;
                str = sprintf('CPLEX: The average time is: %f ms',mean(thist)*1e3);
                
                tLog.cplex(runIter) = mean(thist)*1e3;
            elseif strcmp( SOLVER, 'FORCES')
                Xhist_forces = Xhist;
                Uhist_forces = Uhist;
                str = sprintf('FORCES: The average time is: %f ms',mean(thist)*1e3);
                
                tLog.forces(runIter) = mean(thist)*1e3;
            end

            % print average iteration time 
            %disp(str);

        end % all solvers
        
        fprintf('+ Done with testrun %4d +\n', runIter);
    
    end % nTestRuns
    
    
    %display average run times
    
    fprintf('\nAverage runtimes over %d randomly generated instances:\n', nTestRuns);
    fprintf('quadprog: %.2f ms\n', mean(tLog.quadprog));
    fprintf('CPLEX:    %.2f ms\n', mean(tLog.cplex));
    fprintf('FORCES:   %.2f ms\n', mean(tLog.forces));
    fprintf('qpDUNES:  %.2f ms\n', mean(tLog.qpdunes));


	%% plot trajectories for x, u
    if (enablePlotting)
        tvec = 0:ts:(nsteps-1)*ts; figure;
        for i=1:6
            subplot(6,1,i); 
            stairs(tvec,Xhist_qpdunes(i,1:nsteps), 'r');
            stairs(tvec,Xhist_quadprog(i,1:nsteps), 'b');
            stairs(tvec,Xhist_cplex(i,1:nsteps), 'g');
            stairs(tvec,Xhist_forces(i,1:nsteps), 'k');
            axis([0,50,1.1*min(Xhist(i,1:nsteps)),1.1*max(Xhist(i,1:nsteps))]); ylabel(['x_',num2str(i)]);
        end
        xlabel('t');


        tvec = 0:ts:(nsteps-1)*ts; figure;
        for i=1:3
            subplot(3,1,i); 
            stairs(tvec,Uhist_qpdunes(i,1:nsteps), 'r');
            stairs(tvec,Uhist_quadprog(i,1:nsteps), 'b');
            stairs(tvec,Uhist_cplex(i,1:nsteps), 'g');
            stairs(tvec,Uhist_forces(i,1:nsteps), 'k');
            axis([0,50,1.1*min(Uhist(i,1:nsteps)),1.1*max(Uhist(i,1:nsteps))]); ylabel(['u_',num2str(i)]);
        end
        xlabel('t');
    end
    
    
    %% plot histogram like comparison
    
    fontsize = 14;
    linewidth = 2;
    
%     tVec = [0 exp([-1:0.02:2] *log(10) )];
     tVec = [0 exp([log(1e-1):0.05:log(1e2)] )];
    tBin.qpdunes = histc(tLog.qpdunes,tVec);
    tBin.quadprog = histc(tLog.quadprog,tVec);
    tBin.cplex = histc(tLog.cplex,tVec);
    tBin.forces = histc(tLog.forces,tVec);
    
    figure; hold on;
    set(gca,'XScale','log');
    ylabel('# instances','fontsize',fontsize);
    xlabel('computation time [s]','fontsize',fontsize);
    
    stairs (tVec, tBin.qpdunes, 'r','linewidth',linewidth);
    stairs (tVec, tBin.forces, 'k','linewidth',linewidth);
    stairs (tVec, tBin.cplex, 'g','linewidth',linewidth);
    stairs (tVec, tBin.quadprog, 'b','linewidth',linewidth);
    
    
    legend('qpDUNES','FORCES','CPLEX','quadprog');
    

end






