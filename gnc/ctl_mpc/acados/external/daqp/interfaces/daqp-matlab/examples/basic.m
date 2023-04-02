%% Setup QP data
H = [1.0 0; 0 1];
f = [2.0;2];
A = [1.0 0 ; 0 1];
bupper = [1.0;1];
blower = [-1.0,-1];
sense = int32([0;0]);
%% Solve QP (quadprog interface)
[xstar,fval,exitflag,info] = daqp.quadprog(H,f,A,bupper,blower,sense);
%% Solve QP class interface 
d = daqp();
d.setup(H,f,A,bupper,blower,sense);
[xstar,fval,exitflag,info] = d.solve();
