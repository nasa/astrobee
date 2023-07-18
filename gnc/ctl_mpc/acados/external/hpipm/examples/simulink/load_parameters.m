close all 
clc

% check that env.sh has been run
env_run = getenv('ENV_RUN');
if (~strcmp(env_run, 'true'))
	disp('ERROR: env.sh has not been sourced! Before executing this example, run:');
	disp('source env.sh');
	return;
end




% create dim from qp data
dim = hpipm_ocp_qp_dim('qp_data.c');
dim.print_C_struct();

% extract dims
N = dim.get('N');
NU = dim.get('nu', 0);
NX = dim.get('nx', 1);

% initial state for simulation
% TODO getter for x0 in qp data !!!!!!!!!
x0 = [-1 3];

% sampling time
Ts = 1; % by default one simulation step per second

