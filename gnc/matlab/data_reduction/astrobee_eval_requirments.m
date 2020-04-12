% Eval Astrobee GN&C requirments on the following data sets
%
%
%% Data sets
dir_names = {'/Users/jfusco1/Desktop/2018-03-27-06-55-06_scn2', ...
    '/Users/jfusco1/Desktop/2018-04-04-07-16-02', '/Users/jfusco1/Desktop/2018-04-10-10-24-03'};



%% Process the data
curr_dir = pwd;
for dir_indx=1:length(dir_names)
    cd(dir_names{dir_indx});
    
    astrobee_load_data;
    fprintf('Loading data from: %s\n', pwd);

    % Determine which data sets exist in the workspace
    ds_on.cmd = exist('out_cmd_msg', 'var')  && ~isempty(out_cmd_msg) && isfield(out_cmd_msg, 'traj_quat');
    ds_on.env = exist('out_env_msg', 'var') && ~isempty(out_env_msg);
    ds_on.kfl = exist('out_kfl_msg', 'var') && ~isempty(out_kfl_msg);
    ds_on.act = exist('out_act_msg', 'var') && ~isempty(out_act_msg);
    
    % Run any init files we need variables from
    tunable_init;
    
    %% Derive Data
    calcData(dir_indx) = deriveAstroBeeData;
    t0 = calcData.t0;
end
cd(curr_dir);

