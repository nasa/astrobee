% Copyright (c) 2017, United States Government, as represented by the
% Administrator of the National Aeronautics and Space Administration.
%
% All rights reserved.
%
% The Astrobee platform is licensed under the Apache License, Version 2.0
% (the "License"); you may not use this file except in compliance with the
% License. You may obtain a copy of the License at
%
%     http://www.apache.org/licenses/LICENSE-2.0
%
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
% WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
% License for the specific language governing permissions and limitations
% under the License.
%
% Autocode the freeflyer simulink models
%
%%
run([fileparts(mfilename('fullpath')) '/../../' 'astrobee_set_path'])
% List of subsystems to build
models = {'astrobee/fsw_lib/est_estimator',...
    'astrobee/fsw_lib/ctl_controller', ...
    'astrobee/fsw_lib/fam_force_allocation_module',...
    'astrobee/sim_model_lib',...
    'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module', ...
    'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_2_propulsion_module'};

% Open the freeflyer model
disp('...Openning Model');
open('astrobee')

% Setup parameters for Proto 4
disp('...Initializing Parameters');
astrobee_init;
proto4_init;
astrobee_prep;

% save the current working directory
starting_dir = pwd;

cd(fullfile(ASTROBEE_ROOT, 'code_generation'));
try
    disp('...Removing old generated code');
    if ~isempty(ls('*_rtw'))
        rmdir('*_rtw', 's'); % Remove the old generated code
    end
catch exception
end
disp('...Building Code');
for ii = 1:length(models)
    disp(['%%%%%%%%%%%% Building ' models{ii} '%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%']);
    rtwbuild(models{ii})
    
end
copyfile(fullfile('slprj', 'ert', '_sharedutils'), 'sharedutils')

%% Generate tunable functions
dirs = ls;
ctl_dir = regexp(dirs, 'ctl_\w+_rtw', 'match');
est_dir = regexp(dirs, 'est_\w+_rtw', 'match');
fam_dir = regexp(dirs, 'fam_\w+_rtw', 'match');
sim_dir = regexp(dirs, 'sim_\w+_rtw', 'match');

if ~isempty(ctl_dir)
    cd(ctl_dir{1});
    generate_ctl_param_reads;
    cd ..
end

if ~isempty(est_dir)
    cd(est_dir{1});
    generate_est_param_reads;
    cd ..
end

if ~isempty(fam_dir)
    cd(fam_dir{1});
    generate_fam_param_reads;
    cd ..
end

if ~isempty(sim_dir)
    cd(sim_dir{1});
    generate_sim_param_reads;
    cd ..
end

%% Return to the beginning
cd(starting_dir)