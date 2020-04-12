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

% Control sim initialization file
% Configures parameters used in both FSW and Sim simulink models

ab_verbose                      = true;  %when true, displays output messages
% ab_verbose                      = false;

%% Configuration
if(ab_verbose) 
    disp('Starting Astrobee...');
end
astrobee_set_path;

astrobee_stop_time              = 5*60;
astrobee_time_step_size         = 0.016; % Simulator sample time, .016
astrobee_fsw_step_size          = 0.016; %Used for 1/z functions

%% init scripts
if(ab_verbose) 
    disp('...loading init parameters');
end


% physical properties
abp_astrobee_physical_properties_init;

% Estimator init files
ase_augmented_state_estimator_init;
ctl_controller_init;
fam_force_allocation_module_init;

% Sim init files
ini_sim_initial_conditions_init;
bpm_blower_propulsion_module_init;
epson_imu_model_init;
env_environment_model_init;
cvs_computer_vision_system_init;
mlp_mid_level_processor_init;

%% Busses 
% Needs to be after the init files as data types are defined in the data
% files
if(ab_verbose) 
    disp('...loading bus definitions');
end
bus_defs;
bus_init;

%% FSW init files, blows away any overlapping values
astrobee_version = 'flight';

if(ab_verbose) 
    disp('...loading tunable parameters');
end
tunable_init;

%%
if(ab_verbose) 
    disp('Astrobee Init Complete');
end