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

% Astrobee master prep file.  Configures parameters used in the astrobee simulink model.
% Dependencies: 
%       astrobee_init.m

%% Load Configuration Parameters
cset                                = astrobee_configuration;           % Calls mfile that sets Simulink model configuration parameters
astrobee_config_set_ref             = Simulink.ConfigSetRef;            % Create configuration reference
astrobee_config_set_ref.WSVarName   = 'cset';
astrobee_config_set_ref.Name        = 'astrobee_config_set_ref';

open_sys                            = find_system('SearchDepth', 0);    % Collect the open simulink diagrams

% Apply the configuration set to the astrobee model
%end

% Note: save changes to the config set, DO NOT DO THIS FROM A COMPUTER WITHOUT SIMULINK CODER/ERT
% cset.saveAs('astrobee_configuration.m')

%% general
% abp_astrobee_physical_properties_prep; %removed since no parameters
if tun_debug_ctl_use_truth
    warning('Simulation is using truth data (with no noise injected)')
end

%% Sim prep
bpm_blower_propulsion_module_prep;
epson_imu_model_prep;
env_environment_model_prep;
cvs_computer_vision_system_prep;
mlp_mid_level_processor_prep;

%% FSW prep
% ctl_controller_prep;              %All values moved to lua config file
fam_force_allocation_module_prep;
%ase_augmented_state_estimator_prep;
