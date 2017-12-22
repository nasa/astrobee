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

%% generate_cpp_param_reads.m
% Script that generates the functions for reading tunable parameters

%% Inputs
source_header = 'fam_force_allocation_module.h';

% Boolean flags, will also find any variables tun_*_f + the list below
boolean_exceptions = {'tun_ctl_bypass_cmd_shaper', 'tun_cvs_noise_on', ...
                    'tun_env_drag_disturb_on', 'tun_epson_report_truth', ...
                    'tun_debug_ctl_use_truth', 'tun_ase_gravity_removal'};
output_fname = 'fam_tunable_funcs';
function_name = 'fam_ReadParams';
input_object_type = 'RT_MODEL_fam_force_allocation_T*';
input_object_name = 'fam';

write_tunable_param_functions(source_header, boolean_exceptions, output_fname, function_name, input_object_type, input_object_name);
 
