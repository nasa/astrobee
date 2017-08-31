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

%mlp_mid_level_processor_prep.m

%add the initial sim time to the command_times
mlp_command_times = [mlp_command_times(:,1)+ini_time_seconds, mlp_command_times(:,2)+ini_time_nanoseconds];


mlp_num_commands    = single(size(mlp_command_list,1));
mlp_num_mode_cmds   = single(size(mlp_mode_cmd_list,1));