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

% Generate .csv for testing with fsw

genCSVfromTM('out_act_msg_vpp', out_act_msg_vpp)
genCSVfromTM('out_act_msg', out_act_msg)
genCSVfromTM('out_kfl_msg', out_kfl_msg)
genCSVfromTM('out_cmd_msg', out_cmd_msg)
genCSVfromTM('out_ex_time_msg', out_ex_time_msg)
genCSVfromTM('out_cvs_reg_pulse', out_cvs_reg_pulse)
genCSVfromTM('out_cvs_landmark_msg', out_cvs_landmark_msg)
genCSVfromTM('out_cvs_optflow_msg', out_cvs_optflow_msg)
genCSVfromTM_structs('out_cmc_msg', out_cmc_msg)
genCSVfromTM('out_imu_msg', out_imu_msg)
genCSVfromTM('out_env_msg', out_env_msg)
genCSVfromTM('out_vpp_msg', out_vpp_msg)
genCSVfromTM('out_bpm_msg', out_bpm_msg)
