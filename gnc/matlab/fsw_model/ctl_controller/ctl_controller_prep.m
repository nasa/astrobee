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

% Astrobee controller (CTL) initialization file.  Configures
% parameters used in the library file ctl_controller.
%
%% Moved to lua conig file [ASTROBEE_ROOT '/../../management/astrobee/config/gnc.config']
% tun_ctl_pos_Tp    = single(ctl_pos_kp./tun_ctl_vel_kd);      % Actual proportinal gain in Kp/Kd
% tun_ctl_pos_Ti    = single(ctl_pos_ki./tun_ctl_vel_kd);      % Actual integral gain in Ki/Kd
% ctl_att_Tp        = single(ctl_att_kp./tun_ctl_omega_kd);    % Actual proportinal gain in Kp/Kd
% ctl_att_Ti        = single(ctl_att_ki./tun_ctl_omega_kd);    % Actual integral gain in Ki/Kd

if tun_tun_debug_ctl_use_truth
    warning('Controller using ground truth instead of KFL estimate')
end