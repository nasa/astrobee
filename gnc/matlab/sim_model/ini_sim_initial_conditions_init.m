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

% Free Flyer Sim Initial Conditions Init.  Configures the intial conditions
% for a simulation

%% Time
% Time is maintained as unix time, which is maintained as the number of seconds and nanoseconds since  00:00:00 UTC on 1 January 1970.
% Matlab epoch datenum(1970, 1,1, 0,0,0)
ini_time_seconds        = uint32(1.4213e+09);
ini_time_nanoseconds    = uint32(0);

%% Moved to lua conig file [ASTROBEE_ROOT '/../../management/astrobee/config/gnc.config']

% init_omega_B_ISS_B      = single([0, 0, 0]);            % Initial body rates relative to ISS
% init_Q_ISS2B            = single([0, 0, 0, 1]);         % Initial body quaternion, ISS to Body CF
% 
% ini_P_B_ISS_ISS        = single([0, 0, 0]);            %[m] Initial body velocity
% ini_V_B_ISS_ISS        = single([0, 0, 0]);            %[m/s] Initial body posistion
