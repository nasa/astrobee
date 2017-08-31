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

% Force allocation module initialization file.  Configures
% parameters used in the library file fam_force_allocation_module.

%% Nozzle Parameters
fam_nozzle_x_dir            = [1 2 7 8];                                    %[-]        Nozzles cooresponding to the null vector about each axis
fam_nozzle_y_dir            = [3 4 9 10];
fam_nozzle_z_dir            = [5 6 11 12];

fam_nozzle_pm1              = 1:6;                                          %[-]        Nozzles cooresponding to each PM
fam_nozzle_pm2              = 7:12;
                                  
%% Fan Impeller 
fam_impeller_eq_density     = single(0.001);                                %[-]        Density of points in the lookup table equation
fam_impeller_speeds         = single([2000; 2500; 2800])*units_RPM_2_rps;   %[rad/s]    Basic set of possible impeller speeds
