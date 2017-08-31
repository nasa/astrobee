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

% Force allocation module prep file

%% Nozzles

%calculate the inverse matrix for thrust to body torques and forces (i.e. force/torques -> nozzle thrust matrix)
fam_P_nozzle_B_B                        = [abp_PM1_P_nozzle_B_B; abp_PM2_P_nozzle_B_B];                 %[m] Combined PM1&2 nozzle positions
fam_nozzle_orientations                 = [abp_PM1_nozzle_orientations; abp_PM2_nozzle_orientations];   %[m] Combined PM1&2 nozzle orientations

fam_nozzle_angle2pwm                    = (abp_servo_max_PWM-abp_servo_min_PWM)/(abp_nozzle_max_open_angle-abp_nozzle_min_open_angle);  %[-] Conversion between servo angle (not nozzle angle) command and resulting servo PWM

%% Impeller: 
fam_impeller_speeds_cnt                 = length(fam_impeller_speeds);

%find total thrust (normalized by discharge coefficient) as a function of delta P for each impeller speed
%define and calculate equation from data
fam_impeller_eq_CQ_input                = 0:fam_impeller_eq_density:abp_impeller_eq_max_CQ;             %[m^3/sec] Range of Flow rate coefficients (CQ) to use in equation
fam_impeller_eq_CP_output               = polyval(abp_impeller_eq_CQ2CP_poly,fam_impeller_eq_CQ_input);         %[-] CP = f(CQ) 

%Calculate thrust from Cq and Cp: T/N^2 = (Cq*D^3*h*rho*sqrt(2*Cp)) - (2*A0*rho*D^2*Cp) [then (2*A0*rho*D^2*Cp) accounts for the leakage area,  Need to do this here since thrust changes with pressure]
    %note: that by removing the "zero thrust area" the thrust lookup table will have negative values, this is expected.  We won't actually have negative thrust.
fam_PM1_impeller_eq_thrust_output       = (fam_impeller_eq_CQ_input*(abp_impeller_diameter^3)*abp_impeller_height*const_air_density.*sqrt(2*fam_impeller_eq_CP_output)) - (2*abp_pm1_zero_thrust_area*const_air_density*(abp_impeller_diameter^2)*fam_impeller_eq_CP_output);
fam_PM2_impeller_eq_thrust_output       = (fam_impeller_eq_CQ_input*(abp_impeller_diameter^3)*abp_impeller_height*const_air_density.*sqrt(2*fam_impeller_eq_CP_output)) - (2*abp_pm1_zero_thrust_area*const_air_density*(abp_impeller_diameter^2)*fam_impeller_eq_CP_output);

%calculate max thrust.
[~, fam_PM1_impeller_eq_thrust_max_idx] = max(fam_PM1_impeller_eq_thrust_output);
[~, fam_PM2_impeller_eq_thrust_max_idx] = max(fam_PM2_impeller_eq_thrust_output);

%create lookup tables with valid values (from 0-max thrust)  We use this section of data as it cooresponds to on pulsing, vs off pulsing
fam_PM1_lookup_thrust_breakpoints       = fam_PM1_impeller_eq_thrust_output(1:fam_PM1_impeller_eq_thrust_max_idx);
fam_PM2_lookup_thrust_breakpoints       = fam_PM2_impeller_eq_thrust_output(1:fam_PM2_impeller_eq_thrust_max_idx);
fam_PM1_lookup_Cdp_data                 = fam_impeller_eq_CP_output(1:fam_PM1_impeller_eq_thrust_max_idx);
fam_PM2_lookup_Cdp_data                 = fam_impeller_eq_CP_output(1:fam_PM2_impeller_eq_thrust_max_idx);
