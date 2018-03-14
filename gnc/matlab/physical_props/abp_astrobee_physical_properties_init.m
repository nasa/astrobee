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

% Physical Properties: initalizes the physical parameters of the astrobee FSW and model

%% units and constants
units_in_2_m                    = single(0.0254);
units_RPM_2_rps                 = single(2*pi/60);
units_deg_2_rad                 = single(pi/180);
units_rad_2_deg                 = single(180/pi);
const_air_density               = single(1.225);                                %[kg/m^3]   Air density inside of the ISS

%% Vehicle Properties
% gnc.config now holds many properties: edit([ASTROBEE_ROOT '../../management/astrobee/config/gnc.config'])
% mass properties (mass, cg, MOI) moved to gnc.config
% sensor properties (location, orientation) moved to gnc.config

abp_P_CG_B_B_error              = [0 0 0];                                      %[m]        Position Error of the CG estimate

abp_temp                        = single(300);                                  %[K]        Vehicle temperature

%% Blower Properties
abp_pm1_impeller_orientation    = single([0, 1,0]);                             %[unit vec] PM1 Axis of rotation of the impeller.
abp_pm2_impeller_orientation    = single([0,-1,0]);                             %[unit vec] PM2 Axis of rotation of the impeller.

abp_impeller_diameter           = single(5.5)*units_in_2_m;                     %[m]        Outer diameter of impeller
abp_impeller_height             = single(1.6742)*units_in_2_m;                  %[m]        Trailing Edge height of impeller
abp_impeller_eq_CQ2CP_poly      = [58.72 -45.013 11.782 -2.1758 0.1278 0.0825]; %[-]        Coefficients from Blair Data 2016-03-13 giving Cpth = f(CQh)
abp_impeller_eq_max_CQ          = single(0.33319);                            	%[-]        Max CQ from Blair Data. If impeller equation data changes, this value needs to be re-verified

abp_pm1_zero_thrust_area        = single([0.0044667]);                          %[m^2]      PM1: If air is leaking from the plenum, the '0 thrust' position (all nozzles closed) will have an effective open area
abp_pm2_zero_thrust_area        = single([0.0042273]);                          %[m^2]      PM2: If air is leaking from the plenum, the '0 thrust' position (all nozzles closed) will have an effective open area
%determined from test data - nozzles closed @[2000 2500]rpm => deltaP = [X Y]PA => Cdp = [X Y] => Total leak area = [X Y]

abp_impeller_speed2pwm          = single(.792095);                              %[CNT/(rad/sec)] Converts impeller speeds into PWM values

%% Servo Parameters
abp_servo_max_PWM               = single(255);                                  %[PWM]      Maximum Servo command to avoid hardstops
abp_servo_min_PWM               = single(0);                                   %[PWM]      Minimum Servo command to avoid hardstops

%% Nozzle Properties
abp_nozzle_count                = single(6);                                    %[-]        Number of Nozzles per PM
abp_nozzle_flap_length          = single(0.5353)*units_in_2_m;                  %[m]        Length of the nozzle flappers
abp_nozzle_intake_height        = single(0.5154)*units_in_2_m;                  %[m]        Height of the intake to the nozle
abp_nozzle_out_height           = single(0.4216)*units_in_2_m;                  %[m]        Height of the output of the nozzle
abp_nozzle_flap_count           = single(2);                                    %[-]        Number of flaps per nozzle
abp_nozzle_gear_ratio           = single(1/2);                                  %[-]        Gear Ratio between servo and nozzle.  theta_s = R*theta_n
abp_nozzle_max_open_angle     = single(79.91)*units_deg_2_rad;                %[rad]      THEORETICAL max angle that a nozzle can open to
% abp_nozzle_max_open_angle       = single(73.90)*units_deg_2_rad;                %[rad]      Measured max angle that a nozzle can open to on P4E
abp_nozzle_min_open_angle       = single(15.68)*units_deg_2_rad;                %[rad]      Min angle that the nozzle can close to



abp_PM1_nozzle_names            = ['Rx+'; ...                                   %[-]        PM1: String names for each nozzles.  Reference Only.  
                                   'Rx-'; ...
                                   'Fy+'; ...
                                   'Ay+'; ...
                                   'Rz+'; ...
                                   'Rz-'];
abp_PM2_nozzle_names            = ['Lx-'; ...                                   %[-]        PM2: String names for each nozzles.  Reference Only. 
                                   'Lx+'; ...
                                   'Ay-'; ...
                                   'Fy-'; ...
                                   'Lz+'; ...
                                   'Lz-'];
                           
abp_PM1_nozzle_widths           = single([ 5.0; ...                             %[m]        PM1: Width of each nozzle
                                           5.0; ...
                                           2.8; ...
                                           2.8; ...
                                           2.8; ...
                                           2.8])*units_in_2_m;
abp_PM2_nozzle_widths           = abp_PM1_nozzle_widths;                        %[m]        PM2: Width of each nozzle      

abp_PM1_P_nozzle_B_B            = single([ 6.00,  4.01, -1.56;...               %[m]        PM1: Position vector of the nozzle locations in the body frame
                                          -6.00,  4.01,  1.56;...
                                          2.83,  6.00,  2.83;...
                                          -2.83,  6.00, -2.83;...
                                          -2.66,  4.01,  6.00;...
                                          2.66,  4.01, -6.00])*units_in_2_m;
                                      
abp_PM2_P_nozzle_B_B            = single([-6.00, -4.01, -1.56;...               %[m]        PM2: Position vector of the nozzle locations in the body frame
                                           6.00, -4.01,  1.56;...
                                          -2.83, -6.00,  2.83;...
                                           2.83, -6.00, -2.83;...
                                           2.66, -4.01,  6.00;...
                                          -2.66, -4.01, -6.00])*units_in_2_m;
     
abp_PM1_nozzle_orientations     = single([ 1, 0, 0; ...                         %[unit vec] PM1: Pointing Direction of each nozzle
                                          -1, 0, 0; ...
                                          0, 1, 0; ...
                                          0, 1, 0; ...
                                          0, 0, 1; ...
                                          0, 0,-1]);
                                   
abp_PM2_nozzle_orientations     = single([-1, 0, 0; ...                         %[unit vec] PM2: Pointing Direction of each nozzle
                                           1, 0, 0; ...
                                           0,-1, 0; ...
                                           0,-1, 0; ...
                                           0, 0, 1; ...
                                           0, 0,-1]);

abp_PM1_discharge_coeff         = single([ ...                                  %[-]        PM1: Nozzle Discharge coefficient.  Determined via Test
                                          0.914971062	; ... 
                                          0.755778254	; ...
                                          0.940762925	; ...
                                          0.792109779	; ...
                                          0.92401881	; ...
                                          0.930319765	]);
abp_PM2_discharge_coeff         = single([ ...                                  %[-]        PM2: Nozzle Discharge coefficient.  Determined via Test
                                          0.947114008	; ...
                                          0.764468916	; ...
                                          1.000000000	; ...
                                          0.90480943	; ...
                                          0.936555627	; ...
                                          0.893794766	]);
