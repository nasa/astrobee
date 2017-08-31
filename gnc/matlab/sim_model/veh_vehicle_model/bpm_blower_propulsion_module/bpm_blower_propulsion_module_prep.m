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

%bpm_blower_propulsion_module_prep.m
 
%% Noise warning
if (tun_bpm_noise_on_flag == 0)
    warning('Propulsion Noise Off')
end
    
%% motor parameters
bpm_imp_motor_friction_coeff            = bpm_imp_noload_curr*bpm_imp_motor_torque_k/bpm_imp_noload_speed ;  %[Nm*s] Viscous Friction

%% servo parameters
bpm_servo_max_theta                     = abp_nozzle_gear_ratio*(abp_nozzle_max_open_angle - abp_nozzle_min_open_angle);    
bpm_servo_min_theta                     = 0; 
bpm_servo_pwm2angle                     = (bpm_servo_max_theta-bpm_servo_min_theta)/(abp_servo_max_PWM-abp_servo_min_PWM);                  %[-] Conversion between servo PWM command (0-100) and resulting angle (0-90)
bpm_servo_pwm2angle_bias                = bpm_servo_min_theta/bpm_servo_pwm2angle - abp_servo_min_PWM;                                      %[rad] Bias on the PWM->angle conversion
bpm_servo_motor_r                       = bpm_servo_max_voltage/bpm_servo_peak_curr;                                                        %[ohm] Servo internal resistance
bpm_servo_motor_k                       = bpm_servo_peak_torque*bpm_servo_motor_gear_ratio/(bpm_servo_peak_curr);                           %[Nm/A] Servo motor torque constant

%% aerodynamics parameters
%setup input values based on density/max
bpm_impeller_eq_CQ_input                = 0:bpm_impeller_eq_density:abp_impeller_eq_max_CQ;
%solve the equation for CP
bpm_impeller_eq_CP_output               = polyval(abp_impeller_eq_CQ2CP_poly, bpm_impeller_eq_CQ_input);    %[-] CP = f(CQ)
%calculate total area from equation A = (Cq*D*h)/sqrt(2*CP)
bpm_impeller_eq_totalarea_output        = (bpm_impeller_eq_CQ_input*abp_impeller_diameter*abp_impeller_height)./sqrt(2*bpm_impeller_eq_CP_output); %[-] 

%change into lookup table parameters
bpm_lookup_Cdp_data                     = bpm_impeller_eq_CP_output;
bpm_lookup_totalarea_breakpoints        = bpm_impeller_eq_totalarea_output;

%% Nozzle Misalignment
% 1: Define a vector perpendicular to the sensor's sense axis, and randomly rotate it about the sense axis (360 degrees)
% 2: Rotate the sensors sense axis about that random perpendicular vector (by the misalignment angle)
for i=1:length(abp_PM1_nozzle_orientations)
    
    %Define an arbitrary vector
    if abs(abp_PM1_nozzle_orientations(i,1)) ~= 1
        temp_vec1 = [1 0 0];
    else
        temp_vec1 = [0 1 0];
    end
    if abs(abp_PM2_nozzle_orientations(i,1)) ~= 1
        temp_vec2 = [1 0 0];
    else
        temp_vec2 = [0 1 0];
    end
    
    % Find a perpendicular unit vector
    temp_perp_vec1 = cross(abp_PM1_nozzle_orientations(i,:), temp_vec1);
    temp_perp_vec2 = cross(abp_PM2_nozzle_orientations(i,:), temp_vec2);
    temp_perp_vec1 = temp_perp_vec1/norm(temp_perp_vec1);
    temp_perp_vec2 = temp_perp_vec2/norm(temp_perp_vec2);
    
    %Convert axis-angle to quaternion that rotates the perpendicular vector randomly around the nozzle unit vector (0-360 deg)
    temp_rotation_quat1 = [sin(bpm_PM1_nozzle_orientation_az_error(i)/2)*abp_PM1_nozzle_orientations(i,:) cos(bpm_PM1_nozzle_orientation_az_error(i)/2)];
    temp_rotation_quat2 = [sin(bpm_PM2_nozzle_orientation_az_error(i)/2)*abp_PM2_nozzle_orientations(i,:) cos(bpm_PM2_nozzle_orientation_az_error(i)/2)];
    
    %Rotate
    temp_misalignment_rot_axis1 = quat_rotation_vec(temp_perp_vec1,temp_rotation_quat1);  
    temp_misalignment_rot_axis2 = quat_rotation_vec(temp_perp_vec2,temp_rotation_quat2);
    
    % Define the quat. to rotate the nozzle axis about the newly defined vector by the misalignment amount
    bpm_PM1_Q_nozzle2misaligned(i,:) = [sin(bpm_PM1_nozzle_orientation_el_error(i)/2)*temp_misalignment_rot_axis1 cos(bpm_PM1_nozzle_orientation_el_error(i)/2)];
    bpm_PM2_Q_nozzle2misaligned(i,:) = [sin(bpm_PM2_nozzle_orientation_el_error(i)/2)*temp_misalignment_rot_axis2 cos(bpm_PM2_nozzle_orientation_el_error(i)/2)];

end