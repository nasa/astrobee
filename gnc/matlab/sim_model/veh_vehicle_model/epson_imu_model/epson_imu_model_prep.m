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

% Epson IMU Model prep

%% misalignment
epson_Q_B2sensor_error      = eulers_to_quat(epson_imu_misalignment(1),epson_imu_misalignment(2),epson_imu_misalignment(3));
epson_Q_sensor2gyro_error   = eulers_to_quat(epson_gyro_orth_misalignment(1),epson_gyro_orth_misalignment(2),epson_gyro_orth_misalignment(3));
epson_Q_sensor2accel_error  = eulers_to_quat(epson_accel_orth_misalignment(1),epson_accel_orth_misalignment(2),epson_accel_orth_misalignment(3));

epson_Q_B2gyro_error        = quatmult(epson_Q_B2sensor_error, epson_Q_sensor2gyro_error);
epson_Q_B2accel_error       = quatmult(epson_Q_B2sensor_error, epson_Q_sensor2accel_error);

%% Moved to lua conig file [ASTROBEE_ROOT '/../../management/astrobee/config/gnc.config']
%tun_epson_P_sensor_B_B      = tun_epson_P_sensor_B_B_nom + tun_epson_imu_pos_error;     %[m] nominal positon of the sensor

%% Debug Warnings
if tun_epson_report_truth
    warning('IMU Noise has been disabled');
end

