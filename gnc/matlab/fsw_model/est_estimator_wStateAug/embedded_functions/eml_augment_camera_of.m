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

% This function add loads up the most recent
% camera attitude and position, as well as setting the aug_state_byte
function [state_out, P_out, of_vel_aug, of_omega_aug] = eml_augment_camera_of(P_cam_ISS_ISS, quat_ISS2cam, state_in, P_in, tun_abp_q_body2cam, tun_abp_p_cam_body_body, ase_total_num_states, ase_of_num_aug, replaced_aug)
%#codegen
ase_of_num_aug = double(ase_of_num_aug); % Cast this back to double to not cause issues with other integers
  % retain the values between function calls
  persistent aug_velocity aug_velocity_mag aug_omega aug_omega_mag;
  if isempty(aug_velocity) || isempty(aug_velocity_mag) || isempty(aug_omega) || isempty(aug_omega_mag)
      % Initialize values to zero and the same data type as state_in.V_B_ISS_ISS
      aug_velocity = zeros(ase_of_num_aug, 3, 'like', state_in.V_B_ISS_ISS);
      aug_velocity_mag = zeros(ase_of_num_aug,1, 'like', state_in.V_B_ISS_ISS);
      aug_omega = zeros(ase_of_num_aug,3, 'like', state_in.omega_B_ISS_B);
      aug_omega_mag = zeros(ase_of_num_aug,1, 'like', state_in.omega_B_ISS_B);
  end
  
 
  P = P_in;
  state_out = state_in;
  % Constants/variables I'll need later
  imu_q_global = state_in.quat_ISS2B;
  camera_rot_body = quaternion_to_rotation(tun_abp_q_body2cam');
  imu_rot_global = quaternion_to_rotation(imu_q_global);

  % Augment flag
  % [MSB   Oldest_OF_Aug ... Newest_OF_Aug Valid_ML_Augment(LSB)]
  of_bit_mask = bitshift((2^ase_of_num_aug-1),1); % Mask off just the OF bits
  of_bits = bitand(state_in.aug_state_enum, uint32(of_bit_mask)); % Extract out the bits associated with OF
  ml_aug_bit = bitget(state_in.aug_state_enum, 1);
  shifted_bits  = bitshift(of_bits, 1); % Shift all the OF bits 1 bit more significant
  of_valid_bits = bitset(shifted_bits, 2); % Set the newest OF bit as valid
  of_valid_bits = bitand(of_valid_bits, uint32(of_bit_mask)); % Mask off the unused bits, remove rollover
  state_out.aug_state_enum = uint32(bitor(of_valid_bits, ml_aug_bit)); % Restore the value of the ML aug bit
  %new_jf = bitset(bitshift(bitand(jf, bin2dec('111110')), 1), 2)
  
  % Augment the state vector
  % Ive arranged the state vector so it is:
  % [IMU location, ML Camera Location, Newest OF Camera Location .... Oldest OF Camera Loc]
  replaced_aug = ase_of_num_aug - replaced_aug + 1; % lowest number replaces most recent
  kept_augmentations = zeros(1, ase_of_num_aug-1);
  of_in_prange = zeros(1, 6 * (ase_of_num_aug - 1));
  j = 1;
  for i=1:ase_of_num_aug-1
      if j == replaced_aug
          j = j + 1;
      end
      kept_augmentations(i) = j;
      of_in_prange(6 * (i-1)+1:6*i) = 6 * (j-1) + 21 + (1:6);
      j = j + 1;
  end
  state_out.of_quat_ISS2cam(2:end, :) = state_out.of_quat_ISS2cam(kept_augmentations, :);
  state_out.of_quat_ISS2cam(1, :) = quat_ISS2cam';
  state_out.of_P_cam_ISS_ISS = [P_cam_ISS_ISS'; state_out.of_P_cam_ISS_ISS(kept_augmentations,:)];

  % Augment velocities
  aug_velocity = [state_in.V_B_ISS_ISS'; aug_velocity(kept_augmentations,:)];
  aug_velocity_mag = [norm(state_in.V_B_ISS_ISS,'fro'); aug_velocity_mag(kept_augmentations,:)];
  aug_omega = [state_in.omega_B_ISS_B'; aug_omega(kept_augmentations,:)];
  aug_omega_mag = [norm(state_in.omega_B_ISS_B,'fro'); aug_omega_mag(kept_augmentations,:)];

  of_vel_aug = double(aug_velocity);
  of_omega_aug = double(aug_omega);
  
  % Augmenting the covariance matrix
  % Move covariances down the stack:
  % P = [A B C] then P+1 = [A 0 B]
  %     [D E F]            [0 0 0]
  %     [G H J]            [D 0 E]
  imu_ml_prange = 1:21;
  of_out_prange = 21 + 6 + 1:21 + 6 * ase_of_num_aug;
  % Section D
  P(of_out_prange, imu_ml_prange) = P(of_in_prange, imu_ml_prange);
  % Section B
  P(imu_ml_prange, of_out_prange) = P(imu_ml_prange, of_in_prange);
  % Section E
  P(of_out_prange, of_out_prange) = P(of_in_prange, of_in_prange);
  % Now actually place an augmentation of IMU into OF
  % M here is actually just the left part of J as defined in equation 24 in the
  % Visinav paper by Mourikis '09 + space for the ML augmentation.
  cov_datatype = class(P_in);
  M = [cast(camera_rot_body, 'like', P) zeros(3, 12, cov_datatype) zeros(3, 6, cov_datatype);
       cast(skew(imu_rot_global' * tun_abp_p_cam_body_body'), 'like', P) zeros(3, 9, cov_datatype) eye(3, cov_datatype) zeros(3, 6, cov_datatype)];
  of_new_prange = 15 + 6 + 1:15 + 6 + 6;
%   J = [eye(21, size(P, 2)); M zeros(6, size(P, 2) - size(M, 2)); zeros(size(P, 1) - 27, 27) eye(size(P, 1) - 27, size(P, 2) - 27)];
%   P = J * P * J';
  % P = [A 0 C] then P+1 = [A  AMt  C ]
  %     [0 0 0]            [MA MAMt MC]
  %     [G 0 J]            [G  GMt  J ]
  % Center section
  P(of_new_prange, of_new_prange) = M * P(imu_ml_prange, imu_ml_prange) * M';
  % Top left sections
  P(of_new_prange, imu_ml_prange) = M * P(imu_ml_prange, imu_ml_prange);
  P(imu_ml_prange, of_new_prange) = P(of_new_prange, imu_ml_prange)';
  % Bottom right sections
  P(of_new_prange, of_out_prange) = M * P(imu_ml_prange, of_out_prange);
  P(of_out_prange, of_new_prange) = P(of_new_prange, of_out_prange)';

  P_out = zeros(ase_total_num_states, cov_datatype);
  P_out(1:ase_total_num_states,1:ase_total_num_states) = P(1:ase_total_num_states,1:ase_total_num_states);
end
