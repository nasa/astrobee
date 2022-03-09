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

function [r_out, error_out, H_out, R_mat] = eml_compute_hr_residual_and_h(ml_P_cam_ISS_ISS, ml_quat_ISS2cam, omega, velocity, L, O, valid, ase_of_num_aug, ase_hr_r_mag, ase_hr_distance_r, ase_ml_forward_projection_time, abp_q_body2cam, hr_quat_cam2hr, hr_3d_knowledge_flag)
%#codegen
  error_out = int32(0);
  if hr_3d_knowledge_flag == 1
    handrail_knowledge_dims = 3;
  else
    handrail_knowledge_dims = 2;
  end

  % Get tf matrix that converts from global position (world frame)
  % to local position (camera frame)
  C = quaternion_to_rotation(ml_quat_ISS2cam);
  camera_ml_tf_global = eye(3, 4);
  camera_ml_tf_global(:, 1:3) = C;
  camera_ml_tf_global(:, 4) = -camera_ml_tf_global(:, 1:3) * ml_P_cam_ISS_ISS; 

  % Remove the invalid features from the observations and landmarks
  O = O(logical(valid), :); % Observations from the camera frame
  L = L(logical(valid), :); % Landmarks from the world frame

  % Convert global positions of the landmarks to the local positions
  camera_landmarks = camera_ml_tf_global * [L'; ones(1, size(L, 1))]; % Landmarks in the camera frame
        
  % Create rotation matrix to convert between bases
  % R rotation matrix that rotates from the camera frame into the handrail frame
  % with the axis_body vector as the z axis unit vector
  R = quaternion_to_rotation(hr_quat_cam2hr);   
%   R = eye(3);
  
  newerr = [R zeros(3, 1); zeros(1, 3) 1] * [O' - camera_landmarks; ones(1, size(L, 1))];
  r = reshape(newerr(1:handrail_knowledge_dims, :), size(O, 1) * handrail_knowledge_dims, 1);


  
  next_ml_tf_global = eye(3, 4);
  % move to camera frame and with half a frame rotation
  % TODO: we should use the velocity and omega from the time of the
  % registration pulse
  cam_omega = quat_rotation_vec(omega', abp_q_body2cam);
  next_quat = quat_propagate_step(ml_quat_ISS2cam', cam_omega, ase_ml_forward_projection_time);
  %camera_angle = eulers_to_quat(cam_omega(1), cam_omega(2), cam_omega(3));
  next_ml_tf_global(:, 1:3) = quaternion_to_rotation(next_quat');
  next_ml_tf_global(:, 4) = -camera_ml_tf_global(:, 1:3) * (ml_P_cam_ISS_ISS + ase_ml_forward_projection_time * velocity);
  next_landmarks = next_ml_tf_global * [L'; ones(1, size(L, 1))]; % Landmarks in the camera frame
  %z_next = next_landmarks;
  % Find the difference between the current landmark locations and the
  % distorted landmark locations due to time error, and rotate into the
  % handrail frame
  landmark_error_rail_frame = (R * (camera_landmarks - next_landmarks))';

  omega_error = single(abs(reshape(landmark_error_rail_frame(:,1:handrail_knowledge_dims), size(O,1)*handrail_knowledge_dims,1)));

  
  
  % compute Jacobian
  r_vec = zeros(size(O, 1) * handrail_knowledge_dims, 1);
  H = zeros(handrail_knowledge_dims * size(O, 1), 6, 'single');
  for i=1:size(O, 1)
    %temp = 1.0 ./ camera_landmarks(3, i) * [eye(2, 2) -z_est(:, i)];
    H_theta = eye(3) * skew(camera_landmarks(:, i));
    H_p = -eye(3) * camera_ml_tf_global(:, 1:3);
    H_all = R * [H_theta H_p]; % rotate to new coordinate system
    H(i*handrail_knowledge_dims-(handrail_knowledge_dims-1):i*handrail_knowledge_dims,:) = H_all(1:handrail_knowledge_dims, :); % drop last row, since we don't have knowledge along handrail axis

    
    % make the confidence depend on mapped landmarked error which is function
    % of distance
    % [??] Should the camera_landmarks distance be the RSS, not just Z?
    % r_vec is measurment noise

    r_vec(i*handrail_knowledge_dims-(handrail_knowledge_dims-1) : i*handrail_knowledge_dims) = ...
      single(ase_hr_distance_r * norm(camera_landmarks(3)', 'fro') + ase_hr_r_mag ...
             + omega_error(i*handrail_knowledge_dims-(handrail_knowledge_dims-1) : i*handrail_knowledge_dims) ...
            ).^ 2;
  end
  
  % Compress the r and H
  [q1, T_H] = qr(H);
  N = 6;
  q1 = q1(:, 1:N);

  H = T_H(1:N, :);
  r_out = single(q1' * r);
  R_mat = single(q1' * diag(r_vec) * q1);

  % Convert H to be in terms of the entire state vector
  rows = size(H, 1);
  H_out = [H zeros(rows, 15 + 6 * ase_of_num_aug, 'single')];
