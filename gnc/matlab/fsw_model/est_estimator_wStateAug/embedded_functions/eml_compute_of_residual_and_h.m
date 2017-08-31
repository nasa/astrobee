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

% Code is taken from Zack Morrato's prototyped Optical Flow Kalman Filter.
% This code computes the residual and measurment matrices for the optical
% flow measurements.
function [ r, H, error_out, num_of_tracks, mahal_dists ] = eml_compute_of_residual_and_h(of_measured, global_points, camera_tf_global, valid, ase_of_num_aug, ase_of_num_features, tun_ase_mahal_distance_max, ase_of_r_mag, P)
%#codegen

%   if ase_of_num_aug ~= 5
%     error('eml_compute_of_residual_and_h is optimized for four augmented states.');
%   end
%   ase_of_num_aug = 4;
  
  error_out = 0;
  num_of_tracks = size(global_points, 2);

  z_indices = zeros(1, 2 * ase_of_num_aug);
  z_indices(1:2:end) = 3:4:4 * ase_of_num_aug;
  z_indices(2:2:end) = 3:4:4 * ase_of_num_aug;

  xy_indices = zeros(1, 2 * ase_of_num_aug);
  xy_indices(1:2:end) = 1:4:4 * ase_of_num_aug;
  xy_indices(2:2:end) = 2:4:4 * ase_of_num_aug;

  % Find the reprojection location for all of the points, vectorized. And also calculate the error.
  camera_landmarks = camera_tf_global * global_points;
  z_est = camera_landmarks(xy_indices, :) ./ camera_landmarks(z_indices, :);
  r_uncompressed = of_measured - reshape(z_est, 2 * ase_of_num_aug * num_of_tracks, 1);
  
  % forward propagate the saved state based on velocity to add an error
  % term for the timing of the registration pulse
  %next_ml_tfs_global = zeros(4 * ase_of_num_aug, 4);
  % move to camera frame and with half a frame rotation
  %for i=1:ase_of_num_aug
    %cam_omega = ase_of_forward_projection_time * quat_rotation_vec(of_omega_aug(i, :)', abp_q_body2cam);
    %camera_angle = eulers_to_quat(cam_omega(1), cam_omega(2), cam_omega(3));
    %next_ml_tfs_global(4*i-3:4*i-1, 1:3) = quaternion_to_rotation(camera_angle');
    %next_ml_tfs_global(4*i-3:4*i, 4) = [ase_of_forward_projection_time * of_vel_aug(i, :)'; 1];
    %next_ml_tfs_global(4*i-3:4*i, :) = next_ml_tfs_global(4*i-3:4*i, :) * camera_tf_global(4 * i - 3:4 * i, :);
  %end
  %next_landmarks = next_ml_tfs_global * global_points; % Landmarks in the camera frame
  %z_next = next_landmarks(xy_indices, :) ./ next_landmarks(z_indices, :);
  %omega_error = single(abs(reshape(z_est - z_next, size(r_uncompressed, 1), 1)));
  
  r = zeros((2 * ase_of_num_aug - 3) * num_of_tracks, 1, 'single');
  H = zeros((2 * ase_of_num_aug - 3) * num_of_tracks, 6 * ase_of_num_aug, 'single');
  mahal_dists = nan(ase_of_num_features, 1); % Allocate to maximum size
  %R_mat = zeros(size(H, 1), size(H, 1), 'single');

  cur_row = 1;
  % Calculate measurement jacobians and compression from Hf
  for j = 1:num_of_tracks
    % Copy from residuals as we'll do givens in place
    r_j = r_uncompressed((j-1) * 2 * ase_of_num_aug + 1: ...
                         j * 2 * ase_of_num_aug);
    r_j = r_j(logical(reshape([valid(j, :); valid(j, :)], size(r_j, 1), 1)));
    %R_mat_j = ase_of_r_mag .^ 2 * eye(size(r_j, 1), size(r_j, 1));

    % Calculate H_theta H_p and H_f
    H_f_j = zeros(size(r_j, 1), 3, 'single');
    H_x_j = zeros(size(r_j, 1), ase_of_num_aug * 6, 'single');
    % This is not really all of H_x .. this is just
    % the part of H that covers the OF camera augmentations.
    aug_ind = 1;
    for i = 1:size(r_j, 1) / 2
      while ~valid(j, aug_ind)
          aug_ind = aug_ind + 1;
      end
      ind0 = 4 * aug_ind - 4;
      prefix = (1.0 ./ camera_landmarks(ind0+3, j)) * [eye(2, 2) -z_est(2*aug_ind-1:2*aug_ind, j)];
      H_theta_ji = prefix * skew(camera_landmarks(ind0+1:ind0+3, j));
      H_p_ji = -prefix * camera_tf_global(ind0+1:ind0+3, 1:3);

      % Fill in H_x
      H_x_j(i*2-1:i*2, aug_ind*6-5:aug_ind*6) = [H_theta_ji H_p_ji];

      % Fill in H_f
      H_f_j(i*2-1:i*2, :) = -H_p_ji;
      aug_ind = aug_ind + 1;
    end

    % Project everything on to the left nullspace of H_f
    %
    % The direct approach is:
    %   A = null(H_f')
    %   H_x_of = A' * H_x and r_of = A' * r_j
    %
    % The nullspace can be calculated quickly with QR, we just need to remember
    % to remove a few extract columns in Q (which is the nullspace). QR
    % decomposition can be calculated quickly with Givens rotations. And ..
    % finally Givens rotations can be done in place. So all this mess you see
    % next is because we're doing nullspace projection in place using the
    % Given's rotation, which is a fix number of observations.
    %
    % This is really fast in C/C++. Matlab, I dunno .. probably not because of
    % the loop. Simulink Autogenerated code? Let's hope?

    % first, make each half of H_f_j (stacked vertically) upper triangular
%     for col = 1:3
%       for row = 4:-1:(col+1)
%         [H_f_j, H_x_j, r_j] = apply_givens(H_f_j, H_x_j, r_j, [row-1, row], col, col:3, 1:12);
%       end
%     end
%     for col = 1:3
%       for row = 4:-1:(col+1)
%         [H_f_j, H_x_j, r_j] = apply_givens(H_f_j, H_x_j, r_j, [4+row-1, 4+row], col, col:3, 13:24);
%       end
%     end
%     % now we have [x x x; 0 x x; 0 0 x; 0 0 0; x x x; 0 x x; 0 0 x; 0 0 0]
%     [H_f_j, H_x_j, r_j] = apply_givens(H_f_j, H_x_j, r_j, [1, 5], 1, 1:3, 1:24);
%     % [x x x; 0 x x; 0 0 x; 0 0 0; 0 x x; 0 x x; 0 0 x; 0 0 0]
%     [H_f_j, H_x_j, r_j] = apply_givens(H_f_j, H_x_j, r_j, [2, 5], 2, 2:3, 1:24);
%     [H_f_j, H_x_j, r_j] = apply_givens(H_f_j, H_x_j, r_j, [2, 6], 2, 2:3, 1:24);
%     % [x x x; 0 x x; 0 0 x; 0 0 0; 0 0 x; 0 0 x; 0 0 x; 0 0 0]
%     [H_f_j, H_x_j, r_j] = apply_givens(H_f_j, H_x_j, r_j, [3, 5], 2, 3:3, 1:24);
%     [H_f_j, H_x_j, r_j] = apply_givens(H_f_j, H_x_j, r_j, [3, 6], 2, 3:3, 1:24);
%     [H_f_j, H_x_j, r_j] = apply_givens(H_f_j, H_x_j, r_j, [3, 7], 2, 3:3, 1:24);

    % original version
    for col = 1:3
      for row = size(r_j, 1):-1:(col+1)
        [H_f_j, H_x_j, r_j] = apply_givens(H_f_j, H_x_j, r_j, [row-1, row], col, col:size(H_f_j, 2), 1:size(H_x_j, 2));
      end
    end

    % We now discard the top 3 rows ... (this follows with the algorithm for
    % calculating the nullspace using QR decomposition.) However I believe you
    % can also think of this as 3 DoF loss due to the EKF not solving for this
    % feature's physical location. We can do this because we projected on the
    % nullspace of H_f.
    %dim = rank(H_f_j);
    dim = 3;
    r_j = r_j(1+dim:end);
    %R_mat_j = R_mat_j(1+dim:end, 1+dim:end);
    H_x_j = H_x_j(1+dim:end, :);
    
    % check mahalanobis distance now
    S = H_x_j * P(22:21 + ase_of_num_aug * 6, 22:21 + ase_of_num_aug * 6) * H_x_j';
    S = S + ase_of_r_mag .^ 2 * eye(size(S, 1), size(S, 2));
    S_inv = pinv(S);
    mahal_dist = sqrt(r_j'*S_inv*r_j);
    mahal_dists(j) = mahal_dist;
    % don't use feature if distance too great
    if mahal_dist > tun_ase_mahal_distance_max
        continue;
    end

    % Place our solution for j in the big r vector and big H matrices.
    next_row = cur_row + size(r_j, 1);
    r(cur_row:next_row - 1,:) = r_j;
    %R_mat(cur_row:next_row - 1, cur_row:next_row - 1) = R_mat_j;
    H(cur_row:next_row - 1, :) = H_x_j;
    cur_row = next_row;
  end
  r = r(1:cur_row - 1, :);
  H = H(1:cur_row - 1, :);
  %R_mat = R_mat(1:cur_row - 1, 1:cur_row - 1);
  
  if size(r, 1) < 6 * ase_of_num_aug
      error_out = 1;
      num_of_tracks = 0;
      r = ones(6 * ase_of_num_aug, 1, 'single');
      H = zeros(6 * ase_of_num_aug, size(H, 2), 'single');
      %R_mat = eye(6 * ase_of_num_aug, 6 * ase_of_num_aug, 'single');
  end
end

function [H_f, H_x, r] = apply_givens(H_f, H_x, r, rows, col, H_f_range, H_x_range)
   [g, ~] = planerot(H_f(rows, col));
   H_f(rows, H_f_range) = g * H_f(rows, H_f_range);
   H_x(rows, H_x_range) = g * H_x(rows, H_x_range);
   %R_mat(rows, :)   = g * R_mat(rows, :);
   %R_mat(:, rows)   = R_mat(:, rows) * g';
   r(rows, :)            = g * r(rows, :);
end
