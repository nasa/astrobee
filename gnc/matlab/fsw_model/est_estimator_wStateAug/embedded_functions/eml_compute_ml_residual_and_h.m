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

function [r_out, error_out, H_out, R_mat, numFeatures, mahal_dists] = eml_compute_ml_residual_and_h(ase_inv_focal_length, ase_distortion, ml_P_cam_ISS_ISS, ml_quat_ISS2cam, is_converged, omega, velocity, L, O, valid, P_in, mahal_distance_max, ase_min_ml_meas_tested, ase_of_num_aug, ase_ml_num_features, ase_vis_r_mag, ase_map_error, abp_q_body2cam, ase_ml_forward_projection_time, ML_update, ase_vis_r_mag_AR)
%#codegen
  O = O(logical(valid), :);
  L = L(logical(valid), :);
  O = O .* ase_inv_focal_length;

  camera_ml_tf_global = eye(3, 4);
  camera_ml_tf_global(:, 1:3) = quaternion_to_rotation(ml_quat_ISS2cam);
  camera_ml_tf_global(:, 4) = -camera_ml_tf_global(:, 1:3) * ml_P_cam_ISS_ISS;

  camera_landmarks = camera_ml_tf_global * [L'; ones(1, size(L, 1))]; % Landmarks in the camera frame

  error_out = int32(0);
  
  z_est = camera_landmarks([1 2], :) ./ camera_landmarks([3 3], :);

  r = single(reshape(O' - z_est, size(O, 1) * 2, 1));

  % forward propagate the saved state based on velocity to add an error
  % term for the timing of the registration pulse
  next_ml_tf_global = eye(3, 4);
  % move to camera frame and with half a frame rotation
  % TODO: we should use the velocity and omega from the time of the
  % registration pulse
  cam_omega = ase_ml_forward_projection_time * quat_rotation_vec(omega', abp_q_body2cam);
  camera_angle = eulers_to_quat(cam_omega(1), cam_omega(2), cam_omega(3));
  next_ml_tf_global(:, 1:3) = quaternion_to_rotation(quatmult(camera_angle, ml_quat_ISS2cam')');
  next_ml_tf_global(:, 4) = -camera_ml_tf_global(:, 1:3) * ml_P_cam_ISS_ISS + ase_ml_forward_projection_time * velocity;
  next_landmarks = next_ml_tf_global * [L'; ones(1, size(L, 1))]; % Landmarks in the camera frame
  z_next = next_landmarks([1 2], :) ./ next_landmarks([3 3], :);
  omega_error = single(abs(reshape(z_est - z_next, size(O, 1) * 2, 1)));
  
  % compute Jacobian
  r_vec = zeros(size(O, 1) * 2, 1);
  H = zeros(2 * size(O, 1), 6, 'single');
  for i=1:size(O, 1)
    temp = 1.0 ./ camera_landmarks(3, i) * [eye(2, 2) -z_est(:, i)];
    H_theta = temp * skew(camera_landmarks(:, i));
    H_p = -temp * camera_ml_tf_global(:, 1:3);
    H(i*2-1:i*2,:) = [H_theta H_p];
    
    % account for distortion model
    d2 = 2 * tan(ase_distortion / 2);
    ru = ase_inv_focal_length * norm(O(i, :));
    f1 = ase_inv_focal_length;
    f2 = ase_inv_focal_length;
%     if ru > 1e-5
%         distorted = O(i, :) * tan(ru * ase_distortion) / d2 / ru;
%         radius = norm(distorted);
%         rd = radius * ase_inv_focal_length;
%         t = tan(ase_distortion * rd);
%         f1 = f1 * 1 / (d2 * radius) * (distorted(2)^2 * t / rd + ase_distortion * distorted(1)^2 * (1 + t^2));
%         f2 = f2 * 1 / (d2 * radius) * (distorted(1)^2 * t / rd + ase_distortion * distorted(2)^2 * (1 + t^2));
%     end
    
    if ML_update 
        % make the confidence depend on mapped landmarked error which is function
        % of distance
        r_vec(i*2-1:i*2) = single(ase_map_error ./ camera_landmarks(3, i)' + [f1; f2] * ase_vis_r_mag + omega_error(i*2-1:i*2)) .^ 2;
    else % AR update
        r_vec(i*2-1:i*2) = single( [f1; f2] * ase_vis_r_mag_AR + omega_error(i*2-1:i*2)) .^ 2;
    end
  end
  %r_out = zeros(size(L, 1),1);
  mahal_dists = nan(ase_ml_num_features,1);
  
  num_original = size(r, 1);
  % only ignore observations that don't match if we are converged
  if is_converged && ML_update
%     S = H*P_in(16:21, 16:21)*H';
%     for i=1:size(S, 1)
%       S(i, i) = S(i, i) + r_vec(i);
%     end
%     S_inv = pinv(S);
    for jj = 1:2:size(L, 1)*2
      S = H(jj:jj+1, :) * P_in(16:21, 16:21) * H(jj:jj+1, :)';
      S = S + [r_vec(jj) 0; 0 r_vec(jj+1)];
      S_inv = pinv(S);
      mahal_dist = sqrt(r(jj:jj+1)'*S_inv*r(jj:jj+1));
      mahal_dists((jj+1)/2) = mahal_dist;
    end
    invalid_vec = (mahal_dists(1:size(L, 1))' > mahal_distance_max);
    % toss if nothing is within std. dev. of expected, ransac prob. failed
%     if  sum(mahal_dists < 3) == 0
%         invalid_vec(:) = true;
%     end
    invalid = reshape([invalid_vec; invalid_vec], size(L, 1)*2, 1);
    r(invalid) = [];
    H(invalid,:) = [];
    r_vec(invalid) = [];
  end
  
  numFeatures = uint8(size(r, 1)/2);
  % toss if not enough measurements
  if size(r, 1)/2 < ase_min_ml_meas_tested || size(r, 1) < num_original / 2
    numFeatures = uint8(0);
    error_out = int32(1);
    total_num_states = 21 + (ase_of_num_aug*6); 
    r_out = ones(6, 1, 'single');
    H_out = eye(6, total_num_states, 'single');
    R_mat = eye(6, 6, 'single');
    return;
  end

  
  
  % Compress the r and H
  [q1, T_H] = qr(H);
  N = 6;
  q1 = q1(:, 1:N);

  H = single(T_H(1:N, :));
  r_out = single(q1' * r);
  R_mat = single(q1' * diag(r_vec) * q1);

  % Convert H to be in terms of the entire state vector
  rows = size(H, 1);
  H_out = [H zeros(rows, 15 + 6 * ase_of_num_aug, 'single')];
end
