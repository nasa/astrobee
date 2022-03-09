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
% flow measurments.
function [ of_measured, global_points, camera_tf_global, valid_out, num_of_tracks ] = eml_compute_of_global_points(of_quat_ISS2CAM, of_P_cam_ISS_ISS, of_measured_in, valid, ase_inv_focal_length, ase_of_num_aug)
%#codegen
  % for some reason matlab doesn't like creating matrix sizes based on "unbounded" parameters
%   if ase_of_num_aug ~= 5
%     error('eml_compute_of_global_points is optimized for four augmented states.');
%   end
%   ase_of_num_aug = 4;
  ase_of_num_aug = double(ase_of_num_aug);
  ase_of_num_features = size(valid, 1);
  of_measured = zeros(2 * ase_of_num_features * ase_of_num_aug, 1, 'single');
  global_points = ones(4, ase_of_num_features, 'single');
  valid_out = zeros(ase_of_num_features, ase_of_num_aug, 'int32');
  
  of_measured_in = of_measured_in .* ase_inv_focal_length;       % Convert measurements to normalized units
  % Note for some reason of_measured and valid can not match? Why?

  % Select out only features that are seen by three or more cameras
  valid_in_all = sum(valid, 2) >= 3;
  of_measured_in = of_measured_in(valid_in_all, :, :);
  num_of_tracks = int32(size(of_measured_in, 1));
  valid_out(1:num_of_tracks, :) = valid(valid_in_all, :);

  % reshape the measurements so the order is by feature, x measure then y
  % measure, with every camera observation one after the other.
  of_measured(1:2 * num_of_tracks * ase_of_num_aug) = ...
    reshape(reshape(permute(of_measured_in, [3 1 2]), ...
                    [ase_of_num_aug * num_of_tracks 2])', ...
            [2 * ase_of_num_aug * num_of_tracks 1]);

  % Extract the 4x4 transforms which convert points from global frame to camera
  % frame. I'll be stacking them vertically so they can all be evaluated
  % simultaneously.
  camera_tf_global = repmat(eye(4, 4, 'single'), ase_of_num_aug, 1);
  for i = 1:ase_of_num_aug
    rot_indices = 4*i-3:4*i-1;
    camera_tf_global(rot_indices, 1:3) = ...
      quaternion_to_rotation(of_quat_ISS2CAM(i,:)');
    camera_tf_global(rot_indices, 4) = ...
      -camera_tf_global(rot_indices, 1:3) * of_P_cam_ISS_ISS(i,:)';
  end

  % Generate the transforms from camera 1 into all the other cameras. This is
  % used for point triangulation.
  camera_tf_camera1 = repmat(eye(4), ase_of_num_aug, 1);
  global_tf_camera1 = pinv(camera_tf_global(1:4, 1:4)); % Used to be inv
  for i = 1:ase_of_num_aug
    camera_tf_camera1(4*i-3:4*i, 1:4) = ...
      camera_tf_global(4*i-3:4*i, 1:4) * global_tf_camera1;
  end

  % Triangulate all of the points
  % for use interanlly, allocate outside for speed
  A = zeros(2 * ase_of_num_aug, 3, 'single');
  b = zeros(2 * ase_of_num_aug, 1, 'single');
  for j = 1:num_of_tracks
    % Solving for location of the point in the 3D world

    % Forming A and b matrices, used in a Ax=b to solve for alpha, beta, rho,
    % an inverse depth parameterization of the points location in the 1st
    % camera.
    num_augs = sum(valid_out(j, :));
    aug_ind = 1;
    for i = 1:num_augs
      while ~valid_out(j, aug_ind)
          aug_ind = aug_ind + 1;
      end
      ind0 = 4*aug_ind - 4;
      uv = of_measured((j - 1) * 2 * ase_of_num_aug + 2 * aug_ind - 1: ...
                       (j - 1) * 2 * ase_of_num_aug + 2 * aug_ind, 1);

      A(2*i-1:2*i, 1:3) = ...
        [camera_tf_camera1(ind0+1, 1) - camera_tf_camera1(ind0+3, 1) * uv(1), ...
         camera_tf_camera1(ind0+1, 2) - camera_tf_camera1(ind0+3, 2) * uv(1), ...
         camera_tf_camera1(ind0+1, 4) - camera_tf_camera1(ind0+3, 4) * uv(1); ...
         camera_tf_camera1(ind0+2, 1) - camera_tf_camera1(ind0+3, 1) * uv(2), ...
         camera_tf_camera1(ind0+2, 2) - camera_tf_camera1(ind0+3, 2) * uv(2), ...
         camera_tf_camera1(ind0+2, 4) - camera_tf_camera1(ind0+3, 4) * uv(2)];
      b(2*i-1:2*i, 1) = ...
        [camera_tf_camera1(ind0+3, 3) * uv(1) - camera_tf_camera1(ind0+1, 3); ...
         camera_tf_camera1(ind0+3, 3) * uv(2) - camera_tf_camera1(ind0+2, 3)];
      aug_ind = aug_ind + 1;
    end
    % let's weight the oldest augmentation more heavily
    A(2*num_augs-1:2*num_augs, :) = 5 * A(2*num_augs-1:2*num_augs, :);
    b(2*num_augs-1:2*num_augs, :) = 5 * b(2*num_augs-1:2*num_augs, :);
    inv_depth_p = pinv(A(1:num_augs * 2, :)) * b(1:num_augs * 2, :);
    % solve Ax = B with guass newton

    if inv_depth_p(3) < 1e-5
      % This is a point that is truly in our face because we have no
      % parallax on it. I only see this happen when using perfect camera
      % positions and the robot is stationary (our noise usually makes it so
      % that inverse depth is non zero).
      inv_depth_p(3) = 1e-5;
    end

    % Converting the inverse depth parameterization into the global coordinate
    % frame.
    global_points(1:3, j) = global_tf_camera1(1:3, :) * ([inv_depth_p(1:2) ./ inv_depth_p(3); 1 / inv_depth_p(3); 1]);
  end

end
