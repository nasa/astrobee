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

% How to calibrate:
% Run this on a bag full of data.
function imu_offset = calibrate_camera_extrinsics()
  evalin('base', 'tunable_init');

  directions = [0 0 0 0 1 0; ...
                0 0 0 0 0 1; ...
                0 0 0 1 0 0; ...
                1 0 0 0 0 0; ...
                0 1 0 0 0 0; ...
                0 0 1 0 0 0;];
  multipliers = [1 1 1 0.02 0.02 0.02]';
  windows    = [4 2   1  0.5]';
  precisions = [2  1 0.5 0.2]';
  best = [0 0 0 0 0 0];
  for d=1:size(windows, 1)
    for i=1:size(directions, 1)
      r = multipliers(i) * windows(d) * directions(i, :);
      best = minima_search(@test_transform, best - r, best + r);
    end
  end
  t = best(4:6) * pi / 180;
  q = eulers_to_quat(t(1), t(2), t(3));
  original_rot = evalin('base', 'tun_abp_q_body2cam');
  q = quatmult(q, original_rot);
  fprintf('Best pose: [%10.8g, %10.8g, %10.8g] translation, [%10.8g, %10.8g, %10.8g, %10.8g] quaternion rotation.\n', ...
          best(1), best(2), best(3), q(1), q(2), q(3), q(4));
end

function best = minima_search(error_function, first, last)
  fprintf('Beginning optimization.\n');
  besterr = inf;
  best = 0;
  for val=0:6
    cur = first + (last - first) * (val / 6);
    err = error_function(cur);
    fprintf(['Cur: ', mat2str(cur), ' Error: %g\n'], err);
    if err < besterr
      besterr = err;
      best = cur;
    end
  end
  fprintf(['Best: ', mat2str(best), ' Error: %g\n'], besterr);
end

function err = test_transform(params)
  imu_offset = params(1:3);
  params(4:end) = params(4:end) .* pi ./ 180;
  imu_rotation = eulers_to_quat(params(4), params(5), params(6));

  assignin('base', 'show_graphs', false);
  assignin('base', 'show_results', false);
  assignin('base', 'disable_config_overwrite', true);
  original_imu = evalin('base', 'tun_abp_p_cam_body_body');
  original_quat = evalin('base', 'tun_abp_q_body2cam');
  assignin('base', 'tun_abp_p_cam_body_body', original_imu + imu_offset);
  assignin('base', 'tun_abp_q_body2cam', quatmult(imu_rotation, original_quat));
  evalin('base', 'ase_hw_data_test');
  assignin('base', 'tun_abp_p_cam_body_body', original_imu);
  assignin('base', 'tun_abp_q_body2cam', original_quat);
  evalin('base', 'clear show_graphs show_results disable_config_overwrite disable_ml disable_of;');

  err = evalin('base', 'angular_rmse / 100 + pos_rmse');
end

