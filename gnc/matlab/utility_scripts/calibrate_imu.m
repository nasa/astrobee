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
%  1. Get imu and camera pose estimates from cad files.
%  2. Record bag of spinning in place with no external forces. Set zero_ind
%     to indices of a single spin with sinusoidal acceleration.
%     Make sure that the imu omega is not saturated.
%  3. Run calibrate_imu to get IMU pose.
function imu_offset = calibrate_imu(zero_ind)
  evalin('base', 'tunable_init;');

  directions = [0 0 0 0 1 0; ...
                0 0 0 0 0 1; ...
                0 0 0 1 0 0; ...
                1 0 0 0 0 0; ...
                0 1 0 0 0 0; ...
                0 0 1 0 0 0;];
  multipliers = [1 1 1 0.02 0.02 0.02]';
  windows    = [10 2   0.2  0.02]';
  precisions = [1  0.1 0.01 0.001]';
  best = [0 0 0 0 0 0];
  for d=1:size(windows, 1)
    for i=1:size(directions, 1)
      r = multipliers(i) * windows(d) * directions(i, :);
      best = minima_search(@test_transform, zero_ind, best - r, best + r, multipliers(i) * precisions(d));
    end
  end
  t = best(4:6) * pi / 180;
  q = eulers_to_quat(t(1), t(2), t(3));
  original_rot = evalin('base', 'tun_abp_quat_body2imu');
  q = quatmult(q, original_rot);
  fprintf('Best pose: [%10.8g, %10.8g, %10.8g] translation, [%10.8g, %10.8g, %10.8g, %10.8g] quaternion rotation.\n', ...
          best(1), best(2), best(3), q(1), q(2), q(3), q(4));
end

function best = minima_search(error_function, zero_ind, first, last, precision)
  fprintf('Beginning optimization.\n');
  %starterr = error_function(zero_ind, first);
  %enderr = error_function(zero_ind, last);
  %fprintf(['Left: ',  mat2str(first), ' Error: %g\n'], starterr);
  %fprintf(['Right: ', mat2str(last),  ' Error: %g\n'], enderr);
  total = sum(last-first);
  while total >= precision
    mid = 0.5 * (first + last);
    dx = (last-first) ./ sum(last-first);
    errs = [0 0 0];
    offsets = [-precision * 0.001 0 precision * 0.001];
    for i=1:3
      errs(i) = error_function(zero_ind, mid + offsets(i) * dx);
    end
    lefterr = errs(1);
    miderr = errs(2);
    righterr = errs(3);
    fprintf(['Pose: [%8.6g %8.6g %8.6g %8.6g %8.6g %8.6g] Step: %6g, Errors: %6g, %6g, %6g.\n'], ...
            mid(1), mid(2), mid(3), mid(4), mid(5), mid(6), precision * 0.001, lefterr, miderr, righterr);
    if miderr < righterr && miderr < lefterr
      fprintf('Found local minima.\n');
      break;
    end
    if righterr < lefterr
      first = mid;
    else
      last = mid;
    end
    total = sum(last-first);
  end
  best = mid;
  fprintf(['Best: ', mat2str(best), ' Error: %g\n'], miderr);
end

function err = test_transform(zero_ind, params)
  imu_offset = params(1:3);
  params(4:end) = params(4:end) .* pi ./ 180;
  imu_rotation = eulers_to_quat(params(4), params(5), params(6));

  assignin('base', 'show_graphs', false);
  assignin('base', 'show_results', false);
  assignin('base', 'disable_config_overwrite', true);
  assignin('base', 'disable_ml', true);
  assignin('base', 'disable_of', true);
  original_imu = evalin('base', 'tun_abp_p_imu_body_body');
  original_quat = evalin('base', 'tun_abp_quat_body2imu');
  assignin('base', 'tun_abp_p_imu_body_body', original_imu + imu_offset);
  assignin('base', 'tun_abp_quat_body2imu', quatmult(imu_rotation, original_quat));
  evalin('base', 'ase_hw_data_test');
  assignin('base', 'tun_abp_p_imu_body_body', original_imu);
  assignin('base', 'tun_abp_quat_body2imu', original_quat);
  evalin('base', 'clear show_graphs, show_results, disable_config_overwrite, disable_ml, disable_of;');

  omega = evalin('base', 'out_kfl_msg.omega_B_ISS_B.Data');
  errors = var(omega(:, 1:2));
  err = sqrt(sum(errors));
  accel = evalin('base', 'out_kfl_msg.A_B_ISS_ISS');
  accel = accel.Data(zero_ind, :);
  err = err + sqrt(sum(var(accel)));
end

