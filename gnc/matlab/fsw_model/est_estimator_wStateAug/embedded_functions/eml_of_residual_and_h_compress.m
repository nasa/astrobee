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
function [ r_comp, H_comp ] = eml_of_residual_and_h_compress(r, H, ase_of_num_aug)
%#codegen
   % H is [ x * 12 0 * 12; x * 24; x * 24; x * 24; 0 * 12 x * 12; ...]
%    H = H([2:5:size(H, 1), 3:5:size(H, 1), 4:5:size(H, 1), 1:5:size(H, 1), 5:5:size(H, 1)], :);
   
   rows = 6 * ase_of_num_aug;
   for col=1:rows
       for row=size(H, 1):-1:1+col
           [H, r] = apply_givens(H, r, [row-1, row], col, col:rows);
       end
   end
%    % make first three fifths upper triangular
%    for col=1:rows
%        for row=size(H, 1)-2*size(H, 1) / 5:-1:1+col
%            [H, r] = apply_givens(H, r, [row-1, row], col, col:rows);
%        end
%    end
%    % make next fifth upper triangular
%    for col=1:rows/2
%        for row=size(H, 1)-size(H, 1) / 5:-1:size(H,1)-2*size(H, 1)/5+col+1
%            [H, r] = apply_givens(H, r, [row-1, row], col, col:rows/2);
%        end
%    end
%    % make bottom fifth upper triangular
%    for col=rows/2+1:rows
%        for row=size(H, 1):-1:size(H,1)-size(H, 1)/5+1+col-rows/2
%            [H, r] = apply_givens(H, r, [row-1, row], col, col:rows);
%        end
%    end
%    % eliminate fourth fifth using top part
%    for col=1:rows
%        for row=1:min(col, min(rows/2, size(H,1)/5))
%            [H, r] = apply_givens(H, r, [col, 3 * size(H,1) / 5 + row], col, col:rows);
%        end
%    end
%    % eliminate fifth fifth using top part
%    for col=rows/2+1:rows
%        for row=1:min(col - (rows / 2), size(H,1)/5)
%            [H, r] = apply_givens(H, r, [col, 4 * size(H,1) / 5 + row], col, col:rows);
%        end
%    end

   % Now place the r and H in terms of the full state vector
   r_comp = r(1:rows);
   H_comp = zeros(rows, 15 + 6 + 6 * ase_of_num_aug);
   H_comp(:, 22:end) = H(1:rows, :);
   %R_mat_comp = R_mat(1:rows, 1:rows);
end

function [H, r] = apply_givens(H, r, rows, col, H_range)
   [g, ~] = planerot(H(rows, col));
   H(rows, H_range) = g * H(rows, H_range);
   r(rows, :)       = g * r(rows, :);
   %R_mat(rows, :)   = g * R_mat(rows, :);
   %R_mat(:, rows)   = R_mat(:, rows) * g';
end

