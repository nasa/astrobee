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

% Calculate the delta State
function [error_out, delta_state, P_out]  = eml_compute_delta_state_and_cov(residual, error_in, H, R_mat, P)
%#codegen
error_out = error_in;
if error_in
    delta_state = zeros(size(P, 1), 1, 'single');
    P_out = P;
    return;
end

T = P * H';
S = H*T + R_mat;
[C, p] = chol(S);

% this should never happen! but if it does, fix P to be semipositive
% definite
if p ~= 0
    P = fix_covariance(P);
    T = P * H';
    S = H*T + R_mat;
    [C, p] = chol(S);
end

if p ~= 0
    error_out = 1;
    delta_state = zeros(size(P, 1), 1, 'single');
    P_out = P;
    return;
end
cinv = inv(C);
sinv = cinv * cinv';
K = T * sinv; % Kalman gain
delta_state = single(K * residual);

F = (eye(size(P), 'like',P) - K * H);
% Update covariance
P_out = cast(F * P * F' + K * R_mat *  K', 'like',P); % Ensure this is the same data type as P

% Ensure that the covariance stays semipositive definite and symmetric.
P_out = 0.5 * (P_out + P_out');% + 1e-7 * eye(size(P_out));

end

function [P_out] = fix_covariance(P)
% force all eigenvalues to be positive
[Vc, Dc] = eig(P);
V = real(Vc);
D = real(Dc);
for i = 1:size(D, 1)
   if (D(i, i) <= 1e-6)
       D(i, i) = 1e-6;
    end
end
% recompose matrix from positive eigenvalues
P_out = V * D * V';
end
