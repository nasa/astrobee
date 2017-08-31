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

function [num,den] = first_ord_butter(omega_c,Ts)
%Calculates the filter coefficients for a first order butter worth low pass
%filter.
%   Calculates the filter coefficients for a first order Butterworth low pass
%filter using the bilinear transformation of the equation:
%omega_c/[s+omega_c]

filter_const = omega_c*Ts/2;
num = single( [1 1]*filter_const/(1+filter_const) );  
den = single( [1 ,  (filter_const-1)/(filter_const+1)] );

end

