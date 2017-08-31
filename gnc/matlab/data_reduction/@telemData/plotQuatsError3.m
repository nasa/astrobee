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

function plotQuatsError3(telemData1, telemData2, telemData3, titleStr, legend1, legend2,legend3, style2, style3, t0)
% Prep for plot function in the telem class
figure;%(5); clf;
% Combine calcs into analysis file 
R1 = convert_to_eulers( telemData1.data );
R2 = convert_to_eulers( telemData2.data );
R3 = convert_to_eulers( telemData3.data );



subplot(2,1,1);
plot(telemData1.time - t0, R1*180/pi)
hold_on;
plot(telemData2.time - t0, R2*180/pi, style2)
hold_on;
plot(telemData3.time - t0, R3*180/pi, style3)
title(titleStr); grid on;
ylabel('Euler Angles, Deg'); xlabel('seconds')
legend([legend1 '_\theta'], [legend1 '_\phi'], [legend1 '_\psi'], [legend2 '_\theta'], [legend2 '_\phi'], [legend2 '_\psi'],...
    [legend3 '_\theta'], [legend3 '_\phi'], [legend3 '_\psi']);

% Minus applied to quaternion calculates angle error 
angleError = telemData1 - telemData2;
subplot(2,1,2);
plot(angleError.time - t0, angleError.data*180/pi);
title([titleStr ' Error (' legend1 ' to ' legend2 ')']); grid on;
ylabel('Deg'); xlabel('seconds')
