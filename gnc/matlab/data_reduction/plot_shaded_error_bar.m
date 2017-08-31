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

function plot_shaded_error_bar(x, y, error_values)


for ii = 1:size(y,2)
    
    colorIndx = get(gca, 'ColorOrderIndex');
    colorOrder = get(gca, 'ColorOrder');
    fill([x; flipud(x)], [y(:,ii) - error_values(:,ii); flipud(y(:,ii) + error_values(:,ii))], colorOrder(colorIndx,:), 'linestyle', 'none', 'FaceAlpha', 0.5);
    hold on;    
end

hold_on;
plot(x, y)