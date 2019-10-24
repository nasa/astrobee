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

function varargout = plot(h,varargin)
%PLOT  
% Accepts a TelemData with standard plot options.
% Additoinal properties include:
% 't0' - specify a time to set as the zero time
% 'title' - Specify a title for the plot and window
% 'AbsTime' - Absolute Time 
%
% Examples:
% plot(telemData) 
% plot(telemData, 'property1Name', property1)
% plot(telemData, 'property1Name', property1, 't0', 122)

% See if a zero time has been specified
t0_indx = find((strcmpi(varargin, 't0')), 1,'first');
if ~isempty(t0_indx)
    h.time = h.time - varargin{t0_indx + 1};
    varargin(t0_indx:t0_indx+1) = [];
end

% See if a zero time has been specified
title_indx = find((strcmpi(varargin, 'title')), 1,'first');
if ~isempty(title_indx)
   plot_title = varargin{title_indx+1};
    varargin(title_indx:title_indx+1) = [];
end

abs_plot_indx = find((strcmpi(varargin, 'AbsTime')), 1,'first');
if ~isempty(abs_plot_indx)
    varargin(abs_plot_indx) = [];
    h.time = datetime(h.time, 'ConvertFrom', 'posixtime');
end



pHandle = plot(h.time, h.data, varargin{:});
grid on; 
if ~isempty(title_indx)
    title(plot_title);
    % Set the figure title
    r = gca;
    set(gcf, 'Name', r.Title.String)
end

if nargout>0
    varargout{1} = pHandle;
end


