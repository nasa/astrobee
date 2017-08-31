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

classdef telemData % telemData is a subclass

    
    methods
        % Constructor
        function obj = telemData(T,Y,varargin)
           switch nargin % Need to support the constructor with 0 arguments for some reason
               case 0
                obj.time = [];
                obj.data = [];
               case 1
                   if isa(T, 'timeseries') % if the first imput is a timeseries data type, unpack it
                       obj.time = T.Time;
                       if size(T.Data , 2) == 4 && ismatrix(T.Data) % If it looks like a quaternion time vector
                           obj.data = quaternion(T.Data);
                       else
                           obj.data = T.Data;
                       end
                   else
                       error('Illegal arguments');
                   end
               case 2
                obj.time = T;
                obj.data = Y;
               otherwise
                error('Illegal number of arguments');
           end
        end
        
        % Whats this?
        function dataOut = isempty(dataIn)
            dataOut = builtin('isempty',dataIn);
        end
            
        function y = uminus(obj)
            y = obj;
            y.data = uminus(y.data);
        end
        
        function y = convert_rad2deg(obj)
            y = obj;
            y.data = y.data*180/pi;
        end
        
        function y = mag(obj)
            y = obj;
            y.data = rssrow(y.data);
        end
        
        function y = max(obj)
            y = max(obj.data);
        end
        
        function y = maxerror(obj)
            [~, idx] = max(abs(obj.data), [], 1);
            y = obj.data(sub2ind(size(obj.data), idx, 1:size(obj.data, 2)));
        end
        
        function y = min(obj)
            y = min(obj.data);
        end
        
        function y = mean(obj)
            y = mean(obj.data);
        end
        
        function y = std(obj)
            y = std(obj.data);
        end
        
        function y = mean3sig(obj)
            y = abs(mean(obj.data)) + 3*std(obj.data);
        end
        
        function y = fprintf(varargin)
            td_indx = find(cellfun(@(a) isa(a, 'telemData'), varargin));
            for ii = 1:length(td_indx)
                varargin{td_indx(ii)} = varargin{td_indx(ii)}.data;
            end
            y = fprintf(varargin{:});
        end
        
    end
    % Properties
    properties
        data;
        time;
        masterTime = false;
        units;
        legendName;
        
    end
    
end