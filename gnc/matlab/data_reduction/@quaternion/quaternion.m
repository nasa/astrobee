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

classdef quaternion % telemData is a subclass
    
    methods
        % Constructor
        function obj = quaternion(q)
            switch nargin % Need to support the constructor with 0 arguments for some reason
                case 0
                    obj.value = [];
                case 1
                    
                    obj.value = q;
                otherwise
                    error('Illegal number of arguments');
            end
        end
        

        function y = double(obj)
            y = double(obj.value);
        end
        
        % When matlab calls disp, return the value of the data property
        function disp(obj)
            obj.value
        end
            
        function angleOut = minus(q1, q2)
        % Returns angle between 2 quaternions (radians)
            angleOut = angle_error(q1, q2);
        end
        
        function qOut = mtimes(q1, q2)
            % Returns the product of quaternion multiply (q1*q2)
            qOut = quaternion(quatmult(q1.value, q2.value));
        end
        
        function qOut = uminus(qIn)
        % -q is quaternion inverse
            qOut = quaternion(qIn.inverse);
        
        end

        function qOut = abs(qIn)
            % Normalize and enforce positive scalar
            qOut = qIn.normalize_and_sign_check;
        end
        
        function n = numel(A,varargin)
            n = 1;%numel(A.value);
        end
        
        function [varargout] = subsref(A,S)
            if strcmp(S.type, '.')
                %varargout{1} = A.value;
                varargout{1} = eval(['A' S.type S.subs]);
            else
                varargout{1} = quaternion(subsref(A.value,S));
            end
        end
        
        % Whats this?
        function dataOut = isempty(dataIn)
            dataOut = builtin('isempty',dataIn);
        end
            
        
        
    end
    % Properties
    properties
        value;
    end
    
end