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

function TMDataOut = timeRange(telemDataObj, minTime, maxTime)
TMDataOut = telemDataObj;
validIndx = TMDataOut.time >= minTime & TMDataOut.time <= maxTime;
TMDataOut.time = TMDataOut.time(validIndx);
switch ndims(TMDataOut.data)
    case 2
        TMDataOut.data = TMDataOut.data(validIndx,:);
    case 3 
        TMDataOut.data = TMDataOut.data(:,:,validIndx);
    otherwise
        TMDataOut.data = TMDataOut.data(validIndx,:);
end
