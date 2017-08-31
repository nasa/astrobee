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

function [points] = gen_points_AR_tags(origin, norm_vec)
%returns a set of AR tag points relative to a center point
% note: the AR tags are grouped in 4s.  The first 4 are 1 AR tag, the second 4 are a different AR tag
  
%find two vectors in the plane 
if(acosd(abs(dot(norm_vec, [1 0 0]))) > 10)
    plane_vec1 = cross(norm_vec,[1 0 0]);
else
    plane_vec1 = cross(norm_vec,[0 1 0]);
end
plane_vec2 = cross(norm_vec, plane_vec1);

%location relative to center, in plane defined by vector
%each Tag is represented by (:,:,x) in the matrix.  Whole tag must be visible to report.  
AR_P_tag_origin_2D(:,:,1) = [-.1 -.1; -.2 -.2; -.1 -.2; -.2 -.1];
AR_P_tag_origin_2D(:,:,2) = [.1 .1; .2 .2; .1 .2; .2 .1];
AR_P_tag_origin_2D(:,:,3) = [-.1 .1; -.2 .2; -.1 .2; -.2 .1];
AR_P_tag_origin_2D(:,:,4) = [.1 -.1; .2 -.2; .1 -.2; .2 -.1];

points = [];
AR_P_tag_origin_3D        = zeros(4,3,size(AR_P_tag_origin_2D,1));  %[4 corners per tag, 3D position, number of tags]
for i = 1:size(AR_P_tag_origin_2D,3)
    for j = 1:4
        AR_P_tag_origin_3D(j,:,i) = AR_P_tag_origin_2D(j,1,i)*plane_vec1 + AR_P_tag_origin_2D(j,2,i)*plane_vec2 + origin;
    end
    points = [points; AR_P_tag_origin_3D(:,:,i)];
end

points = single(points);
