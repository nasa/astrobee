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
% Unless required by applicable law or agreed to in writing, software distributed
% under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
% CONDITIONS OF ANY KIND, either express or implied. See the License for the
% specific language governing permissions and limitations under the License.

% Read in the data
[id,x,y,z] = textread('points_raw.csv','%s %f %f %f','delimiter',' ');
x = x / 1e3;
y = y / 1e3;
z = z / 1e3;

%{
1 : M01
2 : M02
3 : M03
4 : M04
5 : M05
6 : M06
7 : M07
8 : M08
9 : M09
10: M10
11: M11
12: M12

M01 M02 M03 = Z-X plane

%}

% Work out the rotation matrix that axis aligns the points
pts = [x,y,z]';
vy  = cross(pts(:,3) - pts(:,1), pts(:,2) - pts(:,1));
vy  = vy / norm(vy);
vx  = cross(pts(:,7) - pts(:,9), pts(:,6) - pts(:,9));
vx  = vx / norm(vx);
vz  = cross(vx, vy);
R = [vx, vy, vz]';
pts = R * pts;

% Rotate the points by 180 degrees to maintain RHS and move +Z towards floor
R = [-1  0  0;
      0  1  0;
      0  0 -1];
pts = R * pts;
         
% Move the origin
origin = [pts(1,5)/2 + pts(1,9)/2;
          pts(2,5);
          pts(3,9)/2 + pts(3,6)/2];
pts = pts - repmat(origin,1,size(pts,2));
          
% Plot the data
clf; hold on; grid on; axis equal;
for i = 1:length(id)
  col = 'k+';
  if (i < 5)
    col = 'r+';
  elseif (i < 13)
    col = 'b+';
  endif
  plot3(pts(1,i),pts(2,i),pts(3,i),col,'MarkerSize',10,'LineWidth',3);
end
text(pts(1,:)+0.1, pts(2,:)+0.1, pts(3,:)+0.1, id);
xlabel('X');
ylabel('Y');
zlabel('Z');


dlmwrite('points_world.csv', [[1:16]',pts'] , " ");
