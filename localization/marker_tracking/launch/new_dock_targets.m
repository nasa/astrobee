% Computes the locations of the AR targets on the dock in World
% frame from measurements performed in the plane of the dock
%
% AR targets are currently caraterized by the positions of three
% corners. The naming refering the a coordinate system attached
% to the target itself if looking at it with its label being
% readable:
%   - top left
%   - top right
%   - bottom left
%
%
% * corners_plane: contains the coordinates of the the AR markers (3
%   corners for each) expressed in the printing plane
% * corners_world: computed coordinate of the AR markers 3 corners
%   expressed in world frame (Z down)
%

in2mm = 25.4;
mm2m = 0.001;

% dock specifications
postAngle = deg2rad(25);

% Coordinates in dock plane:
% - Z out of the docking
% - X to the right, when facing the dock
% - Y to the dock top (post -> charger)

% list of all targets expressed in mm
ar21 = [ [71, 48, 0]; [91, 48, 0]; [71, 28, 0] ];
ar22 = [ [-91, -28, 0]; [-71, -28, 0]; [-91, -48, 0] ];
ar23 = [ [151, -15, 0]; [171, -15, 0]; [151, -35, 0] ];
ar24 = [ [-171, 35, 0]; [-151, 35, 0]; [-171, 15, 0] ];

ar41 = [ [91, -15, 0]; [131, -15, 0]; [91, -55, 0] ];
ar42 = [ [-131, 55, 0]; [-91, 55, 0]; [-131, 15, 0] ];
ar43 = [ [131, 48, 0]; [171, 48, 0]; [131, 8, 0] ];
ar44 = [ [-171, -8, 0]; [-131, -8, 0]; [-171, -48, 0] ];

ar96 = [ [-48, 48, 0]; [48, 48, 0]; [-48, -48, 0] ];

corners_plane = [ ...
            ar21(1,:); ar21(2,:); ar21(3,:);...
            ar22(1,:); ar22(2,:); ar22(3,:);...
            ar23(1,:); ar23(2,:); ar23(3,:);...
            ar24(1,:); ar24(2,:); ar24(3,:);...
            ar41(1,:); ar41(2,:); ar41(3,:);...
            ar42(1,:); ar42(2,:); ar42(3,:);...
            ar43(1,:); ar43(2,:); ar43(3,:);...
            ar44(1,:); ar44(2,:); ar44(3,:);...
            ar96(1,:); ar96(2,:); ar96(3,:)...
            ];

ar_ids = [ 21, 22, 23, 24, 41, 42, 43, 44, 96 ];
corner_names = { 'topleft', 'topright', 'bottomleft' } ;

% Coordinates expressed in world frame, expressed in meters
% relative to the center of the target pattern,
% if the dock was vertical (=in the same plane than
% the post contact surface)
Rplane2zdown = mm2m * [...
        [ 0, 0, 1 ]; ...
        [ -1, 0, 0]; ...
        [ 0, -1, 0] ...
    ];
    
corners_zdown = zeros(size(corners_plane));
for n = 1:size(corners_plane,1)
    corners_zdown(n,:) = Rplane2zdown * corners_plane(n,:)';
end


% Rotation from the post contact plane to the 
% dock surface
Rpost2dock = [...
    [ cos(postAngle), 0, sin(postAngle) ];...
    [ 0, 1, 0 ];...
    [ -sin(postAngle), 0, cos(postAngle) ]...
    ];

% Translation from the center of the granite table to the
% center point between the 2 docks
Tworld2arcenter = [ -0.7053    0.3105   -0.8378]

corners_world = zeros(size(corners_plane));
for n = 1:size(corners_plane,1)
    corners_world(n,:) = Rpost2dock * corners_zdown(n,:)' ...
        + Tworld2arcenter';
end

index = 1;
for i=1:size(ar_ids,2)
    fprintf('  <ar id="%d"\n', ar_ids(i))
    for n=1:size(corner_names,2)
        fprintf('    %s="%.3f %.3f %.3f"\n', ...
            corner_names{n}, ...
            corners_world(index, 1), ...
            corners_world(index, 2), ...
            corners_world(index, 3) )
        index = index + 1;
    end
    fprintf('   />\n')
end
