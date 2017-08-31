% Computes the locations of the AR targets on the dock in World
% frame from measurements performed in the plane of the dock
%
% units are mm up to the final output (m)
% ms_something = MultiscaleARtarget_something
% ar_something = RegularARtarget_something

in2mm = 25.4;
mm2m = 0.001;

postAngle = deg2rad(25);
ms_size = in2mm * [ 8.25, 6.75 ];
ar_size = 40;
ar_vert_offset = 22; % "vertical distance abetween the alum plate to the target
ms_margin = 6; % distance between the edge of the sheet and the printed target
post_to_plate = in2mm * (5-2.507); % from Omar CAD, center of post to alum plate
post_to_edge = in2mm * 5.25; % from Omar CADm center of post to right edge
%right_post_loc = [352, 1500, -585]'; % for now, referenced from the -X/-Y corner! Y not measured...
right_post_loc = [360-1000, 470, -585]'; % relative to the center of the granite table

% coordinates in the dock plane
ar_top = ar_vert_offset-post_to_plate;
ar_bot = ar_top+ar_size;
ar_right = -post_to_edge;
ar_left = ar_right+ar_size;

ms_bot = - (post_to_plate + ms_margin);
ms_top = ms_bot - ms_size(2);
ms_right = ms_margin - in2mm*0.25;
ms_left = ms_right + ms_size(1);

% coordinates in world if the dock was vertical
arD_top_left = [0, ar_right, ar_top]';
arD_top_right = [0, ar_right, ar_bot]';
arD_bot_left = [0, ar_left, ar_top]';

msD_top_left = [0, ms_right, ms_top]';
msD_top_right = [0, ms_right, ms_bot]';
msD_bot_left = [0, ms_left, ms_top]';

Rdock = [...
    [ cos(postAngle), 0, sin(postAngle) ];...
    [ 0, 1, 0 ];...
    [ -sin(postAngle), 0, cos(postAngle) ]...
    ];

arW_top_left = Rdock * arD_top_left + right_post_loc;
arW_top_right = Rdock * arD_top_right + right_post_loc;
arW_bot_left = Rdock * arD_bot_left + right_post_loc;

msW_top_left = Rdock * msD_top_left + right_post_loc;
msW_top_right = Rdock * msD_top_right + right_post_loc;
msW_bot_left = Rdock * msD_bot_left + right_post_loc;

% world coordinate in meters
ar_world = mm2m * [ arW_top_left'; arW_top_right'; arW_bot_left']
ms_world = mm2m * [ msW_top_left'; msW_top_right'; msW_bot_left']
