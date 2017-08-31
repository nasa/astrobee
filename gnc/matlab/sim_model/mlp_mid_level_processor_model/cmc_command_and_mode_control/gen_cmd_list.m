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

function [cmd_list, cmd_time] = gen_cmd_list(path)


cmd_start_time  = 30; %time of the first cmd
min_stop        = 10; %minimum stop time at a station?

fid     = fopen(path,'r');
text    = textscan(fid,'%s','Delimiter','');
text    = text{1};
fclose(fid);

station_idx = find(~cellfun('isempty',strfind(text,'"type" : "Station"')));

% If you can't find any stations try looking for the new name of Stations
% found in the newer versions of GDS
if isempty(station_idx)
    station_idx = find(~cellfun('isempty',strfind(text,'"type" : "ModuleBayStation"')));
end

segment_idx = find(~cellfun('isempty',strfind(text,'"type" : "Segment"')));
all_idx     = sort([station_idx; segment_idx]);

curr_cmd_idx = 0;
for i = 1:length(all_idx)
    %if the indexed line contaions station, handle as station
    if(~cellfun('isempty', strfind(text(all_idx(i)), 'Station')))
%         y_val           = sscanf(cell2mat(text(all_idx(i)+3)), '"y" : %g*');
%         x_val           = sscanf(cell2mat(text(all_idx(i)+4)), '"x" : %g*');
%         q_val           = sscanf(cell2mat(text(all_idx(i)+5)), '"pitch" : %g*');
%         p_val           = sscanf(cell2mat(text(all_idx(i)+6)), '"roll" : %g*');
%         r_val           = sscanf(cell2mat(text(all_idx(i)+7)), '"yaw" : %g*');
%         z_val           = sscanf(cell2mat(text(all_idx(i)+8)), '"z" : %g*');
%         quat_val        = eulers_to_quat(r_val,q_val,p_val);
%         
%         curr_cmd_idx = curr_cmd_idx + 1;
%         cmd_list(curr_cmd_idx,:) = [x_val, y_val, z_val, zeros(1,3), zeros(1,3), quat_val, zeros(1,3), zeros(1,3)];
%         

        %the idea here is that stations are useless except as placeholders for the last command in a segment.  
        %So just capture the 'duration' time command.  Then add that to the begining of the next segment
        if (curr_cmd_idx>0)
            duration = sscanf(cell2mat(text(all_idx(i)+18)), '"duration" : %g*');
            if(~isempty(duration))
                next_time = time_list(curr_cmd_idx) + duration;
            else
                next_time = time_list(curr_cmd_idx) + min_stop;
            end
        else
            next_time = cmd_start_time;
        end
    else %it was a segment
        is_waypts       = 0;    %index of where the waypoint line is relative to "Segment"
        while(cellfun('isempty', strfind(text(all_idx(i)+is_waypts), '"waypoints"')))
            is_waypts = is_waypts+1;
        end
            
        waypts          = strsplit(cell2mat(text(all_idx(i)+is_waypts)), '[');
        waypts          = waypts(3:end);
        for j = 1:length(waypts);
            temp = str2double(strsplit(cell2mat(waypts(j)), {',', '[', ']', ' ','\f','\n','\r','\t','\v'}));
            
            curr_cmd_idx = curr_cmd_idx + 1;
            cmd_list(curr_cmd_idx,:) = temp(3:end-1);
            if(j==1)
                %if the first waypt of a segment, add on the time from the previous station, 
                %and then capture the relative time for the rest of the waypoints
                time_list(curr_cmd_idx) = next_time + temp(2);
                first_time = time_list(curr_cmd_idx);
            else
                %otherwise, just handle the relative timing
                time_list(curr_cmd_idx) = first_time + temp(2);
            end
        end
    end
end

cmd_time = uint32([floor(time_list)', floor((time_list - floor(time_list))*1e9)']);

cmd_list = single(cmd_list);


