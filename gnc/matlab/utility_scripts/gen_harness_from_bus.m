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

% Create a simulink bus creator with zeroes hooked up to it in the base workspace 
% from a simulink bus definition.
% Input: 'bus_name'  -- Name of a bus object in the workspace
function gen_harness_from_bus(busName)
%busName = 'cmd_msg';

busIn = evalin('base', busName);
nInputs = length(busIn.Elements);

modelName = [busName '_bus_creator'];

sys_h = new_system(modelName);

%'Position', When I add blocks to the canvas, I specify the position to provide proper layout. The position parameter provides the top left (x,y) and lower right (x+w,y+h) 
% corners of the block. The x and y values are relative to the origin (0,0) in the upper left corner of the canvas; x increases to the right, and y increases down. 
const.height = 20;
const.width = 150;
const.x = 0;
% const.y = 110;

constantCushion = 25; % Space between constants
constantSpacing = const.height + constantCushion; % Top to top spacing
topBottomCushion = 10; % # of pixels above and below first and last constant

% X-Y coords are top left
creator.x = 320;
creator.y = 55;
creator.width = 5; 
creator.height = topBottomCushion*2 + nInputs*const.height + (nInputs-1)*constantCushion;

busCreator_h = add_block('simulink/Signal Routing/Bus Creator', [modelName '/bus_creator']);
set_param(busCreator_h, 'Inputs', num2str(nInputs)); % Set the number of inputs
set_param(busCreator_h, 'Position', [creator.x creator.y creator.x+creator.width creator.y+creator.height]) % Se the position and dimensions
set_param(busCreator_h, 'OutDataTypeStr', ['Bus: ' busName]);
set_param(busCreator_h, 'NonVirtualBus', 'on');


for ii = 1:nInputs
    constY = creator.y+topBottomCushion+(ii-1)*constantSpacing; % Y location of the top left corner of the constant block
    constant_h = add_block('simulink/Sources/Constant', [modelName '/constant' num2str(ii)], 'Position', [const.x constY const.x+const.width constY+const.height]); % Create constant
    
    if length(busIn.Elements(ii).Dimensions) == 1
        set(constant_h, 'Value', [ '[' num2str(zeros(1,busIn.Elements(ii).Dimensions)) ']']); % Initialize the block to zeroes
    else
        set(constant_h, 'Value', ['zeros([' num2str(busIn.Elements(ii).Dimensions) '])' ]); % Initialize the block to zeroes
    end
    set(constant_h, 'OutDataTypeStr', busIn.Elements(ii).DataType)
    set(constant_h, 'ShowName', 'off') % disable the name from showing

    
    line_h = add_line(sys_h, ['constant' num2str(ii) '/1'],['bus_creator/' num2str(ii)] ); % connect the constant to the bus creator
    set(line_h, 'Name', busIn.Elements(ii).Name)
end

open_system(sys_h)


