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

% Dependencies:
% Requires astrobee_init and/or any prototype _init files to have run
% Run before astrobee_prep

clear tun_*
lua_config_file_name = which('lua_config');
if ~isempty(lua_config_file_name) % if the lua config file exists delete it
    delete(lua_config_file_name);
end

filename = [ASTROBEE_ROOT, '/luaconfig/export_config.lua'];
% generate lua_config.m with either lua or luajit
setenv('LUA_PATH', [ASTROBEE_ROOT, '/../../astrobee/config/?.config'])
setenv('ASTROBEE_CONFIG_VERSION', astrobee_version)
setenv('ASTROBEE_CONFIG_ROBOT', [ASTROBEE_ROOT '/../../astrobee/config/robots/sim.config']);
setenv('ASTROBEE_CONFIG_DIR', [ASTROBEE_ROOT '/../../astrobee/config']);
setenv('ASTROBEE_RESOURCE_DIR', [ASTROBEE_ROOT '/../../astrobee/resources']);
setenv('ASTROBEE_ROBOT', 'sim');
setenv('ASTROBEE_WORLD', 'granite');

[status, ~] = system('lua -v');
if status ~= 0
  luaVer = 'luajit';
else
    luaVer = 'lua';
end

[status, result] = system([luaVer,' ', filename]);
if status ~= 0
  disp(result);
  assert(false);
end
run('lua_config'); % run generated file

