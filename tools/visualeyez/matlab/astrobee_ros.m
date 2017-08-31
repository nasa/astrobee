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

% Fixes R2014a on memric
restoredefaultpath;
addpath('./tcp_udp_ip');
addpath('./vz');

%%%%%%%%%%%%%%
% PARAMETERS %
%%%%%%%%%%%%%%

rate = 100;           % Tracking rate
mmax = 512;           % Maximum number of trackable markers
ncol = 5;             % Number of measurement columns [tcm, led, x, y, z]

%%%%%%%%%%%%%%
%  TRACKING  %
%%%%%%%%%%%%%%
udp  = pnet('udpsocket', 9091);
offs = datenum('1970-1-1 00:00:00');
% Tracking loop
while true
  % Pause between measurements
  pause(1/rate);
  % Get the raw data from the VZ system
  raw_data = VzGetDat;
  % Get the number of measurements
  n = size(raw_data, 1);
  % Get the time stamp
  t = (datenum(clock)-offs)*24*3600.0;
  % Only send the data if we actually have some to send
  if (n > 0 && n <= mmax)
    % Setup the final data,: append a time stamp and convert to vector
    final_data = [t, reshape(raw_data(:,1:ncol)', 1, ncol*n)];
    % Write the data to the UDP pipe
    pnet(udp,'write', double(final_data), 'native');
    pnet(udp,'writepacket', 'spheresgoat.ndc.nasa.gov', 9090);
  end
end
pnet(udp,'close');