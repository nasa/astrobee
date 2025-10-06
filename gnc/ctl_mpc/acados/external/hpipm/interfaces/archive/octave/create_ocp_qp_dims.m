%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                                                 %
% This file is part of HPIPM.                                                                     %
%                                                                                                 %
% HPIPM -- High Performance Interior Point Method.                                                %
% Copyright (C) 2017 by Gianluca Frison.                                                          %
% Developed at IMTEK (University of Freiburg) under the supervision of Moritz Diehl.              %
% All rights reserved.                                                                            %
%                                                                                                 %
% HPIPM is free software; you can redistribute it and/or                                          %
% modify it under the terms of the GNU Lesser General Public                                      %
% License as published by the Free Software Foundation; either                                    %
% version 2.1 of the License, or (at your option) any later version.                              %
%                                                                                                 %
% HPIPM is distributed in the hope that it will be useful,                                        %
% but WITHOUT ANY WARRANTY; without even the implied warranty of                                  %
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                                            %
% See the GNU Lesser General Public License for more details.                                     %
%                                                                                                 %
% You should have received a copy of the GNU Lesser General Public                                %
% License along with HPIPM; if not, write to the Free Software                                    %
% Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA                  %
%                                                                                                 %
% Author: Gianluca Frison, giaf (at) dtu.dk                                                       %
%                                                                                                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function dims = create_ocp_qp_dims( N )
% dims

% control horizon
dims.N = N;
% number of states
dims.nx = zeros(N+1, 1);
% number of inputs
dims.nu = zeros(N+1, 1);
% number of state box constraints
dims.nbx = zeros(N+1, 1);
% number of input box constraints
dims.nbu = zeros(N+1, 1);
% number of general constraints
dims.ng = zeros(N+1, 1);
% number of soft constraints
dims.ns = zeros(N+1, 1);

return;

