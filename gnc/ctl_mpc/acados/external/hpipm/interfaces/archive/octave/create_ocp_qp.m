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

function qp = create_ocp_qp( dims )
% qp

N = dims.N;
nx = dims.nx;
nu = dims.nu;
nbx = dims.nbx;
nbu = dims.nbu;
ng = dims.ng;
ns = dims.ns;

%
qp.A = {};
for ii=1:N
	qp.A{ii} = zeros(nx(ii+1), nx(ii));
end
%
qp.B = {};
for ii=1:N
	qp.B{ii} = zeros(nx(ii+1), nu(ii));
end
%
qp.b = {};
for ii=1:N
	qp.b{ii} = zeros(nx(ii+1), 1);
end
%
qp.Q = {};
for ii=1:N+1
	qp.Q{ii} = zeros(nx(ii), nx(ii));
end
%
qp.R = {};
for ii=1:N+1
	qp.R{ii} = zeros(nu(ii), nu(ii));
end
%
qp.S = {};
for ii=1:N+1
	qp.S{ii} = zeros(nu(ii), nx(ii));
end
%
qp.q = {};
for ii=1:N+1
	qp.q{ii} = zeros(nx(ii), 1);
end
%
qp.r = {};
for ii=1:N+1
	qp.r{ii} = zeros(nu(ii), 1);
end
%
qp.Jx = {};
for ii=1:N+1
	qp.Jx{ii} = zeros(nbx(ii), nx(ii));
end
%
qp.lx = {};
for ii=1:N+1
	qp.lx{ii} = zeros(nbx(ii), 1);
end
%
qp.ux = {};
for ii=1:N+1
	qp.ux{ii} = zeros(nbx(ii), 1);
end
%
qp.Ju = {};
for ii=1:N+1
	qp.Ju{ii} = zeros(nbu(ii), nu(ii));
end
%
qp.lu = {};
for ii=1:N+1
	qp.lu{ii} = zeros(nbu(ii), 1);
end
%
qp.uu = {};
for ii=1:N+1
	qp.uu{ii} = zeros(nbu(ii), 1);
end
%
qp.C = {};
for ii=1:N+1
	qp.C{ii} = zeros(ng(ii), nx(ii));
end
%
qp.D = {};
for ii=1:N+1
	qp.D{ii} = zeros(ng(ii), nu(ii));
end
%
qp.lg = {};
for ii=1:N+1
	qp.lg{ii} = zeros(ng(ii), 1);
end
%
qp.ug = {};
for ii=1:N+1
	qp.ug{ii} = zeros(ng(ii), 1);
end
%
qp.Zl = {};
for ii=1:N+1
	qp.Zl{ii} = zeros(ns(ii), ns(ii));
end
%
qp.Zu = {};
for ii=1:N+1
	qp.Zu{ii} = zeros(ns(ii), ns(ii));
end
%
qp.zl = {};
for ii=1:N+1
	qp.zl{ii} = zeros(ns(ii), 1);
end
%
qp.zu = {};
for ii=1:N+1
	qp.zu{ii} = zeros(ns(ii), 1);
end
%
qp.Jsbx = {};
for ii=1:N+1
	qp.Jsbx{ii} = zeros(nbx(ii), ns(ii));
end
%
qp.Jsbu = {};
for ii=1:N+1
	qp.Jsbu{ii} = zeros(nbu(ii), ns(ii));
end
%
qp.Jsg = {};
for ii=1:N+1
	qp.Jsg{ii} = zeros(ng(ii), ns(ii));
end
%
qp.lls = {};
for ii=1:N+1
	qp.lls{ii} = zeros(ns(ii), 1);
end
%
qp.lus = {};
for ii=1:N+1
	qp.lus{ii} = zeros(ns(ii), 1);
end

return;
