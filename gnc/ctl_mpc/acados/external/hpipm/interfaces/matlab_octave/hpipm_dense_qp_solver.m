%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                                                 %
% This file is part of HPIPM.                                                                     %
%                                                                                                 %
% HPIPM -- High-Performance Interior Point Method.                                                %
% Copyright (C) 2019 by Gianluca Frison.                                                          %
% Developed at IMTEK (University of Freiburg) under the supervision of Moritz Diehl.              %
% All rights reserved.                                                                            %
%                                                                                                 %
% The 2-Clause BSD License                                                                        %
%                                                                                                 %
% Redistribution and use in source and binary forms, with or without                              %
% modification, are permitted provided that the following conditions are met:                     %
%                                                                                                 %
% 1. Redistributions of source code must retain the above copyright notice, this                  %
%    list of conditions and the following disclaimer.                                             %
% 2. Redistributions in binary form must reproduce the above copyright notice,                    %
%    this list of conditions and the following disclaimer in the documentation                    %
%    and/or other materials provided with the distribution.                                       %
%                                                                                                 %
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND                 %
% ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED                   %
% WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE                          %
% DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR                 %
% ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES                  %
% (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;                    %
% LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND                     %
% ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT                      %
% (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS                   %
% SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                                    %
%                                                                                                 %
% Author: Gianluca Frison, gianluca.frison (at) imtek.uni-freiburg.de                             %
%                                                                                                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef hpipm_dense_qp_solver < handle
	
	properties
		C_dim
		C_arg
		C_ws
		time_ext
	end

	methods

		function obj = hpipm_dense_qp_solver(dim, arg)
			obj.C_dim = dim.C_dim;
			obj.C_arg = arg.C_arg;
			% create struct in C
			obj.C_ws = dense_qp_solver_create(obj.C_dim, obj.C_arg);
		end

		function solve(obj, qp, sol)
			obj.time_ext = dense_qp_solver_solve(qp.C_qp, sol.C_sol, obj.C_arg, obj.C_ws);
		end

		function set(obj, field, value)
			if(strcmp(field, 'iter_max') || strcmp(field, 'tol_stat') || strcmp(field, 'tol_eq') || strcmp(field, 'tol_ineq') || strcmp(field, 'tol_comp') || strcmp(field, 'mu0') || strcmp(field, 'reg_prim'))
				dense_qp_solver_arg_set(obj.C_arg, field, value);
			else
				disp(['hpipm_dense_qp_solver.set: wrong field: ', field]);
				keyboard;
			end
		end

		function value = get(varargin)
			obj = varargin{1};
			field = varargin{2};
            tokens = strsplit(field, '_');
			if(strcmp(field, 'time_ext'))
				value = obj.time_ext;
			else
				value = dense_qp_solver_get(obj.C_ws, field);
			end
		end

%		function print_C_struct(obj)
%			dense_qp_print(obj.C_dim, obj.C_qp);
%		end

		function delete(obj)
			%disp('in destructor');
			dense_qp_solver_destroy(obj.C_ws);
		end

	end
end



