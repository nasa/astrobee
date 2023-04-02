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

classdef hpipm_dense_qp_solver_arg < handle
	
	properties
		C_dim
		C_arg
	end

	methods

		function obj = hpipm_dense_qp_solver_arg(dim, in)
			obj.C_dim = dim.C_dim;
			% create struct in C
%			if ( strcmp(in, 'speed_abs') || strcmp(in, 'speed') || strcmp(in, 'balance') || strcmp(in, 'robust') )
				% create empty arg with dimension dim
				mode = in;
				obj.C_arg = dense_qp_solver_arg_create(obj.C_dim, mode);
%			else
%				file = in;
%				compile_mex_one_from_script('dense_qp_solver_arg_load.c', [' -DQP_DATA_H=', file]);
%				obj.C_arg = dense_qp_solver_arg_load(obj.C_dim, file);
%			end
		end

		function set(obj, field, value)
			dense_qp_solver_arg_set(obj.C_arg, field, value);
		end

%		function print_C_struct(obj)
%			dense_qp_print(obj.C_dim, obj.C_qp);
%		end

		function codegen(obj, file_name, mode)
			dense_qp_solver_arg_codegen(obj.C_dim, obj.C_arg, file_name, mode);
		end

		function delete(obj)
			%disp('in destructor');
			dense_qp_solver_arg_destroy(obj.C_arg);
		end

	end
end


