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
classdef hpipm_dense_qp_dim < handle
	
	properties
		C_dim
	end

	methods

%		function obj = hpipm_dense_qp_dim(in)
		function obj = hpipm_dense_qp_dim(varargin)
			if nargin==0
			% create dims struct in C
%			if isa(in, 'char')
%				% load entire dim from C data file
%				file = in;
%				compile_mex_one_from_script('dense_qp_dim_load.c', [' -DQP_DATA_H=', file]);
%				obj.C_dim = dense_qp_dim_load();
%			else
				% create empty dim with horizon N
				obj.C_dim = dense_qp_dim_create();
%			end
			else
				disp('hpipm_dense_qp_dim: wrong number of input arguments (0 allowed)');
				keyboard;
			end
		end

		function set(varargin)
			if nargin==3
				obj = varargin{1};
				field = varargin{2};
				value = varargin{3};
				dense_qp_dim_set(obj.C_dim, field, value);
			else
				disp('hpipm_dense_qp_dim.set: wrong number of input arguments (2 allowed)');
				keyboard;
			end
		end

%		function value = get(varargin)
%			if nargin==2
%				obj = varargin{1};
%				field = varargin{2};
%				value = dense_qp_dim_get(obj.C_dim, field, stage0);
%			else
%				disp('hpipm_dense_qp_dim.get: wrong number of input arguments (1 allowed)');
%			end
%		end

		function print_C_struct(obj)
			dense_qp_dim_print(obj.C_dim);
		end

%		function codegen(obj, file_name, mode)
%			dense_qp_dim_codegen(obj.C_dim, file_name, mode);
%		end

		function delete(obj)
			%disp('in destructor');
			dense_qp_dim_destroy(obj.C_dim);
		end

	end
end
