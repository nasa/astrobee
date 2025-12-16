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

function compile_mex_one_from_script(varargin)

file = varargin{1};

% check that env.sh has been run
env_run = getenv('ENV_RUN');
if (~strcmp(env_run, 'true'))
	disp('ERROR: env.sh has not been sourced! Before executing this example, run:');
	disp('source env.sh');
	return;
end

% get folders
hpipm_folder = getenv('HPIPM_MAIN_FOLDER');
blasfeo_folder = getenv('BLASFEO_MAIN_FOLDER');
mex_flags = getenv('HPIPM_MEX_FLAGS');
data_folder = getenv('DATA_FOLDER');

if(length(hpipm_folder)==0)
	disp('ERROR: the environment variable HPIPM_MAIN_FOLDER has not been specified.');
	return;
end
if(length(blasfeo_folder)==0)
	disp('ERROR: the environment variable BLASFEO_MAIN_FOLDER has not been specified.');
	return;
end
if(length(data_folder)==0)
	disp('ERROR: the environment variable DATA_FOLDER has not been specified.');
	return;
end

% set paths
hpipm_mex_folder = [hpipm_folder, '/interfaces/matlab_octave/'];
hpipm_include = ['-I', hpipm_folder, '/include'];
hpipm_lib = ['-L', hpipm_folder, '/lib'];
blasfeo_include = ['-I', blasfeo_folder, '/include'];
blasfeo_lib = ['-L', blasfeo_folder, '/lib'];
data_include = ['-I', data_folder];

cflags_octave_file = [hpipm_mex_folder, 'cflags_octave.txt'];

if is_octave()
	if exist(cflags_octave_file)==0
		diary cflags_octave_file
		diary on
		mkoctfile -p CFLAGS
		diary off
		input_file = fopen(cflags_octave_file, 'r');
		cflags_tmp = fscanf(input_file, '%[^\n]s');
		fclose(input_file);
		cflags_tmp = [cflags_tmp, ' -std=c99'];
		input_file = fopen(cflags_octave_file, 'w');
		fprintf(input_file, '%s', cflags_tmp);
		fclose(input_file);
	end
	input_file = fopen(cflags_octave_file, 'r');
	cflags_tmp = fscanf(input_file, '%[^\n]s');
	if nargin>1
		cflags_tmp = [cflags_tmp, varargin{2}];
	end
	fclose(input_file);
	setenv('CFLAGS', cflags_tmp);
end

% compile mex
mex_file = [hpipm_mex_folder, file];

%disp(['compiling ', mex_file])
if is_octave()
%	mkoctfile -p CFLAGS
	mex(hpipm_include, blasfeo_include, hpipm_lib, blasfeo_lib, '-lhpipm', '-lblasfeo', mex_file);
else
	if nargin>1
		mex(mex_flags, ['CFLAGS=\$CFLAGS -std=c99', varargin{2}], hpipm_include, blasfeo_include, hpipm_lib, blasfeo_lib, data_include, '-lhpipm', '-lblasfeo', mex_file);
	else
		mex(mex_flags, 'CFLAGS=\$CFLAGS -std=c99', hpipm_include, blasfeo_include, hpipm_lib, blasfeo_lib, data_include, '-lhpipm', '-lblasfeo', mex_file);
	end
end

return;


