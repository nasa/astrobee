function make_daqp(varargin)
% Matlab makefile for DAQP.
%
%    make_daqp: build all components (library and mex) from source and link.
%    make_daqp('lib'): builds the DAQP library using CMake
%    make_daqp('mex'): builds the DAQP mex interface and links it to the library 


if( nargin == 0 )
    what = '';
else
    what = varargin{nargin};
	if(	isempty(strfind(what, 'lib'))   && ...
		isempty(strfind(what, 'mex'))   && ...
		isempty(strfind(what, 'package'))) 
            fprintf('"%s" is not a valid command\n', what);
			return
    end
end

% clear daqpmex from memory 
if(mislocked('daqpmex'))
    munlock('daqpmex');
end

%% Setup directories
start_dir= pwd;
% Set src directory and build directory
[daqp_matlab_dir,~,~] = fileparts(which('make_daqp.m'));
daqp_dir = fullfile(daqp_matlab_dir, '../..');
daqp_build_dir = fullfile(daqp_dir, 'build');

% Include directory
inc_dir = [
    fullfile(sprintf(' -I%s', daqp_dir), 'include'),...
    fullfile(sprintf(' -I%s', daqp_dir), 'codegen')];

%% Compiler commands and arguments 
% Get make and mex commands
make_cmd = 'cmake --build .';
mex_cmd = sprintf('mex -O -silent');
mex_libs = '';


% Add arguments to cmake and mex compiler
cmake_args = '-DMATLAB=True';
mexoptflags = '-DMATLAB';

% Add specific generators for windows linux or mac
if (ispc)
    cmake_args = sprintf('%s %s', cmake_args, '-G "MinGW Makefiles"');
else
    cmake_args = sprintf('%s %s', cmake_args, '-G "Unix Makefiles"');
end

% Pass Matlab root to cmake
Matlab_ROOT = strrep(matlabroot, '\', '/');
cmake_args = sprintf('%s %s%s%s', cmake_args, ...
    '-DMatlab_ROOT_DIR="', Matlab_ROOT, '"');

% Add parameters options to mex and cmake
if (ispc)
   ut = fullfile(matlabroot, 'extern', 'lib', computer('arch'), ...
                 'mingw64', 'libut.lib');
   mex_libs = sprintf('%s "%s"', mex_libs, ut);
else
   mex_libs = sprintf('%s %s', mex_libs, '-lut');
end
% Shared library loading
if (isunix && ~ismac)
   mex_libs = sprintf('%s %s', mex_libs, '-ldl');
end

% Large arrays 
if (~isempty(strfind(computer, '64')) && verLessThan('matlab', '9.4'))
    mexoptflags = sprintf('%s %s', mexoptflags, '-largeArrayDims');
end

% Old-style usage of mxGetPr 
if ~verLessThan('matlab', '9.4')
    mexoptflags = sprintf('%s %s', mexoptflags, '-R2017b');
end

% Legacy stdio for MSVC (to avoid LNK2019 for fprintf)
if(ispc)
    mex_comp_configs = mex.getCompilerConfigurations('C')
    if(contains(mex_comp_configs.ShortName,'MSVC'))
        mex_libs = sprintf('%s %s', mex_libs, '-llegacy_stdio_definitions');
    end
end

% Set optimizer flag
if (~ispc)
    mexoptflags = sprintf('%s %s', mexoptflags, 'COPTIMFLAGS=''-O3''');
end

%% Compile library
if(any(strcmpi(what,'lib')) || isempty(what))
   fprintf('Compiling DAQP...\n');

    % Create build directory and go inside
    if exist(daqp_build_dir, 'dir')
        rmdir(daqp_build_dir, 's');
    end
    mkdir(daqp_build_dir);
    cd(daqp_build_dir);

    % Extend path for CMake mac (via Homebrew)
    PATH = getenv('PATH');
    if ((ismac) && (isempty(strfind(PATH, '/usr/local/bin'))))
        setenv('PATH', [PATH ':/usr/local/bin']);
    end

    % Compile static library with CMake
    [status, output] = system(sprintf('%s %s ..', 'cmake', cmake_args));
    if(status)
        disp(output);
        error('Error configuring CMake environment');
    end

    [status, output] = system(sprintf('%s %s', make_cmd, '--target daqpstat'));
    if (status)
        disp(output);
        error('Error compiling DAQP');
    end

    % Change directory back to matlab interface
    cd(daqp_matlab_dir);
end

%% Compile daqpmex 
if(any(strcmpi(what,'mex')) || isempty(what))
    % Compile interface
    fprintf('Compiling and linking daqpmex...\n');

    % Compile command
    lib_origin = fullfile(daqp_build_dir, 'libdaqpstat.a');
    cmd = sprintf('%s %s %s %s daqpmex.c %s', ...
        mex_cmd, mexoptflags, inc_dir, lib_origin, mex_libs);

    % Compile
    eval(cmd);
end

%% Package  
if(any(strcmpi(what,'package')))
  % Get platform 
  if ispc
	platform = 'windows';
  elseif ismac
	platform = 'mac';
  elseif isunix
	platform = 'linux';
  end

  % Setup directory and copy files
  pkg_name = sprintf('daqp-matlab-%s64', platform);
  if exist(pkg_name, 'dir')
	rmdir(pkg_name, 's');
  end
  mkdir(pkg_name);

  % Copy examples, test, and utils 
  folders = {'examples', 'test', 'utils'};
  for i = 1:length(folders)
	folder = folders{i};
	copyfile(fullfile(daqp_matlab_dir, folder), ...
	  fullfile(pkg_name, folder));
  end
  % Copy files
  files = {'daqp.m', ...
	'daqpmex.c', ...
	sprintf('daqpmex.%s', mexext), ...
	'runtest_daqp.m'};
  for i=1:length(files)
    file = files{i};
    copyfile(fullfile(daqp_matlab_dir, file), ...
  	fullfile(pkg_name, file));
  end
  
  % Copy license
  copyfile(fullfile(daqp_dir, 'LICENSE'), fullfile(pkg_name));

  % Create tarball
  tar(sprintf('%s.tar.gz', pkg_name), pkg_name);
  rmdir(pkg_name, 's');
end

%% Finalize 
cd(start_dir);
end
