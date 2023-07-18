% Matlab file to replace env.sh to allow compile_mex_all.m to run on Windows
% Len van Moorsel
% 2021

%% set environment variables, that are originally set in env.sh

% set paths to folders used by HPIPM
setenv('HPIPM_MAIN_FOLDER',fileparts(fileparts(pwd))); % "$(pwd)/../.."
disp(['  set HPIPM_MAIN_FOLDER to: ', getenv('HPIPM_MAIN_FOLDER')])

setenv('BLASFEO_MAIN_FOLDER',[fileparts(fileparts(fileparts(pwd))),'/blasfeo']); % "$(pwd)/../../../blasfeo"
disp(['  set BLASFEO_MAIN_FOLDER to: ', getenv('BLASFEO_MAIN_FOLDER')])

matlabpath = [getenv('HPIPM_MAIN_FOLDER'),'/interfaces/matlab_octave/']; % $HPIPM_MAIN_FOLDER/interfaces/matlab_octave/
if (getenv('MATLABPATH') == "") % if empty, set path
    setenv('MATLABPATH', matlabpath);
    disp(['  set MATLABPATH to: ', getenv('MATLABPATH')])
else % else, append path
    setenv('MATLABPATH',[getenv('MATLABPATH'),';',matlabpath]);
    disp(['  appended MATLABPATH to: ', getenv('MATLABPATH')])
end

octave_path = [getenv('HPIPM_MAIN_FOLDER'),'/interfaces/matlab_octave/']; % $HPIPM_MAIN_FOLDER/interfaces/matlab_octave/
if (getenv('OCTAVE_PATH') == "") % if empty, set path
    setenv('OCTAVE_PATH', octave_path);
    disp(['  set OCTAVE_PATH to: ', getenv('OCTAVE_PATH')])
else % else, append path
    setenv('OCTAVE_PATH',[getenv('OCTAVE_PATH'),';',octave_path]);
    disp(['  appended OCTAVE_PATH to: ', getenv('OCTAVE_PATH')])
end

ld_library_path = [[getenv('HPIPM_MAIN_FOLDER'),'/lib/'],';',[getenv('BLASFEO_MAIN_FOLDER'),'/lib/']]; % $HPIPM_MAIN_FOLDER/lib:$BLASFEO_MAIN_FOLDER/lib
if (getenv('LD_LIBRARY_PATH') == "") % if empty, set path
    setenv('LD_LIBRARY_PATH', ld_library_path);
    disp(['  set LD_LIBRARY_PATH to: ', getenv('LD_LIBRARY_PATH')])
else % else, append path
    setenv('LD_LIBRARY_PATH',[getenv('LD_LIBRARY_PATH'),';',ld_library_path]);
    disp(['  appended LD_LIBRARY_PATH to: ', getenv('LD_LIBRARY_PATH')])
end

% set environment run variable to true to indicate this script was run
setenv('ENV_RUN','true')