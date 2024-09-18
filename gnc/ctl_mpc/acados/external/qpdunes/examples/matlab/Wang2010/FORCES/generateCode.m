function success = generateCode(stages,params,settings,outvars)
% Generate a custom interior point solver for multistage problems using
% FORCES.
%
%    SUCCESS = GENERATECODE(STAGES) generates, downloads and compiles your
%    custom solver for the multistage problem STAGES. Default settings are
%    used, and the default parameter is 'stages(1).eq.c', i.e. the offset
%    term in the first equality constraint (as typically used in MPC). The
%    default output are simply all stage variables.
%
%    SUCCESS = GENERATECODE(STAGES,PARAMS) does the above but with user
%    defined parameters.
%
%    SUCCESS = GENERATECODE(STAGES,PARAMS,SETTINGS) does the above but with
%    user defined parameters and settings.
%
%    SUCCESS = GENERATECODE(STAGES,PARAMS,SETTINGS,OUTVARS) does the above 
%    but with user defined parameters, settings and outputs.
%
% SEE ALSO MULTISTAGEPROBLEM NEWPARAM NEWOUTPUT GETOPTIONS FORCES_LICENSE

if( nargin < 4 )
    % default output variables
    outvars = getAllOutputs(stages);
end
if( nargin < 3 )
    % default codegen options
    settings = getOptions;
end
if( nargin < 2 )
    % default parameter setting
    params = newParam('z0', 1, 'eq.c');
end

% copyright
fprintf('\n');
fprintf('FORCES - Fast interior point code generation for multistage problems.\n');
fprintf('Copyright (C) 2011-12 Alexander Domahidi [domahidi@control.ee.ethz.ch]\n');
fprintf('Automatic Control Laboratory, ETH Zurich.\n\n');

      
forcesurl = 'http://forces.ethz.ch';
createClassFromWsdl([forcesurl,'/CodeGen.asmx?Wsdl']);
obj = CodeGen;

% save structs in matfile
disp('Preparing data to be sent...');
[~,tmpname] = fileparts(tempname);
matfile = [tmpname,'.mat'];
save([tmpname,'.mat'],'stages','params','settings','outvars','-v6');

% read mat file as string stream
fid = fopen(matfile,'r');
byteStream = fread(fid);
fclose(fid);

% generate code
disp('Sending request to server...');
tic;
try
    zipurl = [forcesurl,generateCodeFromMatlab(obj, '5ddefdf2-2e58-4015-aab2-d9fb48c711a2', byteStream)];
catch err
    delete(matfile);
    throw(err);
end
t = toc;
fprintf('Code successfully generated in %4.2f sec. ',t);

% delete temporary file
delete(matfile);

% download code
disp('Downloading generated code...');
[~,file,ext] = fileparts(zipurl);
outdir = file;
filename = [file,ext];
urlwrite(zipurl,filename);
disp(['Code has been downloaded to ''',filename,'''. Extracting...']);

% unzip
dcontent = dir(outdir);
if( size(dcontent,1) > 0 )
    confirm = input(['Directory ''',outdir,''' will be overwritten. Proceed? [y]/n '],'s');
    if( strcmp(confirm,'y') || isempty(confirm) )        
        rmdir(outdir,'s');
        unzip(filename,file);
        disp('Code successfully unzipped. MEXing your solver...');    
    end
else
    unzip(filename,file);
    disp('Code successfully unzipped. MEXing your solver...');
end

% make mex file
cd([file,'/interface']);
makemex;
cd ..
cd ..
disp(' ');
disp('Code generation and MEXing successfully completed.');
disp(['Type ''help ',file,''' for more documentation. Happy solving!']);

% return success
success = 1;