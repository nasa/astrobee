function install_daqp
    % Install the Matlab interface for DAQP

    % Get platform 
    if ispc
        platform = 'windows';
    elseif ismac
        platform = 'mac';
    elseif isunix
        platform = 'linux';
    end

    fprintf('Downloading binaries...\n');
    package_url= sprintf('https://github.com/darnstrom/daqp/releases/latest/download/daqp-matlab-%s64.tar.gz', platform);
    websave('daqp-matlab.tar.gz', package_url);

    fprintf('Unpacking...\n');
    untar('daqp-matlab.tar.gz','daqp-matlab')

    fprintf('Updating path...\n');
    cd('daqp-matlab')
    addpath(genpath(pwd)); savepath;
    cd ..

    fprintf('Removing tarball... \n');
    delete('daqp-matlab.tar.gz');

    fprintf('DAQP installed!\n');
end
