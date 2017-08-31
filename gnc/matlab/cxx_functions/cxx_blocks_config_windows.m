% Copyright (c) 2017, United States Government, as represented by the
% Administrator of the National Aeronautics and Space Administration.
%
% All rights reserved.
%
% The Astrobee platform is licensed under the Apache License, Version 2.0
% (the "License"); you may not use this file except in compliance with the
% License. You may obtain a copy of the License at
%
%     http://www.apache.org/licenses/LICENSE-2.0
%
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
% WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
% License for the specific language governing permissions and limitations
% under the License.
%
%%generate the mex files used by simulink for the hand coded C++ files listed below
% This file is automated to check the checksums and compile if they have changed

%setup variables
function cxx_blocks_config_windows(ASTROBEE_ROOT, ase_of_num_aug, ase_of_num_features, ab_verbose)
file_names = {'apply_delta_state', 'compute_delta_state_and_cov', 'matrix_multiply', 'of_residual_and_h'};
func_dec1 = ['void apply_delta_state(single u1[], int32 size(u1, 1), uint16 u2, single u3[4], single u4[3], single u5[3], single u6[3], single u7[3], ' ...
            'single u8[4], single u9[3], uint16 u10, single u11[' num2str(ase_of_num_aug.Value) '][4], single u12[' num2str(ase_of_num_aug.Value) ...
            '][3], single y1[4], single y2[3], single y3[3], single y4[3], single y5[3], single y6[4], single y7[3], uint16 y8[1], single y9[' ...
            num2str(ase_of_num_aug.Value) '][4], single y10[' num2str(ase_of_num_aug.Value) '][3])'];
func_dec2 = ['int32 y1 = compute_delta_state_and_cov(single u1[], int32 u2, single u3[][], int32 size(u3, 1), int32 size(u3, 2), ' ...
              'uint32 u4, single u5[][], single u6[][], single y2[size(u3, 2)], single y3[size(u3, 2)][size(u3, 2)])'];
func_dec3 = 'void matrix_multiply(single u1[][], int32 size(u1, 1), int32 size(u1, 2), single u2[][], int32 size(u2, 1), int32 size(u2, 2), single y1[size(u1, 1)][size(u2, 2)])';
func_dec4 = ['int32 y1 = of_residual_and_h(single u1[' num2str(2*ase_of_num_aug.Value*ase_of_num_features.Value) '], single u2[4][' ...
              num2str(ase_of_num_features.Value) '], single u3[' num2str(4 * ase_of_num_aug.Value) '][4], int32 u4[' num2str(ase_of_num_features.Value) ...
              '][' num2str(ase_of_num_aug.Value) '], int32 u5, int32 p1, int32 p2, single p3, single p4, single u6[' num2str(21 + 6 * ase_of_num_aug.Value) ...
              '][' num2str(21 + 6 * ase_of_num_aug.Value) '], single y2[', num2str(6 * ase_of_num_aug.Value) '], single y3[' num2str(6 * ase_of_num_aug.Value) ...
              '][' num2str(21 + 6 * ase_of_num_aug.Value) '], uint32 y4[1], uint8 y5[1], single y6[' num2str(ase_of_num_features.Value) '])'];        
function_declarations = {func_dec1, func_dec2, func_dec3, func_dec4};

cd(fullfile(ASTROBEE_ROOT, 'cxx_functions'));
%check if the checksum file exists, if not, create it.
if exist('file_checksums.mat', 'file')
    load('file_checksums.mat');
else 
    file_checksums.exists = 1;
    if(ab_verbose) 
        disp('.....file_checksums.mat did not exist, creating');
    end
end

%check the eigen version 
cd(fullfile(ASTROBEE_ROOT, '..', '..', 'external', 'eigen'));
eigen_ver_text  = fileread('CMakeLists.txt');
eigen_ver_start = strfind(eigen_ver_text, '/get/')+5;
eigen_ver_end   = strfind(eigen_ver_text, '.tar.')-1;
eigen_ver       = eigen_ver_text(eigen_ver_start:eigen_ver_end);
cd(fullfile(ASTROBEE_ROOT, 'cxx_functions'));

%update if necessary
if(~isfield(file_checksums, 'eigen_lib_ver') || exist('Eigen', 'dir')~=7 || ~strcmp(file_checksums.eigen_lib_ver,eigen_ver))
    %if no version in .mat file, no folder exists, or does not equal current version, checkout new version
    if(ab_verbose) 
        disp('.....No Eigen Library Found, performing one time download now');
    end
    file_checksums.eigen_lib_ver = eigen_ver;
    websave('Eigen', 'http://bitbucket.org/eigen/eigen/get/3.2.2.zip');
    unziped_files = unzip('Eigen.zip');
    new_folder = strsplit(fileparts(unziped_files{1}),{'/','\'});
    movefile([new_folder{1} '/Eigen']);
    rmdir(new_folder{1},'s');
    delete('Eigen.zip');
end
    

for i = 1:4 
    %determine checksums of current '.h' and '.cpp' files
    file_cpp_checksum   = Simulink.getFileChecksum(which([file_names{i} '.cpp']));
    file_h_checksum     = Simulink.getFileChecksum(which([file_names{i} '.h']));
    
    %check if files already exist.  Note: exist==3 impiles a mex file exists
    if(exist(['ex_' file_names{i} '.cpp'], 'file') && exist(['ex_' file_names{i}], 'file')==3 && exist(['ex_' file_names{i} '.tlc'], 'file'))
        %check if checksum even contains correct files
        if(all(isfield(file_checksums,{[file_names{i} 'cpp'], [file_names{i} 'h']})))   
            %check if checksums are correct
            if(strcmp(file_cpp_checksum,file_checksums.([file_names{i} 'cpp'])) && strcmp(file_h_checksum,file_checksums.([file_names{i} 'h'])))
                continue; %skip if they are equal!
            end
        end
    end
    
    if(ab_verbose) 
        disp(['.....Building ' file_names{i}]);
    end
    
    %if any are false, then generate the code
    def                     = legacy_code('initialize');  	% initialize def variable
    def.SourceFiles         = {[file_names{i} '.cpp']};    	% Source file for block
    def.HeaderFiles         = {[file_names{i} '.h']};       % Header file for block
    def.SFunctionName       = ['ex_' file_names{i}];        % sfunction name
    def.OutputFcnSpec       = function_declarations{i};     % See this help file:  http://www.mathworks.com/help/simulink/sfg/integrating-existing-c-functions-into-simulink-models-with-the-legacy-code-tool.html#bq4g1es-6
    def.Options.language    = 'C++';                        % Language
%     def.SrcPaths            = {'./src', './../../external/eigen/Eigen/src'};     % Can add multiple sources as cell arrays of strings
%     def.IncPaths            = {'./include', './../../external/eigen/Eigen'};     
    def.SrcPaths            = {'./src', './Eigen/src'};     % Can add multiple sources as cell arrays of strings
    def.IncPaths            = {'./include', './Eigen'}; 
    def.Options.singleCPPMexFile                = false;    % Option need for code generation
    def.Options.supportsMultipleExecInstances   = true;

    legacy_code('sfcn_tlc_generate', def);
    legacy_code('sfcn_cmex_generate', def);
    legacy_code('compile', def);
    legacy_code('rtwmakecfg_generate', def);
    %legacy_code('slblock_generate', def);
    
    %and store the checksums
    file_checksums.([file_names{i} 'h'])     = file_h_checksum;
    file_checksums.([file_names{i} 'cpp'])   = file_cpp_checksum;
end

%save the checksums file
save('file_checksums.mat', 'file_checksums');
cd(ASTROBEE_ROOT);
return
