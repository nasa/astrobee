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

% % Generate .csv telemData structure
% fileName = 'kfl_output.csv';
% outTM = out_kfl_msg;
function genCSVfromTM_structs(fileName, outTM)
fileName = [fileName '.csv'];

headerStr = [];
outputMat = [];
structFields = fieldnames(outTM);
for ii = 1:length(structFields)
    if isstruct(outTM.(structFields{ii}))
        parentStr = [(structFields{ii}) '.'];
        [temp_outputMat, temp_headerStr] = genStr(parentStr, outTM.(structFields{ii}));
        headerStr = [headerStr temp_headerStr];
        outputMat = [outputMat temp_outputMat];
    else
        parentStr = [];
        temp_struct.(structFields{ii}) = outTM.(structFields{ii});
        [temp_outputMat, temp_headerStr] = genStr(parentStr, temp_struct);
        headerStr = [headerStr temp_headerStr];
        outputMat = [outputMat temp_outputMat];
    end
end
fh = fopen(fileName, 'w');
fprintf(fh, [headerStr '\n']);
fclose(fh);

dlmwrite(fileName, outputMat, '-append', 'precision', 16)

end


function [outputMat, headerStr] = genStr(parentStr, outTM)
headerStr = [];
outputMat = [];
structFields = fieldnames(outTM);
for ii = 1:length(structFields)
    if ismatrix(outTM.(structFields{ii}).data) % We don't accept stuff with more than 2 dimensions right now
        if isa(outTM.(structFields{ii}).data, 'quaternion')
            numCol = 4;
        else
            numCol = size(outTM.(structFields{ii}).data, 2);
        end
        for kk = 1:numCol
            headerStr = [headerStr parentStr structFields{ii}, '[' , num2str(kk), '],' ];
        end
        outputMat = [outputMat double(outTM.(structFields{ii}).data)];
    else
        
        sizeTM = size(outTM.(structFields{ii}).data);
        if numel(sizeTM) == 3
            for kk = 1:size(outTM.(structFields{ii}).data, 1)
                for jj = 1:size(outTM.(structFields{ii}).data, 2)
                    headerStr = [headerStr parentStr structFields{ii}, '[' , num2str(kk), '-', num2str(jj), '],' ];
                end
            end
            outputMat = [outputMat double(reshape(permute(outTM.(structFields{ii}).data, [3 2 1]),sizeTM(3), sizeTM(1)*sizeTM(2)))];
        else
            for kk = 1:size(outTM.(structFields{ii}).data, 1)
                for jj = 1:size(outTM.(structFields{ii}).data, 2)
                    for ll = 1:size(outTM.(structFields{ii}).data, 3)
                        headerStr = [headerStr parentStr structFields{ii}, '[' , num2str(kk), '-', num2str(jj), '-', num2str(ll) '],' ];
                    end
                end
            end
            outputMat = [outputMat double(reshape(permute(outTM.(structFields{ii}).data, [4 3 2 1]),sizeTM(4), sizeTM(1)*sizeTM(2)*sizeTM(3)))];
        end
        
    end
end
end
