function [dims, maxlevel, count] = nestbracket2dim(str,brackets)
%
% [dims, maxlevel, count] = nestbracket2dim(str,brackets)
%
% Extracting the dimension vector of a JSON string formatted array
% by analyzing the pairs of opening/closing bracket tokenss; this function 
% only returns valid dimension information when the array is an N-D array
%
% authors:Qianqian Fang (q.fang <at> neu.edu)
%
% input:
%      str: a string-formatted JSON array using square-brackets for enclosing
%           elements and comma as separators between elements
%      brackets: (optional), a string of length 2, with the first character
%               being the opening token and the 2nd being the closing token.
%               if not given, brackets is set to '[]' to find matching square-brackets;
%               for example, '{}' looks for a matching closing curly-bracket in
%               the string key(pos(startpos,:end))
%
% output:
%      dims: the speculated dimension vector with the length matching the maximum 
%            depth of the embedded bracket pairs. When the input string encodes an
%            N-D array, the dims vector contains all integers; however, returning
%            an all-integer dims vector does not mean the array is rectangular.
%      maxlevel: return the depth of the enclosed brackets in the string, i.e. the
%            length of the dims vector.
%      count: the relative depth from the level 0 - scanning from the left
%            to right of the string, an opening token increases the level by 1
%            and a closing token decreases the level by 1; a zero indicates
%            the positions of a matching bracket of the same level.
%
% example:
%      str='[[ [1,2,3], [4,2,1]], [ [10,1,0], [2,5,10]] ]'; % an N-D array
%      [dim,dep]=nestbracket2dim(str)
%      str='[[ [1,2,3], [4,2,1]], [ [10,1,0], [2,5]] ]'; % an invalid N-D array
%      [dim,dep]=nestbracket2dim(str)

% license:
%     BSD or GPL version 3, see LICENSE_{BSD,GPLv3}.txt files for details
%
% -- this function is part of JSONLab toolbox (http://iso2mesh.sf.net/cgi-bin/index.cgi?jsonlab)
%

if(nargin<2)
    brackets='[]';
end
str=str(str==brackets(1) | str==brackets(2) | str==',');
count=cumsum(str==brackets(1)) - cumsum(str==brackets(2));
maxlevel=max(count);
dims=histc(count,1:maxlevel);
dims(1:end-1)=dims(1:end-1)*0.5;
dims(2:end)=dims(2:end)./dims(1:end-1);
dims=fliplr(dims);