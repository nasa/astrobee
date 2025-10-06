function [endpos, maxlevel] = fast_match_bracket(key,pos,startpos,brackets)
%
% [endpos, maxlevel] = fast_match_bracket(key,pos,startpos,brackets)
%
% A fast function to find the position of a closing bracket token in a string
%
% authors:Qianqian Fang (q.fang <at> neu.edu)
%
% input:
%      key: a preprocessed string containing only relevant opening/closing 
%           bracket characters for accelerating the search.
%      pos: a 1D integer vector with a length matching the length of key, 
%           recording the corresponding position of each char. in the original string.
%      startpos: the index in the original string as the start position to search; the
%               startpos must be at least 1 greater than the opening bracket position
%      brackets: (optional), a string of length 2, with the first character
%               being the opening token and the 2nd being the closing token.
%               if not given, brackets is set to '[]' to find matching square-brackets;
%               for example, '{}' looks for a matching closing curly-bracket in
%               the string key(pos(startpos,:end))
%
% output:
%      endpos: if a matching bracket is found, return its position in the original 
%              string
%      maxlevel: return the depth of the enclosed brackets between the searched pair,
%              includig the searching pair. For example, the matching closing-bracket 
%              of the 1st square bracket (startpos=2) in  '[[[]],[]]' returns a 
%              position of 9, with a maximum depth of 3; searching for the closing 
%              bracket for the 2nd square bracket (startpos=3) returns a position of 
%              5 and max-depth of 2.
%
% example:
%      str='[[ [1,2], 1], 10, [5,10] ]';
%      pos=find(str=='[' | str==']')
%      key=str(pos)
%      [p1,dep]=fast_match_bracket(key,1:length(key),3)
%      [p2,dep]=fast_match_bracket(key,pos,2)
%      [p3,dep]=fast_match_bracket(key,pos,3)
%
% license:
%     BSD or GPL version 3, see LICENSE_{BSD,GPLv3}.txt files for details
%
% -- this function is part of JSONLab toolbox (http://iso2mesh.sf.net/cgi-bin/index.cgi?jsonlab)
%

if(nargin<4)
    brackets='[]';
end
startpos=find( pos >= startpos, 1 );
count = key(startpos:end);
if(length(count)==1 && count==']')
    endpos=pos(end);
    maxlevel=1;
    return;
end
flag=cumsum(count==brackets(1))-cumsum(count==brackets(2))+1;
endpos = find(flag==0,1);
maxlevel=max([1,max(flag(1:endpos))]);
endpos = pos(endpos + startpos-1);
