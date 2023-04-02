function [endpos, maxlevel] = match_bracket(str,startpos,brackets)
%
% [endpos, maxlevel] = match_bracket(str,startpos,brackets)
%
% Looking for the position of a closing bracket token in a string
%
% authors:Qianqian Fang (q.fang <at> neu.edu)
%
% input:
%      str: the full string to be searched
%      startpos: the index in the string as the start position to search; the
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
%      [p1,dep]=match_bracket(str,3)
%      [p2,dep]=match_bracket(str,2)
%      [p3,dep]=match_bracket(str,3)
%
% license:
%     BSD or GPL version 3, see LICENSE_{BSD,GPLv3}.txt files for details
%
% -- this function is part of JSONLab toolbox (http://iso2mesh.sf.net/cgi-bin/index.cgi?jsonlab)
%

if(nargin<3)
    brackets='[]';
end
count = str(startpos:end);
flag=cumsum(count==brackets(1))-cumsum(count==brackets(2))+1;
endpos = find(flag==0,1);
maxlevel=max(flag(1:endpos));
endpos = endpos + startpos-1;
