function val=jsonopt(key,default,varargin)
%
% val=jsonopt(key,default,optstruct)
%
% setting options based on a struct. The struct can be produced
% by varargin2struct from a list of 'param','value' pairs
%
% authors:Qianqian Fang (q.fang <at> neu.edu)
%
% input:
%      key: a string with which one look up a value from a struct
%      default: if the key does not exist, return default
%      optstruct: a struct where each sub-field is a key 
%
% output:
%      val: if key exists, val=optstruct.key; otherwise val=default
%
% license:
%     BSD or GPL version 3, see LICENSE_{BSD,GPLv3}.txt files for details
%
% -- this function is part of JSONLab toolbox (http://iso2mesh.sf.net/cgi-bin/index.cgi?jsonlab)
% 

val=default;
if(nargin<=2)
    return;
end
key0=lower(key);
opt=varargin{1};
if(isstruct(opt))
    if(isfield(opt,key0))
       val=opt.(key0);
    elseif(isfield(opt,key))
       val=opt.(key);
    end
end
