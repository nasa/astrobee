function msgpk=savemsgpack(rootname,obj,varargin)
%
% msgpk=savemsgpack(obj)
%    or
% msgpk=savemsgpack(rootname,obj,filename)
% msgpk=savemsgpack(rootname,obj,opt)
% msgpk=savemsgpack(rootname,obj,'param1',value1,'param2',value2,...)
%
% convert a MATLAB object (cell, struct, array, table, map, handles ...) 
% into a MessagePack binary stream
%
% author: Qianqian Fang (q.fang <at> neu.edu)
% initially created on 2019/05/20
%
% This function is the same as calling saveubjson(...,'MessagePack',1)
%
% Please type "help saveubjson" for details for the supported inputs and outputs.
%
% license:
%     BSD or GPL version 3, see LICENSE_{BSD,GPLv3}.txt files for details
%
% -- this function is part of JSONLab toolbox (http://iso2mesh.sf.net/cgi-bin/index.cgi?jsonlab)
%

if(nargin==1)
    msgpk=saveubjson('',rootname,'MessagePack',1);
elseif(length(varargin)==1 && ischar(varargin{1}))
    msgpk=saveubjson(rootname,obj,'FileName',varargin{1},'MessagePack',1);
else
    msgpk=saveubjson(rootname,obj,varargin{:},'MessagePack',1);
end