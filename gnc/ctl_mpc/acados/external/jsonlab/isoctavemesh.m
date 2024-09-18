function [isoctave, verinfo]=isoctavemesh
%
% [isoctave verinfo]=isoctavemesh
%
% determine whether the code is running in octave
%
% author: Qianqian Fang, <q.fang at neu.edu>
%
% output:
%   isoctave: 1 if in octave, otherwise 0
%   verinfo: a string, showing the version of octave (OCTAVE_VERSION)
%
% -- this function is part of iso2mesh toolbox (http://iso2mesh.sf.net)
%
verinfo='';
isoctave=(exist('OCTAVE_VERSION','builtin')~=0);
if(nargout==2 && isoctave)
    verinfo=OCTAVE_VERSION;
end
