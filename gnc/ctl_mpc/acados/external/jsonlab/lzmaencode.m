function varargout = lzmaencode(varargin)
%
% output = lzmaencode(input)
%    or
% [output, info] = lzmaencode(input)
%
% Compress a string or a numerical array using LZMA-compression
%
% This function depends on the ZMat toolbox (http://github.com/fangq/zmat)
%
% authors:Qianqian Fang (q.fang <at> neu.edu)
%
% input:
%      input: the original data, can be a string, a numerical vector or array
%
% output:
%      output: the compressed byte stream stored in a uint8 vector
%      info: (optional) a struct storing the metadata of the input, see "help zmat" for details
%
% examples:
%      [bytes, info]=lzmaencode(eye(10));
%      orig=lzmadecode(bytes,info);
%
% license:
%     BSD or GPL version 3, see LICENSE_{BSD,GPLv3}.txt files for details 
%
% -- this function is part of JSONLab toolbox (http://iso2mesh.sf.net/cgi-bin/index.cgi?jsonlab)
%

if(nargin==0)
    error('you must provide at least 1 input');
end

if(exist('zmat','file')==2 || exist('zmat','file')==3)
    [varargout{1:nargout}]=zmat(varargin{1}, 1,'lzma',varargin{2:end});
    return;
else
    error('you must install ZMat toolbox to use this feature: http://github.com/fangq/zmat')
end
