function varargout = base64encode(varargin)
%
% output = base64encode(input)
%
% Encoding a binary vector or array using Base64
%
% This function depends on JVM in MATLAB or, can optionally use the ZMat 
% toolbox (http://github.com/fangq/zmat)
%
% Copyright (c) 2012, Kota Yamaguchi
% URL: https://www.mathworks.com/matlabcentral/fileexchange/39526-byte-encoding-utilities
%
% Modified by: Qianqian Fang (q.fang <at> neu.edu)
%
% input:
%      input: a base64-encoded string
%
% output:
%      output: the decoded binary byte-stream as a uint8 vector
%
% examples:
%      bytes=base64encode('Test JSONLab');
%      orig=char(base64decode(bytes))
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
    [varargout{1:nargout}]=zmat(varargin{1}, 1,'base64',varargin{2:end});
    return;
end

if(ischar(varargin{1}))
    varargin{1}=uint8(varargin{1});
end

input=typecast(varargin{1}(:)','uint8');

if(isoctavemesh)
    varargout{1} = base64_encode(uint8(input));
    return;
end

error(javachk('jvm'));
if ischar(input)
    input = uint8(input);
end

varargout{1}  = char(org.apache.commons.codec.binary.Base64.encodeBase64Chunked(input))';
varargout{1}  = regexprep(varargout{1} ,'\r','');
