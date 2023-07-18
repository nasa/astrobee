function output = base64decode(varargin)
%
% output = base64decode(input)
%
% Decoding a Base64-encoded byte-stream to recover the original data
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
    output=zmat(varargin{1},0,'base64');
    return;
elseif(isoctavemesh)
    error('You must install the ZMat toolbox (http://github.com/fangq/zmat) to use this function in Octave');
end

error(javachk('jvm'));

if(ischar(varargin{1}))
    varargin{1}=uint8(varargin{1});
end

input=typecast(varargin{1}(:)','uint8');

output = typecast(org.apache.commons.codec.binary.Base64.decodeBase64(input), 'uint8')';

