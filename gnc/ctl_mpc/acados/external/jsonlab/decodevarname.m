function newname = decodevarname(name,varargin)
%
%    newname = decodevarname(name)
%
%    Decode a hex-encoded variable name (from encodevarname) and restore
%    its original form
%
%    This function is sensitive to the default charset
%    settings in MATLAB, please call feature('DefaultCharacterSet','utf8')
%    to set the encoding to UTF-8 before calling this function.
%
%    author: Qianqian Fang (q.fang <at> neu.edu)
%
%    input:
%        name: a string output from encodevarname, which converts the leading non-ascii
%              letter into "x0xHH_" and non-ascii letters into "_0xHH_"
%              format, where hex key HH stores the ascii (or Unicode) value
%              of the character.
%              
%    output:
%        newname: the restored original string
%
%    example:
%        decodevarname('x0x5F_a')   % returns _a
%        decodevarname('a_')   % returns a_ as it is a valid variable name
%        decodevarname('x0xE58F98__0xE9878F_')  % returns '变量' 
%
%    this file is part of EasyH5 Toolbox: https://github.com/fangq/easyh5
%
%    License: GPLv3 or 3-clause BSD license, see https://github.com/fangq/easyh5 for details
%

newname=name;
isunpack=1;
if(nargin==2 && ~isstruct(varargin{1}))
    isunpack=varargin{1};
elseif(nargin>1)
    isunpack=jsonopt('UnpackHex',1,varargin{:});
end

if(isunpack)
    if(isempty(regexp(name,'0x([0-9a-fA-F]+)_','once')))
        return
    end
    if(exist('native2unicode','builtin'))
        h2u=@hex2unicode;
        newname=regexprep(name,'(^x|_){1}0x([0-9a-fA-F]+)_','${h2u($2)}');
    else
        pos=regexp(name,'(^x|_){1}0x([0-9a-fA-F]+)_','start');
        pend=regexp(name,'(^x|_){1}0x([0-9a-fA-F]+)_','end');
        if(isempty(pos))
            return;
        end
        str0=name;
        pos0=[0 pend(:)' length(name)];
        newname='';
        for i=1:length(pos)
            newname=[newname str0(pos0(i)+1:pos(i)-1) char(hex2dec(str0(pos(i)+3:pend(i)-1)))];
        end
        if(pos(end)~=length(name))
            newname=[newname str0(pos0(end-1)+1:pos0(end))];
        end
    end
end

%--------------------------------------------------------------------------
function str=hex2unicode(hexstr)
val=hex2dec(hexstr);
id=histc(val,[0 2^8 2^16 2^32 2^64]);
type={'uint8','uint16','uint32','uint64'};
bytes=typecast(cast(val,type{id~=0}),'uint8');
str=native2unicode(fliplr(bytes(:,1:find(bytes,1,'last'))));
