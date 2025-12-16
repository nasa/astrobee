function data = loadmsgpack(fname,varargin)
%
%   data = loadmsgpack(fname,varargin)
%
%LOADMSGPACK parses a msgpack byte buffer into Matlab data structures
% LOADMSGPACK(BYTES)
%    reads BYTES as msgpack data, and creates Matlab data structures
%    from it.
%    - strings are converted to strings
%    - numbers are converted to appropriate numeric values
%    - true, false are converted to logical 1, 0
%    - nil is converted to []
%    - arrays are converted to cell arrays
%    - maps are converted to containers.Map
%
% (c) 2016 Bastian Bechtold
% modified by Qianqian Fang <q.fang at neu.edu>
%
% license:
%     BSD 3-clause license or GPL version 3, see LICENSE_{BSD,GPLv3}.txt files for details 
%
% -- this function is part of JSONLab toolbox (http://iso2mesh.sf.net/cgi-bin/index.cgi?jsonlab)
%

    if(exist(fname,'file'))
       fid = fopen(fname,'rb');
       bytes = fread(fid,inf,'uint8=>char')';
       fclose(fid);
    else
       bytes=fname;
    end
    
    opt=varargin2struct(varargin{:});
    opt.simplifycell=jsonopt('SimplifyCell',0,opt);

    jsoncount=1;
    idx=0;
    while idx <= length(bytes)
        [obj, idx] = parse(uint8(bytes(:)), 1, opt);
        data{jsoncount}=obj;
        jsoncount=jsoncount+1;
    end

    jsoncount=length(data);
    if(jsoncount==1 && iscell(data))
        data=data{1};
    end
    if(iscell(data))
        data=cellfun(@(x) jdatadecode(x),data,'UniformOutput',false);
    elseif(isstruct(data))
        data=jdatadecode(data,'Base64',0, opt);
    end
end

function [obj, idx] = parse(bytes, idx, varargin)
    % masks:
    b10000000 = 128;
    b01111111 = 127;
    b11000000 = 192;
    b00111111 = 63;
    b11100000 = 224;
    b00011111 = 31;
    b11110000 = 240;
    b00001111 = 15;
    % values:
    b00000000 = 0;
    b10010000 = 144;
    b10100000 = 160;

    currentbyte = bytes(idx);

    if bitand(b10000000, currentbyte) == b00000000
        % decode positive fixint
        obj = int8(currentbyte);
        idx = idx + 1;
        return
    elseif bitand(b11100000, currentbyte) == b11100000
        % decode negative fixint
        obj = typecast(currentbyte, 'int8');
        idx = idx + 1;
        return
    elseif bitand(b11110000, currentbyte) == b10000000
        % decode fixmap
        len = double(bitand(b00001111, currentbyte));
        [obj, idx] = parsemap(len, bytes, idx+1, varargin{:});
        return
    elseif bitand(b11110000, currentbyte) == b10010000
        % decode fixarray
        len = double(bitand(b00001111, currentbyte));
        [obj, idx] = parsearray(len, bytes, idx+1, varargin{:});
        return
    elseif bitand(b11100000, currentbyte) == b10100000
        % decode fixstr
        len = double(bitand(b00011111, currentbyte));
        [obj, idx] = parsestring(len, bytes, idx + 1);
        return
    end

    switch currentbyte
        case 192 % nil
            obj = [];
            idx = idx+1;
      % case 193 % unused
        case 194 % false
            obj = false;
            idx = idx+1;
        case 195 % true
            obj = true;
            idx = idx+1;
        case 196 % bin8
            len = double(bytes(idx+1));
            [obj, idx] = parsebytes(len, bytes, idx+2);
        case 197 % bin16
            len = double(bytes2scalar(bytes(idx+1:idx+2), 'uint16'));
            [obj, idx] = parsebytes(len, bytes, idx+3);
        case 198 % bin32
            len = double(bytes2scalar(bytes(idx+1:idx+4), 'uint32'));
            [obj, idx] = parsebytes(len, bytes, idx+5);
        case 199 % ext8
            len = double(bytes(idx+1));
            [obj, idx] = parseext(len, bytes, idx+1);
        case 200 % ext16
            len = double(bytes2scalar(bytes(idx+1:idx+2), 'uint16'));
            [obj, idx] = parseext(len, bytes, idx+3);
        case 201 % ext32
            len = double(bytes2scalar(bytes(idx+1:idx+4), 'uint32'));
            [obj, idx] = parseext(len, bytes, idx+5);
        case 202 % float32
            obj = bytes2scalar(bytes(idx+1:idx+4), 'single');
            idx = idx+5;
        case 203 % float64
            obj = bytes2scalar(bytes(idx+1:idx+8), 'double');
            idx = idx+9;
        case 204 % uint8
            obj = bytes(idx+1);
            idx = idx+2;
        case 205 % uint16
            obj = bytes2scalar(bytes(idx+1:idx+2), 'uint16');
            idx = idx+3;
        case 206 % uint32
            obj = bytes2scalar(bytes(idx+1:idx+4), 'uint32');
            idx = idx+5;
        case 207 % uint64
            obj = bytes2scalar(bytes(idx+1:idx+8), 'uint64');
            idx = idx+9;
        case 208 % int8
            obj = bytes2scalar(bytes(idx+1), 'int8');
            idx = idx+2;
        case 209 % int16
            obj = bytes2scalar(bytes(idx+1:idx+2), 'int16');
            idx = idx+3;
        case 210 % int32
            obj = bytes2scalar(bytes(idx+1:idx+4), 'int32');
            idx = idx+5;
        case 211 % int64
            obj = bytes2scalar(bytes(idx+1:idx+8), 'int64');
            idx = idx+9;
        case 212 % fixext1
            [obj, idx] = parseext(1, bytes, idx+1);
        case 213 % fixext2
            [obj, idx] = parseext(2, bytes, idx+1);
        case 214 % fixext4
            [obj, idx] = parseext(4, bytes, idx+1);
        case 215 % fixext8
            [obj, idx] = parseext(8, bytes, idx+1);
        case 216 % fixext16
            [obj, idx] = parseext(16, bytes, idx+1);
        case 217 % str8
            len = double(bytes(idx+1));
            [obj, idx] = parsestring(len, bytes, idx+2);
        case 218 % str16
            len = double(bytes2scalar(bytes(idx+1:idx+2), 'uint16'));
            [obj, idx] = parsestring(len, bytes, idx+3);
        case 219 % str32
            len = double(bytes2scalar(bytes(idx+1:idx+4), 'uint32'));
            [obj, idx] = parsestring(len, bytes, idx+5);
        case 220 % array16
            len = double(bytes2scalar(bytes(idx+1:idx+2), 'uint16'));
            [obj, idx] = parsearray(len, bytes, idx+3, varargin{:});
        case 221 % array32
            len = double(bytes2scalar(bytes(idx+1:idx+4), 'uint32'));
            [obj, idx] = parsearray(len, bytes, idx+5, varargin{:});
        case 222 % map16
            len = double(bytes2scalar(bytes(idx+1:idx+2), 'uint16'));
            [obj, idx] = parsemap(len, bytes, idx+3, varargin{:});
        case 223 % map32
            len = double(bytes2scalar(bytes(idx+1:idx+4), 'uint32'));
            [obj, idx] = parsemap(len, bytes, idx+5, varargin{:});
        otherwise
            error('transplant:parsemsgpack:unknowntype', ...
                  ['Unknown type "' dec2bin(currentbyte) '"']);
    end
end

function value = bytes2scalar(bytes, type)
    % reverse byte order to convert from little-endian to big-endian
    value = typecast(bytes(end:-1:1), type);
end

function [str, idx] = parsestring(len, bytes, idx)
    if(~isoctavemesh)
         str = native2unicode(bytes(idx:idx+len-1)', 'utf-8');
    else
         str = char(bytes(idx:idx+len-1)');
    end
    idx = idx + len;
end

function [out, idx] = parsebytes(len, bytes, idx)
    out = bytes(idx:idx+len-1);
    idx = idx + len;
end

function [out, idx] = parseext(len, bytes, idx)
    obj.type = bytes(idx);
    obj.data = bytes(idx+1:idx+len);
    idx = idx + len + 1;
end

function [out, idx] = parsearray(len, bytes, idx, varargin)
    out = cell(len,1);
    for n=1:len
        [out{n}, idx] = parse(bytes, idx, varargin{:});
    end
    if(isnumeric(out{1}))
      try
        oldobj=out;
        out=cell2mat(out);
        if(iscell(oldobj) && isstruct(out) && numel(out)>1 && jsonopt('SimplifyCellArray',1,varargin{:})==0)
            out=oldobj;
        elseif(size(out,1)>1 && ismatrix(out))
            out=out';
        end
      catch
      end
    end
end

function [out, idx] = parsemap(len, bytes, idx, varargin)
    out = struct();
    for n=1:len
        [key, idx] = parse(bytes, idx, varargin{:});
        [out.(encodevarname(char(key))), idx] = parse(bytes, idx, varargin{:});
    end
end
