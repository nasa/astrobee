function data = loadubjson(fname,varargin)
%
% data=loadubjson(fname,opt)
%    or
% data=loadubjson(fname,'param1',value1,'param2',value2,...)
%
% parse a JSON (JavaScript Object Notation) file or string
%
% authors:Qianqian Fang (q.fang <at> neu.edu)
% initially created on 2013/08/01
%
% input:
%      fname: input file name, if fname contains "{}" or "[]", fname
%             will be interpreted as a UBJSON string
%      opt: a struct to store parsing options, opt can be replaced by 
%           a list of ('param',value) pairs - the param string is equivallent
%           to a field in opt. opt can have the following 
%           fields (first in [.|.] is the default)
%
%           SimplifyCell [0|1]: if set to 1, loadubjson will call cell2mat
%                         for each element of the JSON data, and group 
%                         arrays based on the cell2mat rules.
%           IntEndian [B|L]: specify the endianness of the integer fields
%                         in the UBJSON input data. B - Big-Endian format for 
%                         integers (as required in the UBJSON specification); 
%                         L - input integer fields are in Little-Endian order.
%           NameIsString [0|1]: for UBJSON Specification Draft 8 or 
%                         earlier versions (JSONLab 1.0 final or earlier), 
%                         the "name" tag is treated as a string. To load 
%                         these UBJSON data, you need to manually set this 
%                         flag to 1.
%           FormatVersion [2|float]: set the JSONLab format version; since
%                         v2.0, JSONLab uses JData specification Draft 1
%                         for output format, it is incompatible with all
%                         previous releases; if old output is desired,
%                         please set FormatVersion to 1.9 or earlier.
%
% output:
%      dat: a cell array, where {...} blocks are converted into cell arrays,
%           and [...] are converted to arrays
%
% examples:
%      obj=struct('string','value','array',[1 2 3]);
%      ubjdata=saveubjson('obj',obj);
%      dat=loadubjson(ubjdata)
%      dat=loadubjson(['examples' filesep 'example1.ubj'])
%      dat=loadubjson(['examples' filesep 'example1.ubj'],'SimplifyCell',1)
%
% license:
%     BSD or GPL version 3, see LICENSE_{BSD,GPLv3}.txt files for details 
%
% -- this function is part of JSONLab toolbox (http://iso2mesh.sf.net/cgi-bin/index.cgi?jsonlab)
%

    if(regexp(fname,'[\{\}\]\[]','once'))
       string=fname;
    elseif(exist(fname,'file'))
       fid = fopen(fname,'rb');
       string = fread(fid,inf,'uint8=>char')';
       fclose(fid);
    else
       error('input file does not exist');
    end

    pos = 1; inputlen = length(string); inputstr = string;
    arraytoken=find(inputstr=='[' | inputstr==']' | inputstr=='"');
    jstr=regexprep(inputstr,'\\\\','  ');
    escquote=regexp(jstr,'\\"');
    arraytoken=sort([arraytoken escquote]);

    opt=varargin2struct(varargin{:});
    opt.arraytoken_=arraytoken;

    [os,maxelem,systemendian]=computer;
    opt.flipendian_=(systemendian ~= upper(jsonopt('IntEndian','B',opt)));

    jsoncount=1;
    while pos <= inputlen
        [cc, pos]=next_char(inputstr, pos);
        switch(cc)
            case '{'
                [data{jsoncount}, pos] = parse_object(inputstr, pos, opt);
            case '['
                [data{jsoncount}, pos] = parse_array(inputstr, pos, opt);
            otherwise
                error_pos('Outer level structure must be an object or an array', inputstr, pos);
        end
        jsoncount=jsoncount+1;
    end % while

    jsoncount=length(data);
    if(jsoncount==1 && iscell(data))
        data=data{1};
    end

    if(jsonopt('JDataDecode',1,varargin{:})==1)
        data=jdatadecode(data,'Base64',0,'Recursive',1,varargin{:});
    end
end

%%-------------------------------------------------------------------------
%% helper functions
%%-------------------------------------------------------------------------

function [data, adv]=parse_block(inputstr, pos, type,count,varargin)
    [cid,len]=elem_info(inputstr, pos, type);
    datastr=inputstr(pos:pos+len*count-1);
    newdata=uint8(datastr);
    id=strfind('iUIlLdD',type);
    if(jsonopt('flipendian_',1,varargin{:}))
        newdata=swapbytes(typecast(newdata,cid));
    end
    data=typecast(newdata,cid);
    adv=double(len*count);
end
%%-------------------------------------------------------------------------

function [object, pos] = parse_array(inputstr,  pos, varargin) % JSON array is written in row-major order
    pos=parse_char(inputstr, pos, '[');
    object = cell(0, 1);
    dim=[];
    type='';
    count=-1;
    [cc,pos]=next_char(inputstr,pos);
    if(cc == '$')
        type=inputstr(pos+1);
        pos=pos+2;
    end
    [cc,pos]=next_char(inputstr,pos);
    if(cc == '#')
        pos=pos+1;
        [cc,pos]=next_char(inputstr,pos);
        if(cc=='[')
            [dim, pos]=parse_array(inputstr, pos, varargin{:});
            count=prod(double(dim));
        else
            [val,pos]=parse_number(inputstr,pos, varargin{:});
            count=double(val);
        end
    end
    if(~isempty(type))
        if(count>=0)
            [object, adv]=parse_block(inputstr, pos, type,count,varargin{:});
            if(~isempty(dim))
                object=reshape(object,dim);
            end
            pos=pos+adv;
            return;
        else
            endpos=match_bracket(inputstr,pos);
            [cid,len]=elem_info(inputstr, pos, type);
            count=(endpos-pos)/len;
            [object, adv]=parse_block(inputstr, pos, type,count,varargin{:});
            pos=pos+adv;
            pos=parse_char(inputstr, pos, ']');
            return;
        end
    end
    [cc,pos]=next_char(inputstr,pos);
    if cc ~= ']'
         while 1
            [val, pos] = parse_value(inputstr, pos, varargin{:});
            object{end+1} = val;
            [cc,pos]=next_char(inputstr,pos);
            if cc == ']'
                break;
            end
         end
    end
    if(jsonopt('SimplifyCell',0,varargin{:})==1)
      try
        oldobj=object;
        object=cell2mat(object')';
        if(iscell(oldobj) && isstruct(object) && numel(object)>1 && jsonopt('SimplifyCellArray',1,varargin{:})==0)
            object=oldobj;
        elseif(size(object,1)>1 && ismatrix(object))
            object=object';
        end
      catch
      end
    end
    if(count==-1)
        pos=parse_char(inputstr, pos, ']');
    end
end

%%-------------------------------------------------------------------------

function pos=parse_char(inputstr, pos, c)
    if pos > length(inputstr) || inputstr(pos) ~= c
        error_pos(sprintf('Expected %c at position %%d', c),inputstr, pos);
    else
        pos = pos + 1;
    end
end

%%-------------------------------------------------------------------------

function [c, pos] = next_char(inputstr, pos)
    if pos > length(inputstr)
        c = [];
    else
        c = inputstr(pos);
    end
end

%%-------------------------------------------------------------------------
function [str, pos] = parse_name(inputstr, pos, varargin)
    [val, pos]=parse_number(inputstr,pos,varargin{:});
    bytelen=double(val);
    if(length(inputstr)>=pos+bytelen-1)
        str=inputstr(pos:pos+bytelen-1);
        pos=pos+bytelen;
    else
        error_pos('End of file while expecting end of name', inputstr, pos);
    end
end

%%-------------------------------------------------------------------------

function [str, pos] = parseStr(inputstr, pos, varargin)
    type=inputstr(pos);
    if type ~= 'S' && type ~= 'C' && type ~= 'H'
        error_pos('String starting with S expected at position %d',inputstr, pos);
    else
        pos = pos + 1;
    end
    if(type == 'C')
        str=inputstr(pos);
        pos=pos+1;
        return;
    end
    [val, pos]=parse_number(inputstr,pos,varargin{:});
    bytelen=double(val);
    if(length(inputstr)>=pos+bytelen-1)
        str=inputstr(pos:pos+bytelen-1);
        pos=pos+bytelen;
    else
        error_pos('End of file while expecting end of inputstr',inputstr, pos);
    end
end

%%-------------------------------------------------------------------------

function [num, pos] = parse_number(inputstr, pos, varargin)
    id=strfind('iUIlLdD',inputstr(pos));
    if(isempty(id))
        error_pos('expecting a number at position %d',inputstr, pos);
    end
    type={'int8','uint8','int16','int32','int64','single','double'};
    bytelen=[1,1,2,4,8,4,8];
    datastr=inputstr(pos+1:pos+bytelen(id));
    newdata=uint8(datastr);
    if(jsonopt('flipendian_',1,varargin{:}))
        newdata=swapbytes(typecast(newdata,type{id}));
    end
    num=typecast(newdata,type{id});
    pos = pos + bytelen(id)+1;
end

%%-------------------------------------------------------------------------

function [val, pos] = parse_value(inputstr, pos, varargin)
    switch(inputstr(pos))
        case {'S','C','H'}
            [val, pos] = parseStr(inputstr, pos, varargin{:});
            return;
        case '['
            [val, pos] = parse_array(inputstr, pos, varargin{:});
            return;
        case '{'
            [val, pos] = parse_object(inputstr, pos, varargin{:});
            return;
        case {'i','U','I','l','L','d','D'}
            [val, pos] = parse_number(inputstr, pos, varargin{:});
            return;
        case 'T'
            val = true;
            pos = pos + 1;
            return;
        case 'F'
            val = false;
            pos = pos + 1;
            return;
        case {'Z','N'}
            val = [];
            pos = pos + 1;
            return;
    end
    error_pos('Value expected at position %d', inputstr, pos);
end
%%-------------------------------------------------------------------------

function pos=error_pos(msg, inputstr, pos)
    poShow = max(min([pos-15 pos-1 pos pos+20],length(inputstr)),1);
    if poShow(3) == poShow(2)
        poShow(3:4) = poShow(2)+[0 -1];  % display nothing after
    end
    msg = [sprintf(msg, pos) ': ' ...
    inputstr(poShow(1):poShow(2)) '<error>' inputstr(poShow(3):poShow(4)) ];
    error( ['JSONparser:invalidFormat: ' msg] );
end

%%-------------------------------------------------------------------------
function [object, pos] = parse_object(inputstr, pos, varargin)
    pos=parse_char(inputstr,pos,'{');
    object = [];
    type='';
    count=-1;
    [cc, pos]=next_char(inputstr,pos);
    if(cc == '$')
        type=inputstr(pos+1); % TODO
        pos=pos+2;
    end
    [cc, pos]=next_char(inputstr,pos);
    if(cc == '#')
        pos=pos+1;
        [val,pos]=parse_number(inputstr, pos, varargin{:});
        count=double(val);
    end
    [cc, pos]=next_char(inputstr,pos);
    if cc ~= '}'
        num=0;
        while 1
            if(jsonopt('NameIsString',0,varargin{:}))
                [str, pos] = parseStr(inputstr, pos, varargin{:});
            else
                [str, pos] = parse_name(inputstr, pos, varargin{:});
            end
            if isempty(str)
                error_pos('Name of value at position %d cannot be empty', inputstr, pos);
            end
            [val, pos] = parse_value(inputstr, pos, varargin{:});
            num=num+1;
            object.(encodevarname(str,varargin{:}))=val;
            [cc, pos]=next_char(inputstr,pos);
            if cc == '}' || (count>=0 && num>=count)
                break;
            end
        end
    end
    if(count==-1)
        pos=parse_char(inputstr, pos, '}');
    end
end

%%-------------------------------------------------------------------------
function [cid,len]=elem_info(inputstr, pos, type)
    id=strfind('iUIlLdD',type);
    dataclass={'int8','uint8','int16','int32','int64','single','double'};
    bytelen=[1,1,2,4,8,4,8];
    if(id>0)
        cid=dataclass{id};
        len=bytelen(id);
    else
        error_pos('unsupported type at position %d',inputstr, pos);
    end
end
