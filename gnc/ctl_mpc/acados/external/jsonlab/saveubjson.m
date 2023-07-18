function json=saveubjson(rootname,obj,varargin)
%
% json=saveubjson(obj)
%    or
% json=saveubjson(rootname,obj,filename)
% json=saveubjson(rootname,obj,opt)
% json=saveubjson(rootname,obj,'param1',value1,'param2',value2,...)
%
% convert a MATLAB object  (cell, struct, array, table, map, handles ...) 
% into a Universal Binary JSON (UBJSON) or a MessagePack binary stream
%
% author: Qianqian Fang (q.fang <at> neu.edu)
% initially created on 2013/08/17
%
% input:
%      rootname: the name of the root-object, when set to '', the root name
%           is ignored, however, when opt.ForceRootName is set to 1 (see below),
%           the MATLAB variable name will be used as the root name.
%      obj: a MATLAB object (array, cell, cell array, struct, struct array,
%           class instance)
%      filename: a string for the file name to save the output UBJSON data
%      opt: a struct for additional options, ignore to use default values.
%           opt can have the following fields (first in [.|.] is the default)
%
%           FileName [''|string]: a file name to save the output JSON data
%           ArrayToStruct[0|1]: when set to 0, saveubjson outputs 1D/2D
%                         array in JSON array format; if sets to 1, an
%                         array will be shown as a struct with fields
%                         "_ArrayType_", "_ArraySize_" and "_ArrayData_"; for
%                         sparse arrays, the non-zero elements will be
%                         saved to "_ArrayData_" field in triplet-format i.e.
%                         (ix,iy,val) and "_ArrayIsSparse_":true will be added
%                         with a value of 1; for a complex array, the 
%                         "_ArrayData_" array will include two rows 
%                         (4 for sparse) to record the real and imaginary 
%                         parts, and also "_ArrayIsComplex_":true is added. 
%          NestArray    [0|1]: If set to 1, use nested array constructs
%                         to store N-dimensional arrays (compatible with 
%                         UBJSON specification Draft 12); if set to 0,
%                         use the JData (v0.5) optimized N-D array header;
%                         NestArray is automatically set to 1 when
%                         MessagePack is set to 1
%          ParseLogical [1|0]: if this is set to 1, logical array elem
%                         will use true/false rather than 1/0.
%          SingletArray [0|1]: if this is set to 1, arrays with a single
%                         numerical element will be shown without a square
%                         bracket, unless it is the root object; if 0, square
%                         brackets are forced for any numerical arrays.
%          SingletCell  [1|0]: if 1, always enclose a cell with "[]" 
%                         even it has only one element; if 0, brackets
%                         are ignored when a cell has only 1 element.
%          ForceRootName [0|1]: when set to 1 and rootname is empty, saveubjson
%                         will use the name of the passed obj variable as the 
%                         root object name; if obj is an expression and 
%                         does not have a name, 'root' will be used; if this 
%                         is set to 0 and rootname is empty, the root level 
%                         will be merged down to the lower level.
%          JSONP [''|string]: to generate a JSONP output (JSON with padding),
%                         for example, if opt.JSON='foo', the JSON data is
%                         wrapped inside a function call as 'foo(...);'
%          UnpackHex [1|0]: conver the 0x[hex code] output by loadjson 
%                         back to the string form
%          Compression  'zlib', 'gzip', 'lzma', 'lzip', 'lz4' or 'lz4hc': specify array 
%                         compression method; currently only supports 6 methods. The
%                         data compression only applicable to numerical arrays 
%                         in 3D or higher dimensions, or when ArrayToStruct
%                         is 1 for 1D or 2D arrays. If one wants to
%                         compress a long string, one must convert
%                         it to uint8 or int8 array first. The compressed
%                         array uses three extra fields
%                         "_ArrayZipType_": the opt.Compression value. 
%                         "_ArrayZipSize_": a 1D interger array to
%                            store the pre-compressed (but post-processed)
%                            array dimensions, and 
%                         "_ArrayZipData_": the binary stream of
%                            the compressed binary array data WITHOUT
%                            'base64' encoding
%          CompressArraySize [100|int]: only to compress an array if the total 
%                         element count is larger than this number.
%          MessagePack [0|1]: output MessagePack (https://msgpack.org/)
%                         binary stream instead of UBJSON
%          FormatVersion [2|float]: set the JSONLab output version; since
%                         v2.0, JSONLab uses JData specification Draft 2
%                         for output format, it is incompatible with all
%                         previous releases; if old output is desired,
%                         please set FormatVersion to 1.9 or earlier.
%          Debug [0|1]: output binary numbers in <%g> format for debugging
%          PreEncode [1|0]: if set to 1, call jdataencode first to preprocess
%                         the input data before saving
%
%        opt can be replaced by a list of ('param',value) pairs. The param 
%        string is equivallent to a field in opt and is case sensitive.
% output:
%      json: a binary string in the UBJSON format (see http://ubjson.org)
%
% examples:
%      jsonmesh=struct('MeshVertex3',[0 0 0;1 0 0;0 1 0;1 1 0;0 0 1;1 0 1;0 1 1;1 1 1],... 
%               'MeshTet4',[1 2 4 8;1 3 4 8;1 2 6 8;1 5 6 8;1 5 7 8;1 3 7 8],...
%               'MeshTri3',[1 2 4;1 2 6;1 3 4;1 3 7;1 5 6;1 5 7;...
%                          2 8 4;2 8 6;3 8 4;3 8 7;5 8 6;5 8 7],...
%               'MeshCreator','FangQ','MeshTitle','T6 Cube',...
%               'SpecialData',[nan, inf, -inf]);
%      saveubjson(jsonmesh)
%      saveubjson('',jsonmesh,'meshdata.ubj')
%      saveubjson('mesh1',jsonmesh,'FileName','meshdata.msgpk','MessagePack',1)
%
% license:
%     BSD or GPL version 3, see LICENSE_{BSD,GPLv3}.txt files for details
%
% -- this function is part of JSONLab toolbox (http://iso2mesh.sf.net/cgi-bin/index.cgi?jsonlab)
%
global ismsgpack

if(nargin==1)
   varname=inputname(1);
   obj=rootname;
   if(isempty(varname)) 
      varname='root';
   end
   rootname=varname;
else
   varname=inputname(2);
end
if(length(varargin)==1 && ischar(varargin{1}))
   opt=struct('filename',varargin{1});
else
   opt=varargin2struct(varargin{:});
end
opt.IsOctave=isoctavemesh;

opt.compression=jsonopt('Compression','',opt);
opt.nestarray=jsonopt('NestArray',0,opt);
opt.formatversion=jsonopt('FormatVersion',2,opt);
opt.compressarraysize=jsonopt('CompressArraySize',100,opt);
opt.singletcell=jsonopt('SingletCell',1,opt);
opt.singletarray=jsonopt('SingletArray',0,opt);
opt.arraytostruct=jsonopt('ArrayToStruct',0,opt);
opt.debug=jsonopt('Debug',0,opt);
opt.messagepack=jsonopt('MessagePack',0,opt);
opt.num2cell_=0;

if(jsonopt('PreEncode',1,opt))
    obj=jdataencode(obj,'Base64',0,'UseArrayZipSize',opt.messagepack,opt);
end

dozip=opt.compression;
if(~isempty(dozip))
    if(isempty(strmatch(dozip,{'zlib','gzip','lzma','lzip','lz4','lz4hc'})))
        error('compression method "%s" is not supported',dozip);
    end
    if(exist('zmat','file')~=2 && exist('zmat','file')~=3)
        try
            error(javachk('jvm'));
            try
               base64decode('test');
            catch
               matlab.net.base64decode('test');
            end
        catch
            error('java-based compression is not supported');
        end
    end    
end

ismsgpack=opt.messagepack + bitshift(opt.debug,1);
opt.messagepack=ismsgpack;

if(~bitget(ismsgpack, 1))
    opt.IM_='UiIlL';
    opt.FM_='dD';
    opt.FTM_='FT';
    opt.SM_='CS';
    opt.ZM_='Z';
    opt.OM_={'{','}'};
    opt.AM_={'[',']'};
else
    opt.IM_=char([hex2dec('cc') hex2dec('d0') hex2dec('d1') hex2dec('d2') hex2dec('d3')]);
    opt.FM_=char([hex2dec('ca') hex2dec('cb')]);
    opt.FTM_=char([hex2dec('c2') hex2dec('c3')]);
    opt.SM_=char([hex2dec('a1') hex2dec('db')]);
    opt.ZM_=char(hex2dec('c0'));
    opt.OM_={char(hex2dec('df')),''};
    opt.AM_={char(hex2dec('dd')),''};
end

rootisarray=0;
rootlevel=1;
forceroot=jsonopt('ForceRootName',0,opt);

if((isnumeric(obj) || islogical(obj) || ischar(obj) || isstruct(obj) || ...
        iscell(obj) || isobject(obj)) && isempty(rootname) && forceroot==0)
    rootisarray=1;
    rootlevel=0;
else
    if(isempty(rootname))
        rootname=varname;
    end
end

if((isstruct(obj) || iscell(obj))&& isempty(rootname) && forceroot)
    rootname='root';
end

json=obj2ubjson(rootname,obj,rootlevel,opt);

if(~rootisarray)
    if(bitget(ismsgpack, 1))
        json=[char(129) json opt.OM_{2}];
    else
        json=[opt.OM_{1} json opt.OM_{2}];
    end
end

jsonp=jsonopt('JSONP','',opt);
if(~isempty(jsonp))
    json=[jsonp '(' json ')'];
end

% save to a file if FileName is set, suggested by Patrick Rapin
filename=jsonopt('FileName','',opt);
if(~isempty(filename))
    fid = fopen(filename, 'wb');
    fwrite(fid,json);
    fclose(fid);
end

%%-------------------------------------------------------------------------
function txt=obj2ubjson(name,item,level,varargin)

if(iscell(item) || isa(item,'string'))
    txt=cell2ubjson(name,item,level,varargin{:});
elseif(isstruct(item))
    txt=struct2ubjson(name,item,level,varargin{:});
elseif(isnumeric(item) || islogical(item))
    txt=mat2ubjson(name,item,level,varargin{:});
elseif(ischar(item))
    txt=str2ubjson(name,item,level,varargin{:});
elseif(isa(item,'function_handle'))
    txt=struct2ubjson(name,functions(item),level,varargin{:});
elseif(isa(item,'containers.Map'))
    txt=map2ubjson(name,item,level,varargin{:});
elseif(isa(item,'categorical'))
    txt=cell2ubjson(name,cellstr(item),level,varargin{:});
elseif(isa(item,'table'))
    txt=matlabtable2ubjson(name,item,level,varargin{:});
elseif(isa(item,'graph') || isa(item,'digraph'))
    txt=struct2ubjson(name,jdataencode(item),level,varargin{:});
elseif(isobject(item))
    txt=matlabobject2ubjson(name,item,level,varargin{:});
else
    txt=any2ubjson(name,item,level,varargin{:});
end

%%-------------------------------------------------------------------------
function txt=cell2ubjson(name,item,level,varargin)
txt='';
if(~iscell(item))
        error('input is not a cell');
end
isnum2cell=varargin{1}.num2cell_;
if(isnum2cell)
    item=squeeze(item);
else
    format=varargin{1}.formatversion;
    if(format>1.9 && ~isvector(item))
        item=permute(item,ndims(item):-1:1);
    end
end

dim=size(item);
if(ndims(squeeze(item))>2) % for 3D or higher dimensions, flatten to 2D for now
    item=reshape(item,dim(1),numel(item)/dim(1));
    dim=size(item);
end
bracketlevel=~varargin{1}.singletcell;
Zmarker=varargin{1}.ZM_;
Imarker=varargin{1}.IM_;
Amarker=varargin{1}.AM_;

if(~strcmp(Amarker{1},'['))
    am0=Imsgpk_(dim(2),Imarker,220,144);
else
    am0=Amarker{1};
end
len=numel(item); % let's handle 1D cell first
if(len>bracketlevel) 
    if(~isempty(name))
        txt=[N_(decodevarname(name,varargin{:})) am0]; name=''; 
    else
        txt=am0; 
    end
elseif(len==0)
    if(~isempty(name))
        txt=[N_(decodevarname(name,varargin{:})) Zmarker]; name=''; 
    else
        txt=Zmarker; 
    end
end
if(~strcmp(Amarker{1},'['))
    am0=Imsgpk_(dim(1),Imarker,220,144);
end
for j=1:dim(2)
    if(dim(1)>1)
        txt=[txt am0];
    end
    for i=1:dim(1)
       txt=[txt obj2ubjson(name,item{i,j},level+(len>bracketlevel),varargin{:})];
    end
    if(dim(1)>1)
        txt=[txt Amarker{2}];
    end
end
if(len>bracketlevel)
    txt=[txt Amarker{2}];
end

%%-------------------------------------------------------------------------
function txt=struct2ubjson(name,item,level,varargin)
txt='';
if(~isstruct(item))
	error('input is not a struct');
end
dim=size(item);
if(ndims(squeeze(item))>2) % for 3D or higher dimensions, flatten to 2D for now
    item=reshape(item,dim(1),numel(item)/dim(1));
    dim=size(item);
end
len=numel(item);
forcearray= (len>1 || (varargin{1}.singletarray==1 && level>0));
Imarker=varargin{1}.IM_;
Amarker=varargin{1}.AM_;
Omarker=varargin{1}.OM_;

if(~strcmp(Amarker{1},'['))
    am0=Imsgpk_(dim(2),Imarker,220,144);
else
    am0=Amarker{1};
end

if(~isempty(name)) 
    if(forcearray)
        txt=[N_(decodevarname(name,varargin{:})) am0];
    end
else
    if(forcearray)
        txt=am0;
    end
end
if(~strcmp(Amarker{1},'['))
    am0=Imsgpk_(dim(1),Imarker,220,144);
end
for j=1:dim(2)
  if(dim(1)>1)
      txt=[txt am0];
  end
  for i=1:dim(1)
     names = fieldnames(item(i,j));
     if(~strcmp(Omarker{1},'{'))
        om0=Imsgpk_(length(names),Imarker,222,128);
     else
        om0=Omarker{1};
     end
     if(~isempty(name) && len==1 && ~forcearray)
        txt=[txt N_(decodevarname(name,varargin{:})) om0]; 
     else
        txt=[txt om0]; 
     end
     if(~isempty(names))
       for e=1:length(names)
	     txt=[txt obj2ubjson(names{e},item(i,j).(names{e}),...
             level+(dim(1)>1)+1+forcearray,varargin{:})];
       end
     end
     txt=[txt Omarker{2}];
  end
  if(dim(1)>1)
      txt=[txt Amarker{2}];
  end
end
if(forcearray)
    txt=[txt Amarker{2}];
end

%%-------------------------------------------------------------------------
function txt=map2ubjson(name,item,level,varargin)
txt='';
if(~isa(item,'containers.Map'))
	error('input is not a struct');
end
dim=size(item);
names = keys(item);
val= values(item);
Omarker=varargin{1}.OM_;
Imarker=varargin{1}.IM_;

if(~strcmp(Omarker{1},'{'))
    om0=Imsgpk_(length(names),Imarker,222,128);
else
    om0=Omarker{1};
end
len=prod(dim);
forcearray= (len>1 || (varargin{1}.singletarray==1 && level>0));

if(~isempty(name)) 
    if(forcearray)
        txt=[N_(decodevarname(name,varargin{:})) om0];
    end
else
    if(forcearray)
        txt=om0;
    end
end
for i=1:dim(1)
    if(~isempty(names{i}))
	    txt=[txt obj2ubjson(names{i},val{i},...
             level+(dim(1)>1),varargin{:})];
    end
end
if(forcearray)
    txt=[txt Omarker{2}];
end

%%-------------------------------------------------------------------------
function txt=str2ubjson(name,item,level,varargin)
txt='';
if(~ischar(item))
        error('input is not a string');
end
item=reshape(item, max(size(item),[1 0]));
len=size(item,1);
Amarker=varargin{1}.AM_;
Imarker=varargin{1}.IM_;

if(~strcmp(Amarker{1},'['))
    am0=Imsgpk_(len,Imarker,220,144);
else
    am0=Amarker{1};
end
if(~isempty(name)) 
    if(len>1)
        txt=[N_(decodevarname(name,varargin{:})) am0];
    end
else
    if(len>1)
        txt=am0;
    end
end
for e=1:len
    val=item(e,:);
    if(len==1)
        obj=[N_(decodevarname(name,varargin{:})) '' '',S_(val),''];
        if(isempty(name))
            obj=['',S_(val),''];
        end
        txt=[txt,'',obj];
    else
        txt=[txt,'',['',S_(val),'']];
    end
end
if(len>1)
    txt=[txt Amarker{2}];
end

%%-------------------------------------------------------------------------
function txt=mat2ubjson(name,item,level,varargin)
if(~isnumeric(item) && ~islogical(item))
        error('input is not an array');
end

dozip=varargin{1}.compression;
zipsize=varargin{1}.compressarraysize;
format=varargin{1}.formatversion;

Zmarker=varargin{1}.ZM_;
FTmarker=varargin{1}.FTM_;
Imarker=varargin{1}.IM_;
Omarker=varargin{1}.OM_;
isnest=varargin{1}.nestarray;
ismsgpack=varargin{1}.messagepack;

if(ismsgpack)
    isnest=1;
end
if((length(size(item))>2 && isnest==0)  || issparse(item) || ~isreal(item) || ...
       varargin{1}.arraytostruct || (~isempty(dozip) && numel(item)>zipsize ...
       && strcmp('_ArrayZipData_',decodevarname(name,varargin{:}))==0))
      cid=I_(uint32(max(size(item))),Imarker,varargin{:});
      if(isempty(name))
    	txt=[Omarker{1} N_('_ArrayType_'),S_(class(item)),N_('_ArraySize_'),I_a(size(item),cid(1),Imarker,varargin{:}) ];
      else
          if(isempty(item))
              txt=[N_(decodevarname(name,varargin{:})),Zmarker];
              return;
          else
    	      txt=[N_(decodevarname(name,varargin{:})),Omarker{1},N_('_ArrayType_'),S_(class(item)),N_('_ArraySize_'),I_a(size(item),cid(1),Imarker,varargin{:})];
          end
      end
      childcount=2;
else
    if(isempty(name))
    	txt=matdata2ubjson(item,level+1,varargin{:});
    else
        if(numel(item)==1 && varargin{1}.singletarray==0)
            numtxt=regexprep(regexprep(matdata2ubjson(item,level+1,varargin{:}),'^\[',''),']$','');
           	txt=[N_(decodevarname(name,varargin{:})) numtxt];
        else
    	    txt=[N_(decodevarname(name,varargin{:})),matdata2ubjson(item,level+1,varargin{:})];
        end
    end
    return;
end
if(issparse(item))
    [ix,iy]=find(item);
    data=full(item(find(item)));
    if(~isreal(item))
       data=[real(data(:)),imag(data(:))];
       if(size(item,1)==1)
           % Kludge to have data's 'transposedness' match item's.
           % (Necessary for complex row vector handling below.)
           data=data';
       end
       txt=[txt,N_('_ArrayIsComplex_'),FTmarker(2)];
       childcount=childcount+1;
    end
    txt=[txt,N_('_ArrayIsSparse_'),FTmarker(2)];
    childcount=childcount+1;
    if(~isempty(dozip) && numel(data*2)>zipsize)
        if(size(item,1)==1)
            % Row vector, store only column indices.
            fulldata=[iy(:),data'];
        elseif(size(item,2)==1)
            % Column vector, store only row indices.
            fulldata=[ix,data];
        else
            % General case, store row and column indices.
            fulldata=[ix,iy,data];
        end
        cid=I_(uint32(max(size(fulldata))),Imarker,varargin{:});
        txt=[txt, N_('_ArrayZipSize_'),I_a(size(fulldata),cid(1),Imarker,varargin{:})];
        txt=[txt, N_('_ArrayZipType_'),S_(dozip)];
	    compfun=str2func([dozip 'encode']);
	    txt=[txt,N_('_ArrayZipData_'), I_a(compfun(typecast(fulldata(:),'uint8')),Imarker(1),Imarker,varargin{:})];
        childcount=childcount+3;
    else
        if(size(item,1)==1)
            % Row vector, store only column indices.
            fulldata=[iy(:),data'];
        elseif(size(item,2)==1)
            % Column vector, store only row indices.
            fulldata=[ix,data];
        else
            % General case, store row and column indices.
            fulldata=[ix,iy,data];
        end
        if(ismsgpack)
            cid=I_(uint32(max(size(fulldata))),Imarker,varargin{:});
            txt=[txt,N_('_ArrayZipSize_'),I_a(size(fulldata),cid(1),Imarker,varargin{:})];
            childcount=childcount+1;
        end
        varargin{:}.ArrayToStruct=0;
        txt=[txt,N_('_ArrayData_'),...
               cell2ubjson('',num2cell(fulldata',2)',level+2,varargin{:})];
        childcount=childcount+1;
    end
else
    if(format>1.9)
        item=permute(item,ndims(item):-1:1);
    end
    if(~isempty(dozip) && numel(item)>zipsize)
        if(isreal(item))
            fulldata=item(:)';
            if(islogical(fulldata))
                fulldata=uint8(fulldata);
            end
        else
            txt=[txt,N_('_ArrayIsComplex_'),FTmarker(2)];
            childcount=childcount+1;
            fulldata=[real(item(:)) imag(item(:))];
        end
        cid=I_(uint32(max(size(fulldata))),Imarker,varargin{:});
        txt=[txt, N_('_ArrayZipSize_'),I_a(size(fulldata),cid(1),Imarker,varargin{:})];
        txt=[txt, N_('_ArrayZipType_'),S_(dozip)];
	    compfun=str2func([dozip 'encode']);
	    txt=[txt,N_('_ArrayZipData_'), I_a(compfun(typecast(fulldata(:),'uint8')),Imarker(1),Imarker,varargin{:})];
        childcount=childcount+3;
    else
        if(ismsgpack)
            cid=I_(uint32(length(item(:))),Imarker,varargin{:});
            txt=[txt,N_('_ArrayZipSize_'),I_a([~isreal(item)+1 length(item(:))],cid(1),Imarker,varargin{:})];
            childcount=childcount+1;
        end
        if(isreal(item))
            txt=[txt,N_('_ArrayData_'),...
                matdata2ubjson(item(:)',level+2,varargin{:})];
            childcount=childcount+1;
        else
            txt=[txt,N_('_ArrayIsComplex_'),FTmarker(2)];
            txt=[txt,N_('_ArrayData_'),...
                matdata2ubjson([real(item(:)) imag(item(:))]',level+2,varargin{:})];
            childcount=childcount+2;
        end
    end
end
if(Omarker{1}~='{')
    idx=find(txt==Omarker{1},1,'first');
    if(~isempty(idx))
        txt=[txt(1:idx-1) Imsgpk_(childcount,Imarker,222,128) txt(idx+1:end)];
    end
end
txt=[txt,Omarker{2}];

%%-------------------------------------------------------------------------
function txt=matlabtable2ubjson(name,item,level,varargin)
st=containers.Map();
st('_TableRecords_')=table2cell(item);
st('_TableRows_')=item.Properties.RowNames';
st('_TableCols_')=item.Properties.VariableNames;
if(isempty(name))
    txt=map2ubjson(name,st,level,varargin{:});
else
    temp=struct(name,struct());
    temp.(name)=st;
    txt=map2ubjson(name,temp.(name),level,varargin{:});
end

%%-------------------------------------------------------------------------
function txt=matlabobject2ubjson(name,item,level,varargin)
try
    if numel(item) == 0 %empty object
        st = struct();
    elseif numel(item) == 1 %
        txt = str2ubjson(name, char(item), level, varargin(:));
        return
    else
            propertynames = properties(item);
            for p = 1:numel(propertynames)
                for o = numel(item):-1:1 % aray of objects
                    st(o).(propertynames{p}) = item(o).(propertynames{p});
                end
            end
    end
    txt = struct2ubjson(name,st,level,varargin{:});
catch
    txt = any2ubjson(name,item, level, varargin(:));
end

%%-------------------------------------------------------------------------
function txt=matdata2ubjson(mat,level,varargin)
Zmarker=varargin{1}.ZM_;
if(isempty(mat))
    txt=Zmarker;
    return;
end
dozip=varargin{1}.compression;
zipsize=varargin{1}.compressarraysize;

FTmarker=varargin{1}.FTM_;
Imarker=varargin{1}.IM_;
Omarker=varargin{1}.OM_;
Fmarker=varargin{1}.FM_;
Amarker=varargin{1}.AM_;

isnest=varargin{:}.nestarray;
ismsgpack=varargin{1}.messagepack;
format=varargin{1}.formatversion;
isnum2cell=varargin{1}.num2cell_;

if(ismsgpack)
    isnest=1;
end

if(~isvector(mat) && isnest==1)
   if(format>1.9 && isnum2cell==0)
        mat=permute(mat,ndims(mat):-1:1);
   end
   varargin{1}.num2cell_=1;
end

type='';

if(isa(mat,'integer') || isinteger(mat) || (isfloat(mat) && all(mod(mat(:),1) == 0)))
    if(~any(mat<0))
       if(max(mat(:))<2^8)
           type=Imarker(1);
       end
    end
    if(isempty(type))
        % todo - need to consider negative ones separately
        maxval=max(double(mat(:)));
        if(min(double(mat(:)))>=0 && maxval<=255)
                type='U';
        else
                id= histc(abs(maxval),[0 2^7 2^15 2^31 2^63]);
                if(isempty(id~=0))
                        error('high-precision data is not yet supported');
                end
                key=Imarker(2:end);
                type=key(id~=0);
        end
    end
    if(~isvector(mat) && isnest==1)
        txt=cell2ubjson('',num2cell(mat,1),level,varargin{:});
    elseif(~ismsgpack || size(mat,1)==1)
        txt=I_a(mat(:),type,Imarker,size(mat),varargin{:});
    else
        txt=cell2ubjson('',num2cell(mat,2),level,varargin{:});
    end
elseif(islogical(mat))
    logicalval=FTmarker;
    if(numel(mat)==1)
        txt=logicalval(mat+1);
    else
        if(~isvector(mat) && isnest==1)
            txt=cell2ubjson('',num2cell(uint8(mat,1),level,varargin{:}));
        else
            txt=I_a(uint8(mat(:)),Imarker(1),Imarker,size(mat),varargin{:});
        end
    end
else
    am0=Amarker{1};
    if(Amarker{1}~='[')
        am0=char(145);
    end
    if(numel(mat)==1)
        txt=[am0 D_(mat,Fmarker,varargin{:}) Amarker{2}];
    else
        if(~isvector(mat) && isnest==1)
            txt=cell2ubjson('',num2cell(mat,1),level,varargin{:});
        else
            txt=D_a(mat(:),Fmarker(2),Fmarker,size(mat),varargin{:});
        end
    end
end

%%-------------------------------------------------------------------------
function val=N_(str)
global ismsgpack
if(~bitget(ismsgpack, 1))
    val=[I_(int32(length(str)),'UiIlL',struct('debug',bitget(ismsgpack,2))) str];
else
    val=S_(str);
end
%%-------------------------------------------------------------------------
function val=S_(str)
global ismsgpack
if(bitget(ismsgpack, 1))
    Smarker=char([161,219]);
    Imarker=char([204,208:211]);
else
    Smarker='CS';
    Imarker='UiIlL';
end
if(length(str)==1)
  val=[Smarker(1) str];
else
    if(bitget(ismsgpack, 1))
        val=[Imsgpk_(length(str),Imarker,218,160) str];
    else
        val=['S' I_(int32(length(str)),Imarker,struct('debug',bitget(ismsgpack,2))) str];
    end
end

%%-------------------------------------------------------------------------
function val=Imsgpk_(num,markers,base1,base0)
if(num<16)
    val=char(uint8(num)+uint8(base0));
    return;
end
val=I_(uint32(num),markers);
if(val(1)>char(210))
    num=uint32(num);
    val=[char(210) data2byte(swapbytes(cast(num,'uint32')),'uint8')];
elseif(val(1)<char(209))
    num=uint16(num);
    val=[char(209) data2byte(swapbytes(cast(num,'uint16')),'uint8')];
end
val(1)=char(val(1)-209+base1);

%%-------------------------------------------------------------------------
function val=I_(num, markers, varargin)
if(~isinteger(num))
    error('input is not an integer');
end
Imarker='UiIlL';
if(nargin>=2)
    Imarker=markers;
end
isdebug=0;
if(nargin>=3)
    isdebug=varargin{1}.debug;
end

if(Imarker(1)~='U')
    if(num>=0 && num<127)
       val=uint8(num);
       return;
    end
    if(num<0 && num>=-31)
       val=typecast(int8(num), 'uint8');
       return;
    end
end
if(Imarker(1)~='U' && num<0 && num<127)
   val=[data2byte((swapbytes(cast(num,'uint8')) & 127),'uint8')];
   return;
end
if(num>=0 && num<255)
   if(isdebug)
       val=[Imarker(1) sprintf('<%d>',num)];
   else
       val=[Imarker(1) data2byte(swapbytes(cast(num,'uint8')),'uint8')];
   end
   return;
end
key=Imarker(2:end);
cid={'int8','int16','int32','int64'};
for i=1:4
  if((num>0 && num<2^(i*8-1)) || (num<0 && num>=-2^(i*8-1)))
    if(isdebug)
        val=[key(i) sprintf('<%d>',num)];
    else
        val=[key(i) data2byte(swapbytes(cast(num,cid{i})),'uint8')];
    end
    return;
  end
end
error('unsupported integer');

%%-------------------------------------------------------------------------
function val=D_(num, markers, varargin)
if(~isfloat(num))
    error('input is not a float');
end
isdebug=varargin{1}.debug;
if(isdebug)
    output=sprintf('<%g>',num);
else
    output=data2byte(swapbytes(num),'uint8');
end
Fmarker='dD';
if(nargin>=2)
    Fmarker=markers;
end
if(isa(num,'single'))
  val=[Fmarker(1) output(:)'];
else
  val=[Fmarker(2) output(:)'];
end
%%-------------------------------------------------------------------------
function data=I_a(num,type,markers,dim,varargin)
Imarker='UiIlL';
Amarker={'[',']'};
if(nargin>=3)
    Imarker=markers;
end
if(Imarker(1)~='U' && type<=127)
    type=char(204);
end
id=find(ismember(Imarker,type));

if(id==0)
  error('unsupported integer array');
end

% based on UBJSON specs, all integer types are stored in big endian format

if(id==2)
  data=data2byte(swapbytes(int8(num)),'uint8');
  blen=1;
elseif(id==1)
  data=data2byte(swapbytes(uint8(num)),'uint8');
  blen=1;
elseif(id==3)
  data=data2byte(swapbytes(int16(num)),'uint8');
  blen=2;
elseif(id==4)
  data=data2byte(swapbytes(int32(num)),'uint8');
  blen=4;
elseif(id==5)
  data=data2byte(swapbytes(int64(num)),'uint8');
  blen=8;
end
if(isstruct(dim))
    varargin={dim};
end

isnest=varargin{1}.nestarray;
isdebug=varargin{1}.debug;
if(isdebug)
    output=sprintf('<%g>',num);
else
    output=data(:);
end

if(isnest==0 && numel(num)>1 && Imarker(1)=='U')
  if(nargin>=4 && ~isstruct(dim) && (length(dim)==1 || (length(dim)>=2 && prod(dim)~=dim(2))))
      cid=I_(uint32(max(dim)),Imarker,varargin{:});
      data=['$' type '#' I_a(dim,cid(1),Imarker,varargin{:}) output(:)'];
  else
      data=['$' type '#' I_(int32(numel(data)/blen),Imarker,varargin{:}) output(:)'];
  end
  data=['[' data(:)'];
else
  am0=Amarker{1};
  if(Imarker(1)~='U')
      Amarker={char(hex2dec('dd')),''};
      am0=Imsgpk_(numel(num),Imarker,220,144);
  end  
  if(isdebug)
      data=sprintf([type '<%g>'],num);
  else
      data=reshape(data,blen,numel(data)/blen);
      data(2:blen+1,:)=data;
      data(1,:)=type;
  end
  data=[am0 data(:)' Amarker{2}];
end
%%-------------------------------------------------------------------------
function data=D_a(num,type,markers,dim,varargin)
Fmarker='dD';
Imarker='UiIlL';
Amarker={'[',']'};
if(nargin>=3)
    Fmarker=markers;
end
id=find(ismember(Fmarker,type));

if(id==0)
  error('unsupported float array');
end

if(id==1)
  data=data2byte(swapbytes(single(num)),'uint8');
elseif(id==2)
  data=data2byte(swapbytes(double(num)),'uint8');
end

isnest=varargin{1}.nestarray;
isdebug=varargin{1}.debug;
if(isdebug)
    output=sprintf('<%g>',num);
else
    output=data(:);
end

if(isnest==0 && numel(num)>1 && Fmarker(1)=='d')
  if(nargin>=4 && (length(dim)==1 || (length(dim)>=2 && prod(dim)~=dim(2))))
      cid=I_(uint32(max(dim)));
      data=['$' type '#' I_a(dim,cid(1),Imarker,varargin{:}) output(:)'];
  else
      data=['$' type '#' I_(int32(numel(data)/(id*4)),varargin{:}.IM_,varargin{:}) output(:)'];
  end
  data=['[' data];
else
  am0=Amarker{1};
  if(Fmarker(1)~='d')
      Amarker={char(hex2dec('dd')),''};
      am0=Imsgpk_(numel(num),char([204,208:211]),220,144);
  end
  if(isdebug)
      data=sprintf([type '<%g>'],num);
  else
      data=reshape(data,(id*4),length(data)/(id*4));
      data(2:(id*4+1),:)=data;
      data(1,:)=type;
  end
  data=[am0 data(:)' Amarker{2}];
end

%%-------------------------------------------------------------------------
function txt=any2ubjson(name,item,level,varargin)
st=containers.Map();
st('_DataInfo_')=struct('MATLABObjectClass',class(item),'MATLABObjectSize',size(item));;
st('_ByteStream_')=getByteStreamFromArray(item);

if(isempty(name))
    txt=map2ubjson(name,st,level,varargin{:});
else
    temp=struct(name,struct());
    temp.(name)=st;
    txt=map2ubjson(name,temp.(name),level,varargin{:});
end

%%-------------------------------------------------------------------------
function bytes=data2byte(varargin)
bytes=typecast(varargin{:});
bytes=char(bytes(:)');
