function json=savejson(rootname,obj,varargin)
%
% json=savejson(obj)
%    or
% json=savejson(rootname,obj,filename)
% json=savejson(rootname,obj,opt)
% json=savejson(rootname,obj,'param1',value1,'param2',value2,...)
%
% convert a MATLAB object (cell, struct or array) into a JSON (JavaScript
% Object Notation) string
%
% author: Qianqian Fang (q.fang <at> neu.edu)
% initially created on 2011/09/09
%
% input:
%      rootname: the name of the root-object, when set to '', the root name
%           is ignored, however, when opt.ForceRootName is set to 1 (see below),
%           the MATLAB variable name will be used as the root name.
%      obj: a MATLAB object (array, cell, cell array, struct, struct array,
%           class instance).
%      filename: a string for the file name to save the output JSON data.
%      opt: a struct for additional options, ignore to use default values.
%           opt can have the following fields (first in [.|.] is the default)
%
%           FileName [''|string]: a file name to save the output JSON data
%           FloatFormat ['%.10g'|string]: format to show each numeric element
%                         of a 1D/2D array;
%           IntFormat ['%d'|string]: format to display integer elements
%                         of a 1D/2D array;
%           ArrayIndent [1|0]: if 1, output explicit data array with
%                         precedent indentation; if 0, no indentation
%           ArrayToStruct[0|1]: when set to 0, savejson outputs 1D/2D
%                         array in JSON array format; if sets to 1, an
%                         array will be shown as a struct with fields
%                         "_ArrayType_", "_ArraySize_" and "_ArrayData_"; for
%                         sparse arrays, the non-zero elements will be
%                         saved to _ArrayData_ field in triplet-format i.e.
%                         (ix,iy,val) and "_ArrayIsSparse_" will be added
%                         with a value of 1; for a complex array, the 
%                         _ArrayData_ array will include two columns 
%                         (4 for sparse) to record the real and imaginary 
%                         parts, and also "_ArrayIsComplex_":1 is added. 
%           NestArray    [0|1]: If set to 1, use nested array constructs
%                         to store N-dimensional arrays; if set to 0,
%                         use the annotated array format defined in the
%                         JData Specification (Draft 1 or later).
%           ParseLogical [0|1]: if this is set to 1, logical array elem
%                         will use true/false rather than 1/0.
%           SingletArray [0|1]: if this is set to 1, arrays with a single
%                         numerical element will be shown without a square
%                         bracket, unless it is the root object; if 0, square
%                         brackets are forced for any numerical arrays.
%           SingletCell  [1|0]: if 1, always enclose a cell with "[]" 
%                         even it has only one element; if 0, brackets
%                         are ignored when a cell has only 1 element.
%           ForceRootName [0|1]: when set to 1 and rootname is empty, savejson
%                         will use the name of the passed obj variable as the 
%                         root object name; if obj is an expression and 
%                         does not have a name, 'root' will be used; if this 
%                         is set to 0 and rootname is empty, the root level 
%                         will be merged down to the lower level.
%           Inf ['"$1_Inf_"'|string]: a customized regular expression pattern
%                         to represent +/-Inf. The matched pattern is '([-+]*)Inf'
%                         and $1 represents the sign. For those who want to use
%                         1e999 to represent Inf, they can set opt.Inf to '$11e999'
%           NaN ['"_NaN_"'|string]: a customized regular expression pattern
%                         to represent NaN
%           JSONP [''|string]: to generate a JSONP output (JSON with padding),
%                         for example, if opt.JSONP='foo', the JSON data is
%                         wrapped inside a function call as 'foo(...);'
%           UnpackHex [1|0]: conver the 0x[hex code] output by loadjson 
%                         back to the string form
%           SaveBinary [0|1]: 1 - save the JSON file in binary mode; 0 - text mode.
%           Compact [0|1]: 1- out compact JSON format (remove all newlines and tabs)
%           Compression  'zlib', 'gzip', 'lzma', 'lzip', 'lz4' or 'lz4hc': specify array 
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
%                         "_ArrayZipData_": the "base64" encoded
%                             compressed binary array data. 
%           CompressArraySize [100|int]: only to compress an array if the total 
%                         element count is larger than this number.
%           FormatVersion [2|float]: set the JSONLab output version; since
%                         v2.0, JSONLab uses JData specification Draft 1
%                         for output format, it is incompatible with all
%                         previous releases; if old output is desired,
%                         please set FormatVersion to 1.9 or earlier.
%           Encoding ['']: json file encoding. Support all encodings of
%                         fopen() function
%           PreEncode [1|0]: if set to 1, call jdataencode first to preprocess
%                         the input data before saving
%
%        opt can be replaced by a list of ('param',value) pairs. The param 
%        string is equivallent to a field in opt and is case sensitive.
% output:
%      json: a string in the JSON format (see http://json.org)
%
% examples:
%      jsonmesh=struct('MeshNode',[0 0 0;1 0 0;0 1 0;1 1 0;0 0 1;1 0 1;0 1 1;1 1 1],... 
%               'MeshElem',[1 2 4 8;1 3 4 8;1 2 6 8;1 5 6 8;1 5 7 8;1 3 7 8],...
%               'MeshSurf',[1 2 4;1 2 6;1 3 4;1 3 7;1 5 6;1 5 7;...
%                          2 8 4;2 8 6;3 8 4;3 8 7;5 8 6;5 8 7],...
%               'MeshCreator','FangQ','MeshTitle','T6 Cube',...
%               'SpecialData',[nan, inf, -inf]);
%      savejson('jmesh',jsonmesh)
%      savejson('',jsonmesh,'ArrayIndent',0,'FloatFormat','\t%.5g')
%
% license:
%     BSD or GPL version 3, see LICENSE_{BSD,GPLv3}.txt files for details
%
% -- this function is part of JSONLab toolbox (http://iso2mesh.sf.net/cgi-bin/index.cgi?jsonlab)
%

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

opt.isoctave=isoctavemesh;

opt.compression=jsonopt('Compression','',opt);
opt.nestarray=jsonopt('NestArray',0,opt);
opt.compact=jsonopt('Compact',0,opt);
opt.singletcell=jsonopt('SingletCell',1,opt);
opt.singletarray=jsonopt('SingletArray',0,opt);
opt.formatversion=jsonopt('FormatVersion',2,opt);
opt.compressarraysize=jsonopt('CompressArraySize',100,opt);
opt.intformat=jsonopt('IntFormat','%d',opt);
opt.floatformat=jsonopt('FloatFormat','%.10g',opt);
opt.unpackhex=jsonopt('UnpackHex',1,opt);
opt.arraytostruct=jsonopt('ArrayToStruct',0,opt);
opt.parselogical=jsonopt('ParseLogical',0,opt);
opt.arrayindent=jsonopt('ArrayIndent',1,opt);
opt.num2cell_=0;

if(jsonopt('PreEncode',1,opt))
    obj=jdataencode(obj,'Base64',1,'UseArrayZipSize',0,opt);
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
    opt.Compression=dozip;
end

rootisarray=0;
rootlevel=1;
forceroot=jsonopt('ForceRootName',0,opt);
if((isnumeric(obj) || islogical(obj) || ischar(obj) || isstruct(obj) || ...
        iscell(obj) || isobject(obj) ) && isempty(rootname) && forceroot==0)
    rootisarray=1;
    rootlevel=0;
else
    if(isempty(rootname))
        rootname=varname;
    end
end
if(isa(obj,'containers.Map') && ~strcmp(obj.KeyType,'char'))
    rootisarray=0;
end
if((isstruct(obj) || iscell(obj))&& isempty(rootname) && forceroot)
    rootname='root';
end

whitespaces=struct('tab',sprintf('\t'),'newline',sprintf('\n'),'sep',sprintf(',\n'));
if(opt.compact==1)
    whitespaces=struct('tab','','newline','','sep',',');
end
if(~isfield(opt,'whitespaces_'))
    opt.whitespaces_=whitespaces;
end

nl=whitespaces.newline;

json=obj2json(rootname,obj,rootlevel,opt);

if(rootisarray)
    json=sprintf('%s%s',json,nl);
else
    json=sprintf('{%s%s%s}\n',nl,json,nl);
end

jsonp=jsonopt('JSONP','',opt);
if(~isempty(jsonp))
    json=sprintf('%s(%s);%s',jsonp,json,nl);
end

% save to a file if FileName is set, suggested by Patrick Rapin
filename=jsonopt('FileName','',opt);
if(~isempty(filename))
    if(jsonopt('SaveBinary',0,opt)==1)
        fid = fopen(filename, 'wb');
        fwrite(fid,json);
    else
        encoding = jsonopt('Encoding','',opt);
        if(isempty(encoding))
            fid = fopen(filename,'wt');
        else
            fid = fopen(filename,'wt','n',encoding);
        end
        fwrite(fid,json,'char');
    end
    fclose(fid);
end

%%-------------------------------------------------------------------------
function txt=obj2json(name,item,level,varargin)

if(iscell(item) || isa(item,'string'))
    txt=cell2json(name,item,level,varargin{:});
elseif(isstruct(item))
    txt=struct2json(name,item,level,varargin{:});
elseif(isnumeric(item) || islogical(item))
    txt=mat2json(name,item,level,varargin{:});
elseif(ischar(item))
    txt=str2json(name,item,level,varargin{:});
elseif(isa(item,'function_handle'))
    txt=struct2json(name,functions(item),level,varargin{:});
elseif(isa(item,'containers.Map'))
    txt=map2json(name,item,level,varargin{:});
elseif(isa(item,'categorical'))
    txt=cell2json(name,cellstr(item),level,varargin{:});
elseif(isa(item,'table'))
    txt=matlabtable2json(name,item,level,varargin{:});
elseif(isa(item,'graph') || isa(item,'digraph'))
    txt=struct2json(name,jdataencode(item),level,varargin{:});
elseif(isobject(item))
    txt=matlabobject2json(name,item,level,varargin{:});
else
    txt=any2json(name,item,level,varargin{:});
end

%%-------------------------------------------------------------------------
function txt=cell2json(name,item,level,varargin)
txt={};
if(~iscell(item) && ~isa(item,'string'))
        error('input is not a cell or string array');
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
len=numel(item);
ws=varargin{1}.whitespaces_;
padding0=repmat(ws.tab,1,level);
padding2=repmat(ws.tab,1,level+1);
nl=ws.newline;
bracketlevel=~varargin{1}.singletcell;
if(len>bracketlevel)
    if(~isempty(name))
        txt={padding0, '"', decodevarname(name,varargin{1}.unpackhex),'": [', nl}; name=''; 
    else
        txt={padding0, '[', nl};
    end
elseif(len==0)
    if(~isempty(name))
        txt={padding0, '"' decodevarname(name,varargin{1}.unpackhex) '": []'}; name=''; 
    else
        txt={padding0, '[]'};
    end
end
for j=1:dim(2)
    if(dim(1)>1)
        txt(end+1:end+3)={padding2,'[',nl};
    end
    for i=1:dim(1)
       txt{end+1}=obj2json(name,item{i,j},level+(dim(1)>1)+(len>bracketlevel),varargin{:});
       if(i<dim(1))
           txt(end+1:end+2)={',' nl};
       end
    end
    if(dim(1)>1)
        txt(end+1:end+3)={nl,padding2,']'};
    end
    if(j<dim(2))
        txt(end+1:end+2)={',' nl};
    end
    %if(j==dim(2)) txt=sprintf('%s%s',txt,sprintf(',%s',nl)); end
end
if(len>bracketlevel)
    txt(end+1:end+3)={nl,padding0,']'};
end
txt = sprintf('%s',txt{:});

%%-------------------------------------------------------------------------
function txt=struct2json(name,item,level,varargin)
txt={};
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
ws=varargin{1}.whitespaces_;
padding0=repmat(ws.tab,1,level);
padding2=repmat(ws.tab,1,level+1);
padding1=repmat(ws.tab,1,level+(dim(1)>1)+forcearray);
nl=ws.newline;

if(isempty(item)) 
    if(~isempty(name)) 
        txt={padding0, '"', decodevarname(name,varargin{1}.unpackhex),'": []'};
    else
        txt={padding0, '[]'};
    end
    txt = sprintf('%s',txt{:});
    return;
end
if(~isempty(name))
    if(forcearray)
        txt={padding0, '"', decodevarname(name,varargin{1}.unpackhex),'": [', nl};
    end
else
    if(forcearray)
        txt={padding0, '[', nl};
    end
end
for j=1:dim(2)
  if(dim(1)>1)
      txt(end+1:end+3)={padding2,'[',nl};
  end
  for i=1:dim(1)
    names = fieldnames(item(i,j));
    if(~isempty(name) && len==1 && ~forcearray)
        txt(end+1:end+5)={padding1, '"', decodevarname(name,varargin{1}.unpackhex),'": {', nl};
    else
        txt(end+1:end+3)={padding1, '{', nl};
    end
    if(~isempty(names))
      for e=1:length(names)
	    txt{end+1}=obj2json(names{e},item(i,j).(names{e}),...
             level+(dim(1)>1)+1+forcearray,varargin{:});
        if(e<length(names))
            txt{end+1}=',';
        end
        txt{end+1}=nl;
      end
    end
    txt(end+1:end+2)={padding1,'}'};
    if(i<dim(1))
        txt(end+1:end+2)={',' nl};
    end
  end
  if(dim(1)>1)
      txt(end+1:end+3)={nl,padding2,']'};
  end
  if(j<dim(2))
      txt(end+1:end+2)={',' nl};
  end
end
if(forcearray)
    txt(end+1:end+3)={nl,padding0,']'};
end
txt = sprintf('%s',txt{:});

%%-------------------------------------------------------------------------
function txt=map2json(name,item,level,varargin)
txt={};
if(~isa(item,'containers.Map'))
	error('input is not a containers.Map class');
end
dim=size(item);
names = keys(item);
val= values(item);

if(~strcmp(item.KeyType,'char'))
    mm=cell(1,length(names));
    for i=1:length(names)
        mm{i}={names{i}, val{i}};
    end
    if(isempty(name))
        txt=obj2json('_MapData_',mm,level+1,varargin{:});
    else
        temp=struct(name,struct());
	if(varargin{1}.isoctave)
            temp.(name).('_MapData_')=mm;
	else
	    temp.(name).('x0x5F_MapData_')=mm;
	end
        txt=obj2json(name,temp.(name),level,varargin{:});
    end
    return;
end

len=prod(dim);
forcearray= (len>1 || (varargin{1}.singletarray==1 && level>0));
ws=varargin{1}.whitespaces_;
padding0=repmat(ws.tab,1,level);
nl=ws.newline;

if(isempty(item)) 
    if(~isempty(name)) 
        txt={padding0, '"', decodevarname(name,varargin{1}.unpackhex),'": []'};
    else
        txt={padding0, '[]'};
    end
    txt = sprintf('%s',txt{:});
    return;
end
if(~isempty(name)) 
    if(forcearray)
        txt={padding0, '"', decodevarname(name,varargin{1}.unpackhex),'": {', nl};
    end
else
    if(forcearray)
        txt={padding0, '{', nl};
    end
end

for i=1:dim(1)
    if(~isempty(names{i}))
	    txt{end+1}=obj2json(names{i},val{i},...
             level+(dim(1)>1),varargin{:});
        if(i<length(names))
            txt{end+1}=',';
        end
        if(i<dim(1))
            txt{end+1}=nl;
        end
    end
end
if(forcearray)
    txt(end+1:end+3)={nl,padding0,'}'};
end
txt = sprintf('%s',txt{:});

%%-------------------------------------------------------------------------
function txt=str2json(name,item,level,varargin)
txt={};
if(~ischar(item))
        error('input is not a string');
end
item=reshape(item, max(size(item),[1 0]));
len=size(item,1);
ws=varargin{1}.whitespaces_;
padding1=repmat(ws.tab,1,level);
padding0=repmat(ws.tab,1,level+1);
nl=ws.newline;
sep=ws.sep;

if(~isempty(name)) 
    if(len>1)
        txt={padding1, '"', decodevarname(name,varargin{1}.unpackhex),'": [', nl};
    end
else
    if(len>1)
        txt={padding1, '[', nl};
    end
end
for e=1:len
    if(strcmp('_ArrayZipData_',decodevarname(name,varargin{1}.unpackhex))==0)
        val=escapejsonstring(item(e,:),varargin{:});
    else
        val=item(e,:);
    end
    if(len==1)
        obj=['"' decodevarname(name,varargin{1}.unpackhex) '": ' '"',val,'"'];
        if(isempty(name))
            obj=['"',val,'"'];
        end
        txt(end+1:end+2)={padding1, obj};
    else
        txt(end+1:end+4)={padding0,'"',val,'"'};
    end
    if(e==len)
        sep='';
    end
    txt{end+1}=sep;
end
if(len>1)
    txt(end+1:end+3)={nl,padding1,']'};
end
txt = sprintf('%s',txt{:});

%%-------------------------------------------------------------------------
function txt=mat2json(name,item,level,varargin)
if(~isnumeric(item) && ~islogical(item))
        error('input is not an array');
end
ws=varargin{1}.whitespaces_;
padding1=repmat(ws.tab,1,level);
padding0=repmat(ws.tab,1,level+1);
nl=ws.newline;
sep=ws.sep;

dozip=varargin{1}.compression;
zipsize=varargin{1}.compressarraysize;
format=varargin{1}.formatversion;
isnest=varargin{1}.nestarray;

if(((isnest==0) && length(size(item))>2) || issparse(item) || ~isreal(item) || ...
   (isempty(item) && any(size(item))) || varargin{1}.arraytostruct || ...
   (~isempty(dozip) && numel(item)>zipsize && strcmp('_ArrayZipData_',decodevarname(name,varargin{1}.unpackhex))==0))
    if(isempty(name))
    	txt=sprintf('%s{%s%s"_ArrayType_": "%s",%s%s"_ArraySize_": %s,%s',...
              padding1,nl,padding0,class(item),nl,padding0,regexprep(mat2str(size(item)),'\s+',','),nl);
    else
    	txt=sprintf('%s"%s": {%s%s"_ArrayType_": "%s",%s%s"_ArraySize_": %s,%s',...
              padding1,decodevarname(name,varargin{1}.unpackhex),nl,padding0,class(item),nl,padding0,regexprep(mat2str(size(item)),'\s+',','),nl);
    end
else
    numtxt=matdata2json(item,level+1,varargin{:});   
    if(isempty(name))
    	txt=sprintf('%s%s',padding1,numtxt);
    else
        if(numel(item)==1 && varargin{1}.singletarray==0)
           	txt=sprintf('%s"%s": %s',padding1,decodevarname(name,varargin{1}.unpackhex),numtxt);
        else
    	    txt=sprintf('%s"%s": %s',padding1,decodevarname(name,varargin{1}.unpackhex),numtxt);
        end
    end
    return;
end

dataformat='%s%s%s%s%s';

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
       txt=sprintf(dataformat,txt,padding0,'"_ArrayIsComplex_": ','true', sep);
    end
    txt=sprintf(dataformat,txt,padding0,'"_ArrayIsSparse_": ','true', sep);
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
        txt=sprintf(dataformat,txt,padding0,'"_ArrayZipSize_": ',regexprep(mat2str(size(fulldata)),'\s+',','), sep);
        txt=sprintf(dataformat,txt,padding0,'"_ArrayZipType_": "',dozip, ['"' sep]);
	    compfun=str2func([dozip 'encode']);
        txt=sprintf(dataformat,txt,padding0,'"_ArrayZipData_": "',base64encode(compfun(typecast(fulldata(:),'uint8'))),['"' nl]);
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
        txt=sprintf(dataformat,txt,padding0,'"_ArrayData_": ',...
               matdata2json(fulldata',level+2,varargin{:}), nl);    
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
            txt=sprintf(dataformat,txt,padding0,'"_ArrayIsComplex_": ','true', sep);
            fulldata=[real(item(:)) imag(item(:))]';
        end
        txt=sprintf(dataformat,txt,padding0,'"_ArrayZipSize_": ',regexprep(mat2str(size(fulldata)),'\s+',','), sep);
        txt=sprintf(dataformat,txt,padding0,'"_ArrayZipType_": "',dozip, ['"' sep]);
	    compfun=str2func([dozip 'encode']);
        txt=sprintf(dataformat,txt,padding0,'"_ArrayZipData_": "',base64encode(compfun(typecast(fulldata(:),'uint8'))),['"' nl]);
    else
        if(isreal(item))
            txt=sprintf(dataformat,txt,padding0,'"_ArrayData_": ',...
            matdata2json(item(:)',level+2,varargin{:}), nl);
        else
            txt=sprintf(dataformat,txt,padding0,'"_ArrayIsComplex_": ','true', sep);
            txt=sprintf(dataformat,txt,padding0,'"_ArrayData_": ',...
            matdata2json([real(item(:)) imag(item(:))]',level+2,varargin{:}), nl);
        end
    end
end

txt=sprintf('%s%s%s',txt,padding1,'}');

%%-------------------------------------------------------------------------
function txt=matlabobject2json(name,item,level,varargin)
try
    if numel(item) == 0 %empty object
        st = struct();
    elseif numel(item) == 1 %
        txt = str2json(name, char(item), level, varargin(:));
        return
    else
            propertynames = properties(item);
            for p = 1:numel(propertynames)
                for o = numel(item):-1:1 % aray of objects
                    st(o).(propertynames{p}) = item(o).(propertynames{p});
                end
            end
    end
    txt=struct2json(name,st,level,varargin{:});
catch
    txt = any2json(name,item, level, varargin(:));
end

%%-------------------------------------------------------------------------
function txt=matlabtable2json(name,item,level,varargin)
st=containers.Map();
st('_TableRecords_')=table2cell(item);
st('_TableRows_')=item.Properties.RowNames';
st('_TableCols_')=item.Properties.VariableNames;
if(isempty(name))
    txt=map2json(name,st,level,varargin{:});
else
    temp=struct(name,struct());
    temp.(name)=st;
    txt=map2json(name,temp.(name),level,varargin{:});
end

%%-------------------------------------------------------------------------
function txt=matdata2json(mat,level,varargin)

ws=varargin{1}.whitespaces_;
tab=ws.tab;
nl=ws.newline;
isnest=varargin{1}.nestarray;
format=varargin{1}.formatversion;
isnum2cell=varargin{1}.num2cell_;

if(~isvector(mat) && isnest==1)
   if(format>1.9 && isnum2cell==0)
        mat=permute(mat,ndims(mat):-1:1);
   end
   varargin{1}.num2cell_=1;
   txt=cell2json('',num2cell(mat,1),level-1,varargin{:});
   return;
else
    if(isnest)
        if(isnum2cell)
             mat=mat(:).';
        end
    end
end

if(size(mat,1)==1)
    pre='';
    post='';
    level=level-1;
else
    pre=sprintf('[%s',nl);
    post=sprintf('%s%s]',nl,repmat(tab,1,level-1));
end

if(isempty(mat))
    txt='[]';
    return;
end
if(isinteger(mat))
  floatformat=varargin{1}.intformat;
else
  floatformat=varargin{1}.floatformat;
end
if(numel(mat)==1 && varargin{1}.singletarray==0 && level>0)
    formatstr=[repmat([floatformat ','],1,size(mat,2)-1) [floatformat sprintf(',%s',nl)]];
else
    formatstr=['[' repmat([floatformat ','],1,size(mat,2)-1) [floatformat sprintf('],%s',nl)]];
end
if(nargin>=2 && size(mat,1)>1 && varargin{1}.arrayindent==1)
    formatstr=[repmat(tab,1,level) formatstr];
end

txt=sprintf(formatstr,mat');
txt(end-length(nl):end)=[];
if(islogical(mat) && (numel(mat)==1 || varargin{1}.parselogical==1))
   txt=regexprep(txt,'1','true');
   txt=regexprep(txt,'0','false');
end

txt=[pre txt post];
if(any(isinf(mat(:))))
    txt=regexprep(txt,'([-+]*)Inf',jsonopt('Inf','"$1_Inf_"',varargin{:}));
end
if(any(isnan(mat(:))))
    txt=regexprep(txt,'NaN',jsonopt('NaN','"_NaN_"',varargin{:}));
end

%%-------------------------------------------------------------------------
function txt=any2json(name,item,level,varargin)
st=containers.Map();
st('_DataInfo_')=struct('MATLABObjectName',name, 'MATLABObjectClass',class(item),'MATLABObjectSize',size(item));
st('_ByteStream_')=char(base64encode(getByteStreamFromArray(item)));

if(isempty(name))
    txt=map2json(name,st,level,varargin{:});
else
    temp=struct(name,struct());
    temp.(name)=st;
    txt=map2json(name,temp.(name),level,varargin{:});
end

%%-------------------------------------------------------------------------
function newstr=escapejsonstring(str,varargin)
newstr=str;
if(isempty(str) || isempty(regexp(str,'\W', 'once')))
    return;
end
isoct=varargin{1}.isoctave;
if(isoct)
   vv=sscanf(OCTAVE_VERSION,'%f');
   if(vv(1)>=3.8)
       isoct=0;
   end
end
if(isoct)
  escapechars={'\\','\"','\/','\a','\f','\n','\r','\t','\v'};
  for i=1:length(escapechars)
    newstr=regexprep(newstr,escapechars{i},escapechars{i});
  end
  newstr=regexprep(newstr,'\\\\(u[0-9a-fA-F]{4}[^0-9a-fA-F]*)','\$1');
else
  escapechars={'\\','\"','\/','\a','\b','\f','\n','\r','\t','\v'};
  esc={'\\\\','\\"','\\/','\\a','\\b','\\f','\\n','\\r','\\t','\\v'};
  for i=1:length(escapechars)
    newstr=regexprep(newstr,escapechars{i},esc{i});
  end
  newstr=regexprep(newstr,'\\\\(u[0-9a-fA-F]{4}[^0-9a-fA-F]*)','\\$1');
end
