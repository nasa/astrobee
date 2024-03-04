function jdata=jdataencode(data, varargin)
%
% jdata=jdataencode(data)
%    or
% jdata=jdataencode(data, options)
% jdata=jdataencode(data, 'Param1',value1, 'Param2',value2,...)
%
% Serialize a MATLAB struct or cell array into a JData-compliant 
% structure as defined in the JData spec: http://github.com/fangq/jdata
%
% This function implements the JData Specification Draft 2 (Oct. 2019)
% see http://github.com/fangq/jdata for details
%
% author: Qianqian Fang (q.fang <at> neu.edu)
%
% input:
%     data: a structure (array) or cell (array) to be encoded.
%     options: (optional) a struct or Param/value pairs for user
%              specified options (first in [.|.] is the default)
%         Base64: [0|1] if set to 1, _ArrayZipData_ is assumed to
%       	       be encoded with base64 format and need to be
%       	       decoded first. This is needed for JSON but not
%       	       UBJSON data
%         Prefix: ['x0x5F'|'x'] for JData files loaded via loadjson/loadubjson, the
%                      default JData keyword prefix is 'x0x5F'; if the
%                      json file is loaded using matlab2018's
%                      jsondecode(), the prefix is 'x'; this function
%                      attempts to automatically determine the prefix;
%                      for octave, the default value is an empty string ''.
%         UseArrayZipSize: [1|0] if set to 1, _ArrayZipSize_ will be added to 
%       	       store the "pre-processed" data dimensions, i.e.
%       	       the original data stored in _ArrayData_, and then flaten
%       	       _ArrayData_ into a row vector using row-major
%       	       order; if set to 0, a 2D _ArrayData_ will be used
%         MapAsStruct: [0|1] if set to 1, convert containers.Map into
%       	       struct; otherwise, keep it as map
%         Compression: ['zlib'|'gzip','lzma','lz4','lz4hc'] - use zlib method 
%       	       to compress data array
%         CompressArraySize: [100|int]: only to compress an array if the  
%       	       total element count is larger than this number.
%         FormatVersion [2|float]: set the JSONLab output version; since
%       	       v2.0, JSONLab uses JData specification Draft 1
%       	       for output format, it is incompatible with all
%       	       previous releases; if old output is desired,
%       	       please set FormatVersion to 1.9 or earlier.
%
% example:
%     jd=jdataencode(struct('a',rand(5)+1i*rand(5),'b',[],'c',sparse(5,5)))
%
% license:
%     BSD or GPL version 3, see LICENSE_{BSD,GPLv3}.txt files for details 
%
% -- this function is part of JSONLab toolbox (http://iso2mesh.sf.net/cgi-bin/index.cgi?jsonlab)
%


if(nargin==0)
    help jdataencode
    return;
end

opt=varargin2struct(varargin{:});
if(isoctavemesh)
    opt.prefix=jsonopt('Prefix','',opt);
else
    opt.prefix=jsonopt('Prefix',sprintf('x0x%X','_'+0),opt);
end
opt.compression=jsonopt('Compression','',opt);
opt.nestarray=jsonopt('NestArray',0,opt);
opt.formatversion=jsonopt('FormatVersion',2,opt);
opt.compressarraysize=jsonopt('CompressArraySize',100,opt);
opt.base64=jsonopt('Base64',0,opt);
opt.mapasstruct=jsonopt('MapAsStruct',0,opt);
opt.usearrayzipsize=jsonopt('UseArrayZipSize',1,opt);
opt.messagepack=jsonopt('MessagePack',0,opt);

jdata=obj2jd(data,opt);

%%-------------------------------------------------------------------------
function newitem=obj2jd(item,varargin)

if(iscell(item))
    newitem=cell2jd(item,varargin{:});
elseif(isstruct(item))
    newitem=struct2jd(item,varargin{:});
elseif(isnumeric(item) || islogical(item))
    newitem=mat2jd(item,varargin{:});
elseif(ischar(item) || isa(item,'string'))
    newitem=mat2jd(item,varargin{:});
elseif(isa(item,'containers.Map'))
    newitem=map2jd(item,varargin{:});
elseif(isa(item,'categorical'))
    newitem=cell2jd(cellstr(item),varargin{:});
elseif(isa(item,'function_handle'))
    newitem=struct2jd(functions(item),varargin{:});
elseif(isa(item,'table'))
    newitem=table2jd(item,varargin{:});
elseif(isa(item,'digraph') || isa(item,'graph'))
    newitem=graph2jd(item,varargin{:});
elseif(isobject(item))
    newitem=matlabobject2jd(item,varargin{:});
else
    newitem=item;
end

%%-------------------------------------------------------------------------
function newitem=cell2jd(item,varargin)

newitem=cellfun(@(x) obj2jd(x, varargin{:}), item, 'UniformOutput',false);

%%-------------------------------------------------------------------------
function newitem=struct2jd(item,varargin)

num=numel(item);
if(num>1)  % struct array
    newitem=obj2jd(num2cell(item),varargin{:});
    try
       newitem=cell2mat(newitem);
    catch
    end
else       % a single struct
    names=fieldnames(item);
    newitem=struct;
    for i=1:length(names)
        newitem.(names{i})=obj2jd(item.(names{i}),varargin{:});
    end
end

%%-------------------------------------------------------------------------
function newitem=map2jd(item,varargin)

names=item.keys;
if(varargin{1}.mapasstruct)  % convert a map to struct
    newitem=struct;
    if(~strcmp(item.KeyType,'char'))
        data=num2cell(reshape([names, item.values],length(names),2),2);
        for i=1:length(names)
            data{i}{2}=obj2jd(data{i}{2},varargin{:});
        end
        newitem.(N_('_MapData_',varargin{:}))=data;
    else
        for i=1:length(names)
            newitem.(N_(names{i},varargin{:}))=obj2jd(item(names{i}),varargin{:});
        end
    end
else   % keep as a map and only encode its values
    newitem=containers.Map('KeyType',item.KeyType,'ValueType','any');
    for i=1:length(names)
        newitem(names{i})=obj2jd(item(names{i}),varargin{:});
    end
end
%%-------------------------------------------------------------------------
function newitem=mat2jd(item,varargin)

if(isempty(item) || isa(item,'string') || ischar(item) || varargin{1}.nestarray || ...
        ((isvector(item) || ismatrix(item)) && isreal(item) && ~issparse(item)))
    newitem=item;
    if(~(varargin{1}.messagepack && size(item,1)>1))
        return;
    end
end

zipmethod=varargin{1}.compression;
minsize=varargin{1}.compressarraysize;

if(isa(item,'logical'))
    item=uint8(item);
end

N=@(x) N_(x,varargin{:});

newitem=struct(N('_ArrayType_'),class(item),N('_ArraySize_'),size(item));

if(isreal(item))
    if(issparse(item))
	fulldata=full(item(find(item)));
        newitem.(N('_ArrayIsSparse_'))=true;
	newitem.(N('_ArrayZipSize_'))=[2+(~isvector(item)),length(fulldata)];
        if(isvector(item))
            newitem.(N('_ArrayData_'))=[find(item(:))', fulldata(:)'];
        else
            [ix,iy]=find(item);
	        newitem.(N('_ArrayData_'))=[ix(:)' , iy(:)', fulldata(:)'];
	end
    else
        if(varargin{1}.formatversion>1.9)
                item=permute(item,ndims(item):-1:1);
        end
	newitem.(N('_ArrayData_'))=item(:)';
    end
else
    newitem.(N('_ArrayIsComplex_'))=true;
    if(issparse(item))
	fulldata=full(item(find(item)));
        newitem.(N('_ArrayIsSparse_'))=true;
	newitem.(N('_ArrayZipSize_'))=[3+(~isvector(item)),length(fulldata)];
        if(isvector(item))
            newitem.(N('_ArrayData_'))=[find(item(:))', real(fulldata(:))', imag(fulldata(:))'];
        else
            [ix,iy]=find(item);
            newitem.(N('_ArrayData_'))=[ix(:)' , iy(:)' , real(fulldata(:))', imag(fulldata(:))'];
	end
    else
        if(varargin{1}.formatversion>1.9)
                item=permute(item,ndims(item):-1:1);
        end
        newitem.(N('_ArrayZipSize_'))=[2,numel(item)];
	newitem.(N('_ArrayData_'))=[real(item(:))', imag(item(:))'];
    end
end

if(varargin{1}.usearrayzipsize==0 && isfield(newitem,N('_ArrayZipSize_')))
    data=newitem.(N('_ArrayData_'));
    data=reshape(data,fliplr(newitem.(N('_ArrayZipSize_'))));
    newitem.(N('_ArrayData_'))=permute(data,ndims(data):-1:1);
    newitem=rmfield(newitem,N('_ArrayZipSize_'));
end

if(~isempty(zipmethod) && numel(item)>minsize)
    compfun=str2func([zipmethod 'encode']);
    newitem.(N('_ArrayZipType_'))=lower(zipmethod);
    newitem.(N('_ArrayZipSize_'))=size(newitem.(N('_ArrayData_')));
    newitem.(N('_ArrayZipData_'))=compfun(typecast(newitem.(N('_ArrayData_')),'uint8'));
    newitem=rmfield(newitem,N('_ArrayData_'));
    if(varargin{1}.base64)
        newitem.(N('_ArrayZipData_'))=char(base64encode(newitem.(N('_ArrayZipData_'))));
    end
end

if(isfield(newitem,N('_ArrayData_')) && isempty(newitem.(N('_ArrayData_'))))
    newitem.(N('_ArrayData_'))=[];
end

%%-------------------------------------------------------------------------
function newitem=table2jd(item,varargin)

newitem=struct;
newitem.(N_('_TableCols_',varargin{:}))=item.Properties.VariableNames;
newitem.(N_('_TableRows_',varargin{:}))=item.Properties.RowNames';
newitem.(N_('_TableRecords_',varargin{:}))=table2cell(item);

%%-------------------------------------------------------------------------
function newitem=graph2jd(item,varargin)

newitem=struct;
nodedata=table2struct(item.Nodes);
if(isfield(nodedata,'Name'))
    nodedata=rmfield(nodedata,'Name');
    newitem.(N_('_GraphNodes_',varargin{:}))=containers.Map(item.Nodes.Name,num2cell(nodedata),'UniformValues',false);
else
    newitem.(N_('_GraphNodes_',varargin{:}))=containers.Map(1:max(item.Edges.EndNodes(:)),num2cell(nodedata),'UniformValues',false);
end
edgenodes=num2cell(item.Edges.EndNodes);
edgedata=table2struct(item.Edges);
if(isfield(edgedata,'EndNodes'))
    edgedata=rmfield(edgedata,'EndNodes');
end
edgenodes(:,3)=num2cell(edgedata);
if(isa(item,'graph'))
    if(strcmp(varargin{1}.prefix,'x'))
        newitem.(genvarname('_GraphEdges0_'))=edgenodes;
    else
        newitem.(encodevarname('_GraphEdges0_'))=edgenodes;
    end
else
    newitem.(N_('_GraphEdges_',varargin{:}))=edgenodes;
end

%%-------------------------------------------------------------------------
function newitem=matlabobject2jd(item,varargin)
try
    if numel(item) == 0 %empty object
        newitem = struct();
    elseif numel(item) == 1 %
        newitem = char(item);
    else
        propertynames = properties(item);
        for p = 1:numel(propertynames)
            for o = numel(item):-1:1 % aray of objects
                newitem(o).(propertynames{p}) = item(o).(propertynames{p});
            end
        end
    end
catch
    newitem=any2jd(item,varargin{:});
end

%%-------------------------------------------------------------------------
function newitem=any2jd(item,varargin)

N=@(x) N_(x,varargin{:});
newitem.(N('_DataInfo_'))=struct('MATLABObjectClass',class(item),'MATLABObjectSize',size(item));
newitem.(N('_ByteStream_'))=getByteStreamFromArray(item);  % use undocumented matlab function
if(varargin{1}.base64)
    newitem.(N('_ByteStream_'))=char(base64encode(newitem.(N('_ByteStream_'))));
end

%%-------------------------------------------------------------------------
function newname=N_(name,varargin)

newname=[varargin{1}.prefix name];
