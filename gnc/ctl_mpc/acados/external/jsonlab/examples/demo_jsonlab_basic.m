%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         Demonstration of Basic Utilities of JSONlab
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

rngstate = rand ('state');
randseed=hex2dec('623F9A9E');
clear data2json json2data

if(exist('isequaln')==0)
    isequaln=@isequal;
end

fprintf(1,'\n%%=================================================\n')
fprintf(1,'%%  a simple scalar value \n')
fprintf(1,'%%=================================================\n\n')

data2json=pi
savejson('',data2json)
json2data=loadjson(ans)

fprintf(1,'\n%%=================================================\n')
fprintf(1,'%%  an empty array \n')
fprintf(1,'%%=================================================\n\n')

data2json=[]
savejson('',data2json)
json2data=loadjson(ans)

fprintf(1,'\n%%=================================================\n')
fprintf(1,'%%  an ampty string \n')
fprintf(1,'%%=================================================\n\n')

data2json=''
savejson('emptystr',data2json)
json2data=loadjson(ans)

fprintf(1,'\n%%=================================================\n')
fprintf(1,'%%  a simple row vector \n')
fprintf(1,'%%=================================================\n\n')

data2json=1:3
savejson('',data2json)
json2data=loadjson(ans)
if(~isequaln(json2data,data2json))
    warning('conversion does not preserve original data');
end

fprintf(1,'\n%%=================================================\n')
fprintf(1,'%%  a simple column vector \n')
fprintf(1,'%%=================================================\n\n')

data2json=(1:3)'
savejson('',data2json)
json2data=loadjson(ans)
if(~isequaln(json2data,data2json))
    warning('conversion does not preserve original data');
end

fprintf(1,'\n%%=================================================\n')
fprintf(1,'%%  a string array \n')
fprintf(1,'%%=================================================\n\n')

data2json=['AC';'EG']
savejson('',data2json)
json2data=loadjson(ans)

fprintf(1,'\n%%=================================================\n')
fprintf(1,'%%  a string with escape symbols \n')
fprintf(1,'%%=================================================\n\n')

data2json=sprintf('AB\tCD\none"two')
savejson('str',data2json)
json2data=loadjson(ans)

fprintf(1,'\n%%=================================================\n')
fprintf(1,'%%  a mix-typed cell \n')
fprintf(1,'%%=================================================\n\n')

data2json={'a',true,[2;3]}
savejson('',data2json)
json2data=loadjson(ans)
if(~isequaln(json2data,data2json))
    warning('conversion does not preserve original data');
end

fprintf(1,'\n%%=================================================\n')
fprintf(1,'%%  a 3-D array in nested array form\n')
fprintf(1,'%%=================================================\n\n')

data2json=reshape(1:(2*4*6),[2,4,6]);
savejson('',data2json,'NestArray',1)
json2data=loadjson(ans)
if(~isequaln(json2data,data2json))
    warning('conversion does not preserve original data');
end

fprintf(1,'\n%%=================================================\n')
fprintf(1,'%%  a 3-D array in annotated array form\n')
fprintf(1,'%%=================================================\n\n')

data2json=reshape(1:(2*4*6),[2,4,6]);
savejson('',data2json,'NestArray',0)
json2data=loadjson(ans)
if(~isequaln(json2data,data2json))
    warning('conversion does not preserve original data');
end

fprintf(1,'\n%%=================================================\n')
fprintf(1,'%%  a 4-D array in annotated array form\n')
fprintf(1,'%%=================================================\n\n')

data2json=reshape(1:(2*4*3*2),[2,4,3,2]);
savejson('',data2json,'NestArray',0)  % nestarray for 4-D or above is not working
json2data=loadjson(ans)
if(~isequaln(json2data,data2json))
    warning('conversion does not preserve original data');
end

fprintf(1,'\n%%=================================================\n')
fprintf(1,'%%  a 3-D array in nested array form (JSONLab 1.9)\n')
fprintf(1,'%%=================================================\n\n')

data2json=reshape(1:(2*4*6),[2,4,6]);
savejson('',data2json,'NestArray',1,'FormatVersion',1.8)
json2data=loadjson(ans,'FormatVersion',1.8)
if(~isequaln(json2data,data2json))
    warning('conversion does not preserve original data');
end

fprintf(1,'\n%%=================================================\n')
fprintf(1,'%%  a 3-D array in annotated array form (JSONLab 1.9 or earlier)\n')
fprintf(1,'%%=================================================\n\n')

data2json=reshape(1:(2*4*6),[2,4,6]);
savejson('',data2json,'NestArray',0,'FormatVersion',1.8)
json2data=loadjson(ans,'FormatVersion',1.8)
if(~isequaln(json2data,data2json))
    warning('conversion does not preserve original data');
end

fprintf(1,'\n%%=================================================\n')
fprintf(1,'%%  a complex number\n')
fprintf(1,'%%=================================================\n\n')

data2json=1+2i
savejson('',data2json)
json2data=loadjson(ans) 
if(~isequaln(json2data,data2json))
    warning('conversion does not preserve original data');
end

fprintf(1,'\n%%=================================================\n')
fprintf(1,'%%  a complex matrix\n')
fprintf(1,'%%=================================================\n\n')

data2json=magic(6);
data2json=data2json(:,1:3)+data2json(:,4:6)*1i
savejson('',data2json)
json2data=loadjson(ans)
if(~isequaln(json2data,data2json))
    warning('conversion does not preserve original data');
end

fprintf(1,'\n%%=================================================\n')
fprintf(1,'%%  MATLAB special constants\n')
fprintf(1,'%%=================================================\n\n')

data2json=[NaN Inf -Inf]
savejson('specials',data2json)
json2data=loadjson(ans)
if(~isequaln(json2data.specials,data2json))
    warning('conversion does not preserve original data');
end

fprintf(1,'\n%%=================================================\n')
fprintf(1,'%%  a real sparse matrix\n')
fprintf(1,'%%=================================================\n\n')

data2json=sprand(10,10,0.1)
savejson('sparse',data2json,'FloatFormat','%.18g')
json2data=loadjson(ans)
if(~isequaln(json2data.sparse,data2json))
    warning('conversion does not preserve original data');
end

fprintf(1,'\n%%=================================================\n')
fprintf(1,'%%  a complex sparse matrix\n')
fprintf(1,'%%=================================================\n\n')

data2json=sprand(10,10,0.1);
data2json=data2json-data2json*1i
savejson('complex_sparse',data2json,'FloatFormat','%.18g')
json2data=loadjson(ans)
if(~isequaln(json2data.complex_sparse,data2json))
    warning('conversion does not preserve original data');
end

fprintf(1,'\n%%=================================================\n')
fprintf(1,'%%  an all-zero sparse matrix\n')
fprintf(1,'%%=================================================\n\n')

data2json=sparse(2,3);
savejson('all_zero_sparse',data2json)
json2data=loadjson(ans)
if(~isequaln(json2data.all_zero_sparse,data2json))
    warning('conversion does not preserve original data');
end

fprintf(1,'\n%%=================================================\n')
fprintf(1,'%%  an empty sparse matrix\n')
fprintf(1,'%%=================================================\n\n')

data2json=sparse([]);
savejson('empty_sparse',data2json)
json2data=loadjson(ans)
if(~isequaln(json2data.empty_sparse,data2json))
    warning('conversion does not preserve original data');
end

fprintf(1,'\n%%=================================================\n')
fprintf(1,'%%  an empty 0-by-0 real matrix\n')
fprintf(1,'%%=================================================\n\n')

data2json=[];
savejson('empty_0by0_real',data2json)
json2data=loadjson(ans)

fprintf(1,'\n%%=================================================\n')
fprintf(1,'%%  an empty 0-by-3 real matrix\n')
fprintf(1,'%%=================================================\n\n')

data2json=zeros(0,3);
savejson('empty_0by3_real',data2json)
json2data=loadjson(ans)
if(~isequaln(json2data.empty_0by3_real,data2json))
    warning('conversion does not preserve original data');
end

fprintf(1,'\n%%=================================================\n')
fprintf(1,'%%  a sparse real column vector\n')
fprintf(1,'%%=================================================\n\n')

data2json=sparse([0,3,0,1,4]');
savejson('sparse_column_vector',data2json)
json2data=loadjson(ans)

fprintf(1,'\n%%=================================================\n')
fprintf(1,'%%  a sparse complex column vector\n')
fprintf(1,'%%=================================================\n\n')

data2json=data2json-1i*data2json;
savejson('complex_sparse_column_vector',data2json)
json2data=loadjson(ans)

fprintf(1,'\n%%=================================================\n')
fprintf(1,'%%  a sparse real row vector\n')
fprintf(1,'%%=================================================\n\n')

data2json=sparse([0,3,0,1,4]);
savejson('sparse_row_vector',data2json)
json2data=loadjson(ans)

fprintf(1,'\n%%=================================================\n')
fprintf(1,'%%  a sparse complex row vector\n')
fprintf(1,'%%=================================================\n\n')

data2json=data2json-1i*data2json;
savejson('complex_sparse_row_vector',data2json)
json2data=loadjson(ans)

fprintf(1,'\n%%=================================================\n')
fprintf(1,'%%  a structure\n')
fprintf(1,'%%=================================================\n\n')

data2json=struct('name','Think Different','year',1997,'magic',magic(3),...
                 'misfits',[Inf,NaN],'embedded',struct('left',true,'right',false))
savejson('astruct',data2json,struct('ParseLogical',1))
json2data=loadjson(ans)
class(json2data.astruct.embedded.left)

fprintf(1,'\n%%=================================================\n')
fprintf(1,'%%  a structure array\n')
fprintf(1,'%%=================================================\n\n')

data2json=struct('name','Nexus Prime','rank',9);
data2json(2)=struct('name','Sentinel Prime','rank',9);
data2json(3)=struct('name','Optimus Prime','rank',9);
savejson('Supreme Commander',data2json)
json2data=loadjson(ans)

fprintf(1,'\n%%=================================================\n')
fprintf(1,'%%  a cell array\n')
fprintf(1,'%%=================================================\n\n')

data2json=cell(3,1);
data2json{1}=struct('buzz',1.1,'rex',1.2,'bo',1.3,'hamm',2.0,'slink',2.1,'potato',2.2,...
              'woody',3.0,'sarge',3.1,'etch',4.0,'lenny',5.0,'squeeze',6.0,'wheezy',7.0);
data2json{2}=struct('Ubuntu',['Kubuntu';'Xubuntu';'Lubuntu']);
data2json{3}=[10.04,10.10,11.04,11.10]
savejson('debian',data2json,struct('FloatFormat','%.2f'))
json2data=loadjson(ans)

fprintf(1,'\n%%=================================================\n')
fprintf(1,'%%  invalid field-name handling\n')
fprintf(1,'%%=================================================\n\n')

json2data=loadjson('{"ValidName":1, "_InvalidName":2, ":Field:":3, "项目":"绝密"}')

fprintf(1,'\n%%=================================================\n')
fprintf(1,'%%  a function handle\n')
fprintf(1,'%%=================================================\n\n')

data2json=@(x) x+1
savejson('handle',data2json)
json2data=loadjson(ans)

fprintf(1,'\n%%=================================================\n')
fprintf(1,'%%  a 2D cell array\n')
fprintf(1,'%%=================================================\n\n')

data2json={{1,{2,3}},{4,5},{6};{7},{8,9},{10}};
savejson('data2json',data2json)
json2data=loadjson(ans)  % only savejson works for cell arrays, loadjson has issues

fprintf(1,'\n%%=================================================\n')
fprintf(1,'%%  a 2D struct array\n')
fprintf(1,'%%=================================================\n\n')

data2json=repmat(struct('idx',0,'data','structs'),[2,3])
for i=1:6
    data2json(i).idx=i;
end
savejson('data2json',data2json)
json2data=loadjson(ans)


if(exist('datetime'))
    fprintf(1,'\n%%=================================================\n')
    fprintf(1,'%%  datetime object \n')
    fprintf(1,'%%=================================================\n\n')

    data2json=datetime({'8 April 2015','9 May 2015'}, 'InputFormat','d MMMM yyyy')
    savejson('',data2json)
    json2data=loadjson(ans)
end

if(exist('containers.Map'))
    fprintf(1,'\n%%=================================================\n')
    fprintf(1,'%%  a container.Maps object \n')
    fprintf(1,'%%=================================================\n\n')

    data2json=containers.Map({'Andy','William','Om'},[21,21,22])
    savejson('',data2json)
    json2data=loadjson(ans)
end

if(exist('istable'))
    fprintf(1,'\n%%=================================================\n')
    fprintf(1,'%%  a table object \n')
    fprintf(1,'%%=================================================\n\n')

    Names={'Andy','William','Om'}';
    Age=[21,21,22]';
    data2json=table(Names,Age)
    savejson('table',table(Names,Age))
    json2data=loadjson(ans)
end

try
    val=zlibencode('test');
    fprintf(1,'\n%%=================================================\n')
    fprintf(1,'%%  a 2-D array in compressed array format\n')
    fprintf(1,'%%=================================================\n\n')

    data2json=eye(10);
    data2json(20,1)=1;
    savejson('',data2json,'Compression','zlib','CompressionSize',0)  % nestarray for 4-D or above is not working
    json2data=loadjson(ans)
    if(~isequaln(json2data,data2json))
        warning('conversion does not preserve original data');
    end
catch
end

rand ('state',rngstate);

