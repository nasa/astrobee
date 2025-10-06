% JSONLAB
%
% Files
%   base64decode       - output = base64decode(input)
%   base64encode       - output = base64encode(input)
%   decodevarname      - newname = decodevarname(name)
%   encodevarname      - newname = encodevarname(name)
%   fast_match_bracket - [endpos, maxlevel] = fast_match_bracket(key,pos,startpos,brackets)
%   gzipdecode         - output = gzipdecode(input)
%   gzipencode         - output = gzipencode(input)
%   isoctavemesh       - [isoctave verinfo]=isoctavemesh
%   jdatadecode        - newdata=jdatadecode(data,opt,...)
%   jdataencode        - jdata=jdataencode(data)
%   jsonopt            - val=jsonopt(key,default,optstruct)
%   loadjson           - data=loadjson(fname,opt)
%   loadmsgpack        - PARSEMSGPACK parses a msgpack byte buffer into Matlab data structures
%   loadubjson         - data=loadubjson(fname,opt)
%   lz4decode          - output = lz4decode(input)
%   lz4encode          - output = lz4encode(input)
%   lz4hcdecode        - output = lz4hcdecode(input)
%   lz4hcencode        - output = lz4hcencode(input)
%   lzipdecode         - output = lzipdecode(input)
%   lzipencode         - output = lzipencode(input)
%   lzmadecode         - output = lzmadecode(input)
%   lzmaencode         - output = lzmaencode(input)
%   match_bracket      - [endpos, maxlevel] = match_bracket(str,startpos,brackets)
%   mergestruct        - s=mergestruct(s1,s2)
%   nestbracket2dim    - [dims, maxlevel, count] = nestbracket2dim(str,brackets)
%   savejson           - json=savejson(rootname,obj,filename)
%   savemsgpack        - msgpk=savemsgpack(rootname,obj,filename)
%   saveubjson         - json=saveubjson(rootname,obj,filename)
%   varargin2struct    - opt=varargin2struct('param1',value1,'param2',value2,...)
%   zlibdecode         - output = zlibdecode(input)
%   zlibencode         - output = zlibencode(input)

%%=== # JData specification ===

%==== function jdata=jdataencode(data, varargin) ====
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

%==== function newdata=jdatadecode(data,varargin) ====
%
% newdata=jdatadecode(data,opt,...)
%
% Convert all JData object (in the form of a struct array) into an array
% (accepts JData objects loaded from either loadjson/loadubjson or 
% jsondecode for MATLAB R2018a or later)
%
% This function implements the JData Specification Draft 2 (Oct. 2019)
% see http://github.com/fangq/jdata for details
%
%
% input:
%      data: a struct array. If data contains JData keywords in the first
%            level children, these fields are parsed and regrouped into a
%            data object (arrays, trees, graphs etc) based on JData 
%            specification. The JData keywords are
%               "_ArrayType_", "_ArraySize_", "_ArrayData_"
%               "_ArrayIsSparse_", "_ArrayIsComplex_", 
%               "_ArrayZipType_", "_ArrayZipSize", "_ArrayZipData_"
%      opt: (optional) a list of 'Param',value pairs for additional options 
%           The supported options include
%               Recursive: [1|0] if set to 1, will apply the conversion to 
%                            every child; 0 to disable
%               Base64: [0|1] if set to 1, _ArrayZipData_ is assumed to
%                         be encoded with base64 format and need to be
%                         decoded first. This is needed for JSON but not
%                         UBJSON data
%               Prefix: ['x0x5F'|'x'] for JData files loaded via loadjson/loadubjson, the
%                         default JData keyword prefix is 'x0x5F'; if the
%                         json file is loaded using matlab2018's
%                         jsondecode(), the prefix is 'x'; this function
%                         attempts to automatically determine the prefix;
%                         for octave, the default value is an empty string ''.
%               FormatVersion: [2|float]: set the JSONLab output version; 
%                         since v2.0, JSONLab uses JData specification Draft 1
%                         for output format, it is incompatible with all
%                         previous releases; if old output is desired,
%                         please set FormatVersion to 1
%
% output:
%      newdata: the covnerted data if the input data does contain a JData 
%               structure; otherwise, the same as the input.
%
% examples:
%      obj={[],{'test'},true,struct('sparse',sparse(2,3),'magic',uint8(magic(5)))}
%      jdata=jdatadecode(jdataencode(obj))
%      isequaln(obj,jdata)
%
% license:
%     BSD or GPL version 3, see LICENSE_{BSD,GPLv3}.txt files for details 
%

%%=== # JSON ===

%==== function data = loadjson(fname,varargin) ====
%
% data=loadjson(fname,opt)
%    or
% data=loadjson(fname,'param1',value1,'param2',value2,...)
%
% parse a JSON (JavaScript Object Notation) file or string
%
% created on 2011/09/09, including previous works from 
%
%         Nedialko Krouchev: http://www.mathworks.com/matlabcentral/fileexchange/25713
%            created on 2009/11/02
%         François Glineur: http://www.mathworks.com/matlabcentral/fileexchange/23393
%            created on  2009/03/22
%         Joel Feenstra:
%         http://www.mathworks.com/matlabcentral/fileexchange/20565
%            created on 2008/07/03
%
% input:
%      fname: input file name; if fname contains "{}" or "[]", fname
%             will be interpreted as a JSON string
%      opt: (optional) a struct to store parsing options, opt can be replaced by 
%           a list of ('param',value) pairs - the param string is equivallent
%           to a field in opt. opt can have the following 
%           fields (first in [.|.] is the default)
%
%           SimplifyCell [0|1]: if set to 1, loadjson will call cell2mat
%                         for each element of the JSON data, and group 
%                         arrays based on the cell2mat rules.
%           FastArrayParser [1|0 or integer]: if set to 1, use a
%                         speed-optimized array parser when loading an 
%                         array object. The fast array parser may 
%                         collapse block arrays into a single large
%                         array similar to rules defined in cell2mat; 0 to 
%                         use a legacy parser; if set to a larger-than-1
%                         value, this option will specify the minimum
%                         dimension to enable the fast array parser. For
%                         example, if the input is a 3D array, setting
%                         FastArrayParser to 1 will return a 3D array;
%                         setting to 2 will return a cell array of 2D
%                         arrays; setting to 3 will return to a 2D cell
%                         array of 1D vectors; setting to 4 will return a
%                         3D cell array.
%           ShowProgress [0|1]: if set to 1, loadjson displays a progress bar.
%           ParseStringArray [0|1]: if set to 0, loadjson converts "string arrays" 
%                         (introduced in MATLAB R2016b) to char arrays; if set to 1,
%                         loadjson skips this conversion.
%           FormatVersion [2|float]: set the JSONLab format version; since
%                         v2.0, JSONLab uses JData specification Draft 1
%                         for output format, it is incompatible with all
%                         previous releases; if old output is desired,
%                         please set FormatVersion to 1.9 or earlier.
%           Encoding ['']: json file encoding. Support all encodings of
%                         fopen() function
%           JDataDecode [1|0]: if set to 1, call jdatadecode to decode
%                        JData structures defined in the JData
%                        Specification.
%
% output:
%      dat: a cell array, where {...} blocks are converted into cell arrays,
%           and [...] are converted to arrays
%
% examples:
%      dat=loadjson('{"obj":{"string":"value","array":[1,2,3]}}')
%      dat=loadjson(['examples' filesep 'example1.json'])
%      dat=loadjson(['examples' filesep 'example1.json'],'SimplifyCell',1)
%
% license:
%     BSD or GPL version 3, see LICENSE_{BSD,GPLv3}.txt files for details 
%

%==== function json=savejson(rootname,obj,varargin) ====
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

%%=== # UBJSON ===

%==== function data = loadubjson(fname,varargin) ====
%
% data=loadubjson(fname,opt)
%    or
% data=loadubjson(fname,'param1',value1,'param2',value2,...)
%
% parse a JSON (JavaScript Object Notation) file or string
%
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

%==== function json=saveubjson(rootname,obj,varargin) ====
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

%%=== # MessagePack ===

%==== function data = loadmsgpack(fname,varargin) ====
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
%
% license:
%     BSD 3-clause license or GPL version 3, see LICENSE_{BSD,GPLv3}.txt files for details 
%

%==== function msgpk=savemsgpack(rootname,obj,varargin) ====
%
% msgpk=savemsgpack(obj)
%    or
% msgpk=savemsgpack(rootname,obj,filename)
% msgpk=savemsgpack(rootname,obj,opt)
% msgpk=savemsgpack(rootname,obj,'param1',value1,'param2',value2,...)
%
% convert a MATLAB object (cell, struct, array, table, map, handles ...) 
% into a MessagePack binary stream
%
% initially created on 2019/05/20
%
% This function is the same as calling saveubjson(...,'MessagePack',1)
%
% Please type "help saveubjson" for details for the supported inputs and outputs.
%
% license:
%     BSD or GPL version 3, see LICENSE_{BSD,GPLv3}.txt files for details
%

%%=== # Compression and decompression ===

%==== function varargout = zlibencode(varargin) ====
%
% output = zlibencode(input)
%    or
% [output, info] = zlibencode(input)
%
% Compress a string or numerical array using the ZLIB-compression
%
% This function depends on JVM in MATLAB or, can optionally use the ZMat 
% toolbox (http://github.com/fangq/zmat)
%
% Copyright (c) 2012, Kota Yamaguchi
% URL: https://www.mathworks.com/matlabcentral/fileexchange/39526-byte-encoding-utilities
%
%
% input:
%      input: the original data, can be a string, a numerical vector or array
%
% output:
%      output: the decompressed byte stream stored in a uint8 vector; if info is 
%            given, output will restore the original data's type and dimensions
%
% examples:
%      [bytes, info]=zlibencode(eye(10));
%      orig=zlibdecode(bytes,info);
%
% license:
%     BSD or GPL version 3, see LICENSE_{BSD,GPLv3}.txt files for details 
%

%==== function varargout = zlibdecode(varargin) ====
%
% output = zlibdecode(input)
%    or
% output = zlibdecode(input,info)
%
% Decompressing a ZLIB-compressed byte-stream to recover the original data
% This function depends on JVM in MATLAB or, can optionally use the ZMat 
% toolbox (http://github.com/fangq/zmat)
%
% Copyright (c) 2012, Kota Yamaguchi
% URL: https://www.mathworks.com/matlabcentral/fileexchange/39526-byte-encoding-utilities
%
%
% input:
%      input: a string, int8/uint8 vector or numerical array to store ZLIB-compressed data
%      info (optional): a struct produced by the zmat/lz4hcencode function during 
%            compression; if not given, the inputs/outputs will be treated as a
%            1-D vector
%
% output:
%      output: the decompressed byte stream stored in a uint8 vector; if info is 
%            given, output will restore the original data's type and dimensions
%
% examples:
%      [bytes, info]=zlibencode(eye(10));
%      orig=zlibdecode(bytes,info);
%
% license:
%     BSD or GPL version 3, see LICENSE_{BSD,GPLv3}.txt files for details 
%

%==== function varargout = gzipencode(varargin) ====
%
% output = gzipencode(input)
%    or
% [output, info] = gzipencode(input)
%
% Compress a string or numerical array using the GZIP-compression
%
% This function depends on JVM in MATLAB or, can optionally use the ZMat 
% toolbox (http://github.com/fangq/zmat)
%
% Copyright (c) 2012, Kota Yamaguchi
% URL: https://www.mathworks.com/matlabcentral/fileexchange/39526-byte-encoding-utilities
%
%
% input:
%      input: the original data, can be a string, a numerical vector or array
%
% output:
%      output: the decompressed byte stream stored in a uint8 vector; if info is 
%            given, output will restore the original data's type and dimensions
%
% examples:
%      [bytes, info]=gzipencode(eye(10));
%      orig=gzipdecode(bytes,info);
%
% license:
%     BSD or GPL version 3, see LICENSE_{BSD,GPLv3}.txt files for details 
%

%==== function varargout = gzipdecode(varargin) ====
%
% output = gzipdecode(input)
%    or
% output = gzipdecode(input,info)
%
% Decompressing a GZIP-compressed byte-stream to recover the original data
% This function depends on JVM in MATLAB or, can optionally use the ZMat 
% toolbox (http://github.com/fangq/zmat)
%
% Copyright (c) 2012, Kota Yamaguchi
% URL: https://www.mathworks.com/matlabcentral/fileexchange/39526-byte-encoding-utilities
%
%
% input:
%      input: a string, int8/uint8 vector or numerical array to store the GZIP-compressed data
%      info (optional): a struct produced by the zmat/lz4hcencode function during 
%            compression; if not given, the inputs/outputs will be treated as a
%            1-D vector
%
% output:
%      output: the decompressed byte stream stored in a uint8 vector; if info is 
%            given, output will restore the original data's type and dimensions
%
% examples:
%      [bytes, info]=gzipencode(eye(10));
%      orig=gzipdecode(bytes,info);
%
% license:
%     BSD or GPL version 3, see LICENSE_{BSD,GPLv3}.txt files for details 
%

%==== function varargout = lzmaencode(varargin) ====
%
% output = lzmaencode(input)
%    or
% [output, info] = lzmaencode(input)
%
% Compress a string or a numerical array using LZMA-compression
%
% This function depends on the ZMat toolbox (http://github.com/fangq/zmat)
%
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

%==== function varargout = lzmadecode(varargin) ====
%
% output = lzmadecode(input)
%    or
% output = lzmadecode(input,info)
%
% Decompressing an LZMA-compressed byte-stream to recover the original data
% This function depends on the ZMat toolbox (http://github.com/fangq/zmat)
%
%
% input:
%      input: a string, int8/uint8 vector or numerical array to store LZMA-compressed data
%      info (optional): a struct produced by the zmat/lzmaencode function during 
%            compression; if not given, the inputs/outputs will be treated as a
%            1-D vector
%
% output:
%      output: the decompressed byte stream stored in a uint8 vector; if info is 
%            given, output will restore the original data's type and dimensions
%
% examples:
%      [bytes, info]=lzmaencode(eye(10));
%      orig=lzmadecode(bytes,info);
%
% license:
%     BSD or GPL version 3, see LICENSE_{BSD,GPLv3}.txt files for details 
%

%==== function varargout = lzipencode(varargin) ====
%
% output = lzipencode(input)
%    or
% [output, info] = lzipencode(input)
%
% Compress a string or a numerical array using LZip-compression
%
% This function depends on the ZMat toolbox (http://github.com/fangq/zmat)
%
%
% input:
%      input: the original data, can be a string, a numerical vector or array
%
% output:
%      output: the compressed byte stream stored in a uint8 vector
%      info: (optional) a struct storing the metadata of the input, see "help zmat" for details
%
% examples:
%      [bytes, info]=lzipencode(eye(10));
%      orig=lzipdecode(bytes,info);
%
% license:
%     BSD or GPL version 3, see LICENSE_{BSD,GPLv3}.txt files for details 
%

%==== function varargout = lzipdecode(varargin) ====
%
% output = lzipdecode(input)
%    or
% output = lzipdecode(input,info)
%
% Decompressing an Lzip-compressed byte-stream to recover the original data
% This function depends on the ZMat toolbox (http://github.com/fangq/zmat)
%
%
% input:
%      input: a string, int8/uint8 vector or numerical array to store Lzip-compressed data
%      info (optional): a struct produced by the zmat/lzipencode function during 
%            compression; if not given, the inputs/outputs will be treated as a
%            1-D vector
%
% output:
%      output: the decompressed byte stream stored in a uint8 vector; if info is 
%            given, output will restore the original data's type and dimensions
%
% examples:
%      [bytes, info]=lzipencode(eye(10));
%      orig=lzipdecode(bytes,info);
%
% license:
%     BSD or GPL version 3, see LICENSE_{BSD,GPLv3}.txt files for details 
%

%==== function varargout = lz4hcencode(varargin) ====
%
% output = lz4encode(input)
%    or
% [output, info] = lz4encode(input)
%
% Compress a string or a numerical array using LZ4-compression
%
% This function depends on the ZMat toolbox (http://github.com/fangq/zmat)
%
%
% input:
%      input: the original data, can be a string, a numerical vector or array
%
% output:
%      output: the compressed byte stream stored in a uint8 vector
%      info: (optional) a struct storing the metadata of the input, see "help zmat" for details
%
% examples:
%      [bytes, info]=lz4encode(eye(10));
%      orig=lz4decode(bytes,info);
%
% license:
%     BSD or GPL version 3, see LICENSE_{BSD,GPLv3}.txt files for details 
%

%==== function varargout = lz4decode(varargin) ====
%
% output = lz4decode(input)
%    or
% output = lz4decode(input,info)
%
% Decompressing an LZ4-compressed byte-stream to recover the original data
% This function depends on the ZMat toolbox (http://github.com/fangq/zmat)
%
%
% input:
%      input: a string, int8/uint8 vector or numerical array to store LZ4-compressed data
%      info (optional): a struct produced by the zmat/lz4encode function during 
%            compression; if not given, the inputs/outputs will be treated as a
%            1-D vector
%
% output:
%      output: the decompressed byte stream stored in a uint8 vector; if info is 
%            given, output will restore the original data's type and dimensions
%
% examples:
%      [bytes, info]=lz4encode(eye(10));
%      orig=lz4decode(bytes,info);
%
% license:
%     BSD or GPL version 3, see LICENSE_{BSD,GPLv3}.txt files for details 
%

%==== function varargout = lz4hcencode(varargin) ====
%
% output = lz4hcencode(input)
%    or
% [output, info] = lz4hcencode(input)
%
% Compress a string or a numerical array using LZ4HC-compression
%
% This function depends on the ZMat toolbox (http://github.com/fangq/zmat)
%
%
% input:
%      input: the original data, can be a string, a numerical vector or array
%
% output:
%      output: the compressed byte stream stored in a uint8 vector
%      info: (optional) a struct storing the metadata of the input, see "help zmat" for details
%
% examples:
%      [bytes, info]=lz4hcencode(eye(10));
%      orig=lz4hcdecode(bytes,info);
%
% license:
%     BSD or GPL version 3, see LICENSE_{BSD,GPLv3}.txt files for details 
%

%==== function varargout = lz4hcdecode(varargin) ====
%
% output = lz4hcdecode(input)
%    or
% output = lz4hcdecode(input,info)
%
% Decompressing an LZ4HC-compressed byte-stream to recover the original data
% This function depends on the ZMat toolbox (http://github.com/fangq/zmat)
%
%
% input:
%      input: a string, int8/uint8 vector or numerical array to store LZ4HC-compressed data
%      info (optional): a struct produced by the zmat/lz4hcencode function during 
%            compression; if not given, the inputs/outputs will be treated as a
%            1-D vector
%
% output:
%      output: the decompressed byte stream stored in a uint8 vector; if info is 
%            given, output will restore the original data's type and dimensions
%
% examples:
%      [bytes, info]=lz4hcencode(eye(10));
%      orig=lz4hcdecode(bytes,info);
%
% license:
%     BSD or GPL version 3, see LICENSE_{BSD,GPLv3}.txt files for details 
%

%==== function varargout = base64encode(varargin) ====
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

%==== function output = base64decode(varargin) ====
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

%==== function str = encodevarname(str,varargin) ====
%
%    newname = encodevarname(name)
%
%    Encode an invalid variable name using a hex-format for bi-directional
%    conversions. 
%    This function is sensitive to the default charset
%    settings in MATLAB, please call feature('DefaultCharacterSet','utf8')
%    to set the encoding to UTF-8 before calling this function.
%
%
%    input:
%        name: a string, can be either a valid or invalid variable name
%
%    output:
%        newname: a valid variable name by converting the leading non-ascii
%              letter into "x0xHH_" and non-ascii letters into "_0xHH_"
%              format, where HH is the ascii (or Unicode) value of the
%              character.
%
%              if the encoded variable name CAN NOT be longer than 63, i.e. 
%              the maximum variable name specified by namelengthmax, and
%              one uses the output of this function as a struct or variable
%              name, the name will be trucated at 63. Please consider using
%              the name as a containers.Map key, which does not have such
%              limit.
%
%    example:
%        encodevarname('_a')   % returns x0x5F_a
%        encodevarname('a_')   % returns a_ as it is a valid variable name
%        encodevarname('变量')  % returns 'x0xE58F98__0xE9878F_' 
%

%==== function newname = decodevarname(name,varargin) ====
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

%%=== # Miscellaneous functions ===

%==== function val=jsonopt(key,default,varargin) ====
%
% val=jsonopt(key,default,optstruct)
%
% setting options based on a struct. The struct can be produced
% by varargin2struct from a list of 'param','value' pairs
%
%
% input:
%      key: a string with which one look up a value from a struct
%      default: if the key does not exist, return default
%      optstruct: a struct where each sub-field is a key 
%
% output:
%      val: if key exists, val=optstruct.key; otherwise val=default
%
% license:
%     BSD or GPL version 3, see LICENSE_{BSD,GPLv3}.txt files for details
%

%==== function s=mergestruct(s1,s2) ====
%
% s=mergestruct(s1,s2)
%
% merge two struct objects into one
%
%
% input:
%      s1,s2: a struct object, s1 and s2 can not be arrays
%
% output:
%      s: the merged struct object. fields in s1 and s2 will be combined in s.
%
% license:
%     BSD or GPL version 3, see LICENSE_{BSD,GPLv3}.txt files for details 
%

%==== function opt=varargin2struct(varargin) ====
%
% opt=varargin2struct('param1',value1,'param2',value2,...)
%   or
% opt=varargin2struct(...,optstruct,...)
%
% convert a series of input parameters into a structure
%
%
% input:
%      'param', value: the input parameters should be pairs of a string and a value
%       optstruct: if a parameter is a struct, the fields will be merged to the output struct
%
% output:
%      opt: a struct where opt.param1=value1, opt.param2=value2 ...
%
% license:
%     BSD or GPL version 3, see LICENSE_{BSD,GPLv3}.txt files for details 
%

%==== function [endpos, maxlevel] = match_bracket(str,startpos,brackets) ====
%
% [endpos, maxlevel] = match_bracket(str,startpos,brackets)
%
% Looking for the position of a closing bracket token in a string
%
%
% input:
%      str: the full string to be searched
%      startpos: the index in the string as the start position to search; the
%               startpos must be at least 1 greater than the opening bracket position
%      brackets: (optional), a string of length 2, with the first character
%               being the opening token and the 2nd being the closing token.
%               if not given, brackets is set to '[]' to find matching square-brackets;
%               for example, '{}' looks for a matching closing curly-bracket in
%               the string key(pos(startpos,:end))
%
% output:
%      endpos: if a matching bracket is found, return its position in the original 
%              string
%      maxlevel: return the depth of the enclosed brackets between the searched pair,
%              includig the searching pair. For example, the matching closing-bracket 
%              of the 1st square bracket (startpos=2) in  '[[[]],[]]' returns a 
%              position of 9, with a maximum depth of 3; searching for the closing 
%              bracket for the 2nd square bracket (startpos=3) returns a position of 
%              5 and max-depth of 2.
%
% example:
%      str='[[ [1,2], 1], 10, [5,10] ]';
%      [p1,dep]=match_bracket(str,3)
%      [p2,dep]=match_bracket(str,2)
%      [p3,dep]=match_bracket(str,3)
%
% license:
%     BSD or GPL version 3, see LICENSE_{BSD,GPLv3}.txt files for details
%

%==== function [endpos, maxlevel] = fast_match_bracket(key,pos,startpos,brackets) ====
%
% [endpos, maxlevel] = fast_match_bracket(key,pos,startpos,brackets)
%
% A fast function to find the position of a closing bracket token in a string
%
%
% input:
%      key: a preprocessed string containing only relevant opening/closing 
%           bracket characters for accelerating the search.
%      pos: a 1D integer vector with a length matching the length of key, 
%           recording the corresponding position of each char. in the original string.
%      startpos: the index in the original string as the start position to search; the
%               startpos must be at least 1 greater than the opening bracket position
%      brackets: (optional), a string of length 2, with the first character
%               being the opening token and the 2nd being the closing token.
%               if not given, brackets is set to '[]' to find matching square-brackets;
%               for example, '{}' looks for a matching closing curly-bracket in
%               the string key(pos(startpos,:end))
%
% output:
%      endpos: if a matching bracket is found, return its position in the original 
%              string
%      maxlevel: return the depth of the enclosed brackets between the searched pair,
%              includig the searching pair. For example, the matching closing-bracket 
%              of the 1st square bracket (startpos=2) in  '[[[]],[]]' returns a 
%              position of 9, with a maximum depth of 3; searching for the closing 
%              bracket for the 2nd square bracket (startpos=3) returns a position of 
%              5 and max-depth of 2.
%
% example:
%      str='[[ [1,2], 1], 10, [5,10] ]';
%      pos=find(str=='[' | str==']')
%      key=str(pos)
%      [p1,dep]=fast_match_bracket(key,1:length(key),3)
%      [p2,dep]=fast_match_bracket(key,pos,2)
%      [p3,dep]=fast_match_bracket(key,pos,3)
%
% license:
%     BSD or GPL version 3, see LICENSE_{BSD,GPLv3}.txt files for details
%

%==== function [dims, maxlevel, count] = nestbracket2dim(str,brackets) ====
%
% [dims, maxlevel, count] = nestbracket2dim(str,brackets)
%
% Extracting the dimension vector of a JSON string formatted array
% by analyzing the pairs of opening/closing bracket tokenss; this function 
% only returns valid dimension information when the array is an N-D array
%
%
% input:
%      str: a string-formatted JSON array using square-brackets for enclosing
%           elements and comma as separators between elements
%      brackets: (optional), a string of length 2, with the first character
%               being the opening token and the 2nd being the closing token.
%               if not given, brackets is set to '[]' to find matching square-brackets;
%               for example, '{}' looks for a matching closing curly-bracket in
%               the string key(pos(startpos,:end))
%
% output:
%      dims: the speculated dimension vector with the length matching the maximum 
%            depth of the embedded bracket pairs. When the input string encodes an
%            N-D array, the dims vector contains all integers; however, returning
%            an all-integer dims vector does not mean the array is rectangular.
%      maxlevel: return the depth of the enclosed brackets in the string, i.e. the
%            length of the dims vector.
%      count: the relative depth from the level 0 - scanning from the left
%            to right of the string, an opening token increases the level by 1
%            and a closing token decreases the level by 1; a zero indicates
%            the positions of a matching bracket of the same level.
%
% example:
%      str='[[ [1,2,3], [4,2,1]], [ [10,1,0], [2,5,10]] ]'; % an N-D array
%      [dim,dep]=nestbracket2dim(str)
%      str='[[ [1,2,3], [4,2,1]], [ [10,1,0], [2,5]] ]'; % an invalid N-D array
%      [dim,dep]=nestbracket2dim(str)
% license:
%     BSD or GPL version 3, see LICENSE_{BSD,GPLv3}.txt files for details
%

