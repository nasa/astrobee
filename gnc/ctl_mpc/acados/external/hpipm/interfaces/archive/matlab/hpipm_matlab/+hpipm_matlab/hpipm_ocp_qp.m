classdef hpipm_ocp_qp
	


	properties
		hp
		np
		py_qp
	end



	methods
		

		function obj = hpipm_ocp_qp(dims)
			obj.np = py.importlib.import_module('numpy');
			obj.hp = py.importlib.import_module('hpipm_python');
			obj.py_qp = obj.hp.hpipm_ocp_qp(dims.py_dims);
		end
	

		function set(obj, varargin)
			import hpipm_matlab.*
			if (length(varargin)==2)
				field = varargin{1};
				mat = varargin{2};
				py_mat = py.list({});
				for i=1:length(mat)
					mat0 = mat{i};
					py_mat.append(m2py(mat0, obj.np));
				end
				obj.py_qp.set(field, py_mat);
			else
				field = varargin{1};
				mat0 = varargin{2};
				idx = varargin{3};
				obj.py_qp.set(field, m2py(mat0, obj.np), int32(idx));
			end
		end

		function print_C_struct(obj)
			obj.py_qp.print_C_struct();
		end

	end

end



