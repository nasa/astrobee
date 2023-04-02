classdef hpipm_ocp_qp_dim
	
	properties
		hp
		np
		py_dims
	end

	methods

		function obj = hpipm_ocp_qp_dim(N)
			obj.np = py.importlib.import_module('numpy');
			obj.hp = py.importlib.import_module('hpipm_python');
			obj.py_dims = obj.hp.hpipm_ocp_qp_dim(int32(N));
		end

		function set(obj, varargin)
			if(length(varargin)==2)
				field = varargin{1};
				n_dim = varargin{2};
				m = length(n_dim);
				py_n_dim = obj.np.zeros({int32(m), int32(1)}, obj.np.int32);
				for i=1:m
					py_n_dim.itemset(int32(i-1), int32(0), int32(n_dim(i)));
				end
				obj.py_dims.set(field, py_n_dim);
			else
				field = varargin{1};
				n_dim = varargin{2};
				idx = varargin{3};
				obj.py_dims.set(field, int32(n_dim), int32(idx));
			end
		end

		function print_C_struct(obj)
			obj.py_dims.print_C_struct();
		end
	end
end
