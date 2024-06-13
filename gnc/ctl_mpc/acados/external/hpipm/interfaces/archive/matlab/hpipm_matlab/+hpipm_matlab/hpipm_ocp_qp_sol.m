classdef hpipm_ocp_qp_sol
	


	properties
		hp
		np
		py_qp_sol
	end



	methods
		

		function obj = hpipm_ocp_qp_sol(dims)
%			import hpipm_matlab.*;
			obj.np = py.importlib.import_module('numpy');
			obj.hp = py.importlib.import_module('hpipm_python');
			obj.py_qp_sol = obj.hp.hpipm_ocp_qp_sol(dims.py_dims);
		end
	

		function u = get_u(obj, varargin)
			import hpipm_matlab.*
			if (length(varargin)==0)
				u = {};
				py_u = obj.py_qp_sol.get_u();
				shape = obj.np.shape(py_u);
				m = int64(shape{1});
				for ii=1:m
					u{ii} = py2m(py_u{ii}, obj.np);
				end
			else
				idx = varargin{1};
				py_u = obj.py_qp_sol.get_u(int32(idx));
				u = py2m(py_u, obj.np);
			end
		end


		function x = get_x(obj, varargin)
			import hpipm_matlab.*
			if (length(varargin)==0)
				x = {};
				py_x = obj.py_qp_sol.get_x();
				shape = obj.np.shape(py_x);
				m = int64(shape{1});
				for ii=1:m
					x{ii} = py2m(py_x{ii}, obj.np);
				end
			else
				idx = varargin{1};
				py_x = obj.py_qp_sol.get_x(int32(idx));
				x = py2m(py_x, obj.np);
			end
		end


		function print_C_struct(obj)
			obj.py_qp_sol.print_C_struct();
		end


	end

end
