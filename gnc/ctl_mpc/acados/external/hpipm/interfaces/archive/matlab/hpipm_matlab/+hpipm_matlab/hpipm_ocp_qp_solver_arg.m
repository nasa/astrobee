classdef hpipm_ocp_qp_solver_arg
	


	properties
		hp
		np
		py_arg
	end



	methods
		

		function obj = hpipm_ocp_qp_solver_arg(dims)
%			import hpipm_matlab.*;
			obj.np = py.importlib.import_module('numpy');
			obj.hp = py.importlib.import_module('hpipm_python');
			obj.py_arg = obj.hp.hpipm_ocp_qp_solver_arg(dims.py_dims);
		end


		function set_mu0(obj, mu0)
			obj.py_arg.set_mu0(mu0);
		end


		function set_iter_max(obj, iter_max)
			obj.py_arg.set_iter_max(int32(iter_max));
		end


		function set_tol_stat(obj, tol_stat)
			obj.py_arg.set_tol_stat(tol_stat);
		end


		function set_tol_eq(obj, tol_eq)
			obj.py_arg.set_tol_eq(tol_eq);
		end


		function set_tol_ineq(obj, tol_ineq)
			obj.py_arg.set_tol_ineq(tol_ineq);
		end


		function set_tol_comp(obj, tol_comp)
			obj.py_arg.set_tol_comp(tol_comp);
		end


		function set_reg_prim(obj, reg_prim)
			obj.py_arg.set_reg_prim(reg_prim);
		end


	end

end
	

