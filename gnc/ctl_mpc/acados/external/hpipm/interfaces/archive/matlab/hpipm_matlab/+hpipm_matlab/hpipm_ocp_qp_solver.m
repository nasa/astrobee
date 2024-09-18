classdef hpipm_ocp_qp_solver
	


	properties
		hp
		np
		py_arg
		py_solver
	end



	methods
		

		function obj = hpipm_ocp_qp_solver(dim, arg)
%			import hpipm_matlab.*;
			obj.np = py.importlib.import_module('numpy');
			obj.hp = py.importlib.import_module('hpipm_python');
			obj.py_solver = obj.hp.hpipm_ocp_qp_solver(dim.py_dims, arg.py_arg);
			obj.py_arg = arg.py_arg;
		end


		function return_flag = solve(obj, qp, qp_sol)
			py_flag = obj.py_solver.solve(qp.py_qp, qp_sol.py_qp_sol);
			return_flag = int64(py_flag.real);
		end


		function res = get_res_stat(obj)
			res = obj.py_solver.get_res_stat();
		end


		function res = get_res_eq(obj)
			res = obj.py_solver.get_res_eq();
		end


		function res = get_res_ineq(obj)
			res = obj.py_solver.get_res_ineq();
		end


		function res = get_res_comp(obj)
			res = obj.py_solver.get_res_comp();
		end


		function iters = get_iter(obj)
			py_iters = obj.py_solver.get_iter();
			iters = int64(py_iters.real);
		end


		function res = get_stat(obj)
			import hpipm_matlab.*;
			py_res = obj.py_solver.get_stat();
			res = py2m(py_res, obj.np);
		end


	end

end
	
