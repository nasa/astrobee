classdef hpipm_solver
   properties
	  py_hpipm_solver
	  py_qp_data
	  py_qp_dims
   end
   
   methods
	   function obj = hpipm_solver(m_qp_dims, m_qp_data)
			import hpipm_matlab.*
			hp = py.importlib.import_module('hpipm_python');
			py_qp_dims = hp.hpipm_dims();
			obj.py_qp_dims = m2py_obj(m_qp_dims, py_qp_dims);
			
			py_qp_data = hp.hpipm_data();
			obj.py_qp_data = m2py_obj(m_qp_data, py_qp_data);
			obj.py_hpipm_solver = hp.hpipm_solver(py_qp_dims, py_qp_data);		   
	   end
	   
	   function return_flag = solve(obj)
		   return_flag = int64(obj.py_hpipm_solver.solve().real);
	   end

	   function print_sol(obj)
		   obj.py_hpipm_solver.print_sol()
	   end
   end
end

