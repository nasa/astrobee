function m_qp_data = hpipm_data()
	import hpipm_matlab.*
	hp = py.importlib.import_module('hpipm_python');
	py_qp_data = hp.hpipm_data();
	m_qp_data = py2m_obj(py_qp_data);
end
