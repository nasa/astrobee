function m_qp_dims = hpipm_dims()
    import hpipm_matlab.*
    hp = py.importlib.import_module('hpipm_python');
    py_qp_dims = hp.hpipm_dims();
    m_qp_dims = py2m_obj(py_qp_dims);
end