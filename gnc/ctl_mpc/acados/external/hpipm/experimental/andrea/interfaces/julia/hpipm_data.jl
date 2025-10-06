function hpipm_data()
    py_qp_data = hp.hpipm_data()
    ju_qp_data = py2ju_obj(py_qp_data)
    return ju_qp_data
end
