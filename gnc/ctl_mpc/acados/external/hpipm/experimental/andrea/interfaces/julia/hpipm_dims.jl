function hpipm_dims()
    py_qp_dims = hp.hpipm_dims()
    ju_qp_dims = py2ju_obj(py_qp_dims)
    return ju_qp_dims
end
