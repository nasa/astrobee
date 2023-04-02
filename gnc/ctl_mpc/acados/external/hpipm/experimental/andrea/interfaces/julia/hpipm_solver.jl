type hpipm_solver
    # "attribtues"
    py_hpipm_solver
    py_hpipm_data
    py_hpipm_dims

    # "methods"
    solve
    print_sol

    function hpipm_solver(ju_qp_dims, ju_qp_data)
        this = new()

        py_qp_data = hp.hpipm_data()
        this.py_hpipm_data = ju2py_obj(ju_qp_data, py_qp_data) 

        py_qp_dims = hp.hpipm_dims()
        this.py_hpipm_dims = ju2py_obj(ju_qp_dims, py_qp_dims) 

        py_qp_solver = hp.hpipm_solver(this.py_hpipm_dims, this.py_hpipm_data)
        this.py_hpipm_solver = py_qp_solver

        this.solve = function()
            (this.py_hpipm_solver)[:solve]()
        end

        this.print_sol = function()
            this.py_hpipm_solver[:print_sol]()
        end

        return this
    end
end
