module hpipm_julia
    using PyCall
    @pyimport numpy as np
    @pyimport hpipm_python as hp

    export hpipm_data, hpipm_dims, hpipm_solver
    include("py2ju_obj.jl")
    include("ju2py_obj.jl")
    include("hpipm_data.jl")
    include("hpipm_dims.jl")
    include("hpipm_solver.jl")
end
