# ENV["PYTHON"] = /usr/bin/python3
# Pkg.build("PyCall")

using PyCall
@pyimport hpipm_python as hp
@pyimport numpy as np

qp_data = hp.hpipm_data()

A = np.array([1, 0, 1, 1])
B = np.array([0, 1])
b = np.array([0, 0])

Q = np.array([1, 0, 0, 1])
S = np.array([0, 0])
R = np.array([1])
q = np.array([1, 1])
r = np.array([0])

qp_data[:A] = PyVector([A, A, A, A, A])
qp_data[:B] = PyVector([B, B, B, B, B])
qp_data[:b] = PyVector([b, b, b, b, b])
qp_data[:Q] = PyVector([Q, Q, Q, Q, Q, Q])
qp_data[:S] = PyVector([S, S, S, S, S, S])
qp_data[:R] = PyVector([R, R, R, R, R, R])
qp_data[:q] = PyVector([q, q, q, q, q, q])
qp_data[:r] = PyVector([r, r, r, r, r, r])

x0 = np.array([1, 1])

qp_data[:d_lb] = PyVector([x0])
qp_data[:d_ub] = PyVector([x0])

qp_data[:idxb] = PyVector([np.array([1, 2])])

qp_dims = hp.hpipm_dims()

qp_dims[:nx]   = np.array([2, 2, 2, 2, 2, 2])
qp_dims[:nu]   = np.array([1, 1, 1, 1, 1, 0])
qp_dims[:nb]   = np.array([2, 0, 0, 0, 0, 0])
qp_dims[:nbx]  = np.array([2, 0, 0, 0, 0, 0])
qp_dims[:nbu]  = np.array([0, 0, 0, 0, 0, 0])
qp_dims[:ng]   = np.array([0, 0, 0, 0, 0, 0])
qp_dims[:ns]   = np.array([0, 0, 0, 0, 0, 0])
qp_dims[:nsbx] = np.array([0, 0, 0, 0, 0, 0])
qp_dims[:nsbu] = np.array([0, 0, 0, 0, 0, 0])
qp_dims[:nsg]  = np.array([0, 0, 0, 0, 0, 0])
qp_dims[:N]    = 5

# set up solver
solver = hp.hpipm_solver(qp_dims, qp_data)

# solve qp
return_flag = solver[:solve]()

println("HPIPM returned with flag $(return_flag)\n")

if return_flag == 0
    println("-> QP solved Solution:\n")
    solver[:print_sol]()
else
    println("-> Solver failed.\n")
end
