HPMPC -- Library for High-Performance implementation of solvers for MPC.

The library aims at providing routines for high-performance implementation of solvers for linear MPC and MHE. Critical linear-algebra routines are highly optimized for a number of different computer architectures. These routines are used to efficiently implement a Riccati recursion solver for the uncontrained MPC and MHE problems (LQCP), that in turn is the key routine in solvers for constrained MPC problems. At the moment, Interior-Point (IP) method and ADMM (Alternating Direction Method of Multipliers) solvers are available for both box and soft constrained MPC problems.

The code is highly-optimized for a number of common architectures, plus a reference version in plain C code. The target architecture can be set in the configuration file

hpmpc_main_folder/Makefile.rule

The configuration file provides a good choice of compiler and compiler flags for supported architectures, and it can be used to set different values if needed. However, notice that the code is intended to be compiled using gcc or clang, and some optimized routine may not work with other compilers.

The folder test_problems contains some test problem for the linear-algebra, for the LQCP solvers and for the MPC solvers. The test problem can be chosen by editing the file 

hpmpc_main_folder/test_problems/Makefile

An higher level C interface is provided in the folder hpmpc_main_folder/interfaces/c. This interface is used to provide Octave wrappers using mex files and Octave test problems in the folder hpmpc_main_folder/interfaces/octave.

The code comes as a library, that can solve problems of every size. It is generated typing in a terminal the command

$ make_static_library

for the static library and

$ make_shared_library

for the dynamic libray. The command

$ make

builds the static library and builds and runs the test problem. Static library and headers can be installed using the command

$ make_install_static_library

while the dynamic library and headers can be installed using the command

$ make_install_shared_library

Alternatively, the library can be build using Cmake. This can be done by creating the folder hpmpc_main_folder/build and then type in a terminal the command

$ cd hpmcp_main_folder/build
$ cmake ..
$ make

More documentation will be available soon.

Questions and comments can be send to the author Gianluca Frison, at the email addresses

giaf (at) dtu.dk
gianluca.frison (at) imtek.uni-freiburg.de


