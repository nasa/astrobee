## Test quadprog
import daqp
import numpy as np
from ctypes import * 
import ctypes.util

H = np.array([[1, 0], [0, 1]],dtype=c_double)
f = np.array([2, 2],dtype=c_double)
A = np.array([[1, 0], [0, 1]],dtype=c_double)
bupper = np.array([1,1],dtype=c_double)
blower= np.array([-1,-1],dtype=c_double)
sense = np.array([0,0],dtype=c_int)

m = daqp.daqp()
(xstar,fval,exitflag,info) = m.quadprog(H,f,A,bupper,blower,sense)
print("Optimal solution:")
print(xstar)
print("Exit flag:",exitflag)
print("Solver info:", info)
