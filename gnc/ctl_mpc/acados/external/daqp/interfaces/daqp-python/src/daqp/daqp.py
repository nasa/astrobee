import numpy as np
from ctypes import * 
import ctypes.util
import os.path
import platform
import daqp.types as types

class daqp:
    def __init__(self):
        # load library
        if platform.system()=='Windows':
            try: 
                self._daqp=CDLL("libdaqp.dll")
            except:
                try: 
                    self._daqp=CDLL("daqp.dll")
                except:
                    print("Could not locate .dll; Make sure DAQP is installed correctly.")
        else: # Unix
            try:
                self._daqp=CDLL("libdaqp.so")
            except:
                try:
                    self._daqp=CDLL("/usr/local/lib/libdaqp.so")
                except:
                    print("Could not locate .so; Make sure DAQP is installed correctly.")

    def solve(self):
        self._daqp.daqp_solve(self.work)

    def quadprog(self, H=None,f=None,
            A=None,bupper=None,blower=None,sense=None, bin_ids=None, **settings):
        (mA, n) = np.shape(A)
        m = np.size(bupper)
        ms = m-mA
        bin_ids_cand = np.where(sense&16)[0]
        nb = np.size(bin_ids_cand) 
        if nb > 0:
            bin_ids = np.array(bin_ids_cand,dtype=c_int)

        # Setup qp, settings and result
        qp = types.QP(n,m,ms,
                np.ascontiguousarray(H).ctypes.data_as(POINTER(c_double)),
                np.ascontiguousarray(f).ctypes.data_as(POINTER(c_double)),
                np.ascontiguousarray(A).ctypes.data_as(POINTER(c_double)),
                np.ascontiguousarray(bupper).ctypes.data_as(POINTER(c_double)),
                np.ascontiguousarray(blower).ctypes.data_as(POINTER(c_double)),
                np.ascontiguousarray(sense).ctypes.data_as(POINTER(c_int)),
                np.ascontiguousarray(bin_ids).ctypes.data_as(POINTER(c_int)),
                nb)
        daqp_options = types.DAQPSettings()
        # Create struct to put result in
        result = types.DAQPResult()
        x = np.zeros([n,1]);
        lam = np.zeros([m,1]);
        result.x = np.ascontiguousarray(x).ctypes.data_as(POINTER(c_double));
        result.lam = np.ascontiguousarray(lam).ctypes.data_as(POINTER(c_double));
        # Call C api
        self._daqp.daqp_quadprog(byref(result),byref(qp),byref(daqp_options))
        # Collect results 
        info = {'solve_time':result.solve_time,
                'setup_time': result.setup_time,
                'iterations': result.iter,
                'nodes': result.nodes,
                'lam': lam}
        return x, result.fval, result.exitflag, info

    def linprog(self, f=None,
            A=None,bupper=None,blower=None,sense=None, **settings):
        self.quadprog(None,f,A,bupper,blower,sense, settings)
