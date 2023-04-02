"""
Test the python interface to daqp.

Runs in Github CI
"""
import unittest
from ctypes import c_double, c_int

import daqp

import numpy as np


class Testing(unittest.TestCase):
    """Testing class for daqp python interface."""

    def test_python_demo(self):
        """Python demo code test."""
        # Test camera movement
        H = np.array([[1, 0], [0, 1]], dtype=c_double)
        f = np.array([1, 1], dtype=c_double)
        A = np.array([[1, 1], [1, -1]], dtype=c_double)
        bupper = np.array([1, 2, 3, 4], dtype=c_double)
        blower = np.array([-1, -2, -3, -4], dtype=c_double)
        sense = np.array([0, 0, 0, 0], dtype=c_int)
        d = daqp.daqp()
        (xstar, fval, exitflag, info) = d.quadprog(H, f, A, bupper, blower, sense)
        self.assertEqual(exitflag, 1)


if __name__ == '__main__':
    unittest.main()
