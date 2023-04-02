---
layout: page
title: Python 
permalink: /start/python
nav: 3 
parent: Interfaces 
grand_parent: Getting started 
math: mathjax3
---


## Setting up the problem
In Python we define the problem as 
```python
# Import relevant modules 
import daqp
import numpy as np
from ctypes import * 
import ctypes.util

# Define the problem
H = np.array([[1, 0], [0, 1]],dtype=c_double)
f = np.array([1, 1],dtype=c_double)
A = np.array([[1, 1], [1, -1]],dtype=c_double)
bupper = np.array([1,2,3,4],dtype=c_double)
blower = np.array([-1,-2,-3,-4],dtype=c_double)
sense = np.array([0,0,0,0],dtype=c_int)

```
`sense` determines the type of the constraints (more details are given [here](/daqp/parameters/#constraint-classification)).

Note: When $$b_u$$ and $$b_l$$ has more elements than the number of rows in $$A$$, the first elements in $$b_u$$ and $$b_l$$ are interpreted as simple bounds. 

## Calling DAQP
DAQP can be called through a quadprog call: 
```python
d = daqp.daqp()
(xstar,fval,exitflag,info) = d.quadprog(H,f,A,bupper,blower,sense)
```
