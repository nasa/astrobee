---
layout: page
title: acados 
permalink: /start/acados
nav: 3 
parent: Interfaces 
grand_parent: Getting started 
math: mathjax3
---
DAQP can be used as a solver in [acados](https://docs.acados.org/), a modular framework for fast embedded optimal control.

## Installation
To install acados with support for DAQP, pass the flag `ACADOS_WITH_DAQP` when calling `cmake` during the installation steps [here](https://docs.acados.org/installation/index.html).

## Usage 
Below are instruction on how DAQP can be used as QP solver in acados. For details on how problems are setup up and solved, see the [examples](https://docs.acados.org/examples/index.html) in acados.

### C 
In the acados [C interface](https://docs.acados.org/c_interface/index.html) DAQP can be invoked as the inner QP solver by setting the field `qp_solver` of an `ocp_qp_solver_plan_t` to `FULL_CONDENSING_DAQP`
### MATLAB
In the acados [MATLAB interface](https://docs.acados.org/matlab_octave_interface/index.html), DAQP can be invoked as the inner QP solver by setting the solver option `qp_solver` to `full_condensing_daqp`. 

### Python
In the acados [Python interface](https://docs.acados.org/python_interface/index.html), DAQP can be invoked as the inner QP solver by setting the solver option `qp_solver` to `FULL_CONDENSING_DAQP`. 
