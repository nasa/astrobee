---
layout: default
title: About 
nav_order: 1
description: "DAQP is a dual active-set solver for convex quadratic programs"
permalink: /
math: mathjax3
---
## Background

DAQP is a dual active-set solver that solves quadratic programs (QPs) in the form 

$$\begin{aligned}
&\underset{x}{\text{minimize}}&& \frac{1}{2} x^T H x + f^T x\\
&\text{subject to} && l\:\: \leq \:x \:\:\leq u, \\
& && b_l \leq A x \leq b_u, \\
\end{aligned}$$

where $$H\succ 0$$. The case when $$H\succeq 0$$ is also handled through proximal-point iterations, which in particular means that DAQP also solves linear programs (LPs). Moreover, DAQP can also solve mixed-integer QPs by handling binary constraints of the form $$Ax \in \lbrace b_l, b_u \rbrace$$ (for which $$x_i \in \lbrace 0,1 \rbrace $$ is a special case).

DAQP has been developed for QPs that arise in real-time Model Predictive Control (MPC) applications, with focus on fully condensed MPC formulations. As such, DAQP efficiently solves small/medium scale, dense, QPs and LPs. If your aim is to solve large-scale sparse problems, consider instead solvers, such as [OSQP](https://osqp.org/), that exploit sparsity.

For technical details and numerical results, see the papers

**A Dual Active-Set Solver for Embedded Quadratic Programming Using Recursive LDL$$^T$$ Updates** <br>
D. Arnström, A. Bemporad, D. Axehill <br>
*IEEE Transactions on Automatic Control*, vol. 67, no. 8, pp. 4362-4369, 2022

**A Linear Programming Method Based on Proximal-Point Iterations With Applications to Multi-Parametric Programming** <br>
D. Arnström, A. Bemporad, D. Axehill <br>
*IEEE Control Systems Letters*, vol. 6, pp. 2066-2071, 2022

## Code
*Available [here](https://github.com/darnstrom/daqp)*. <br>
The solver is written in C and is *library free*. DAQP can be interfaced to C, Julia, MATLAB, and Python.  
