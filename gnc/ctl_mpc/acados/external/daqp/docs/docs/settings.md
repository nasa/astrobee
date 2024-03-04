---
layout: page
title: Parameters 
permalink: /parameters/
nav: 3 
---
<details open markdown="block">
<summary>
Table of contents
</summary>
{: .text-delta }
1. TOC
{:toc}
</details>


## Settings

|  Parameter |  Description| Default |
|:-------------|:------------------|:------:|
| `primal_tol`  | Tolerance for primal infeasibility|  1e-6 |
| `dual_tol`	 | Tolerance for dual infeasibility| 1e-12|
| `zero_tol` | Values below are regarded as zero | 1e-14|
| `pivot_tol` | Value used for determining if rows in the LDL' factorization should be exchanged. A higher value improves stability. | 1e-4|
| `progress_tol` | Minimum change in objective function to consider it progress | 1e-6|
| `cycle_tol` | Allowed number of iterations without progress before terminating| 10 |
| `iter_limit` | Maximum number of iterations before terminating| 1000 |
| `fval_bound` | Maximum allowed objective function value. The solver terminates if the dual objective exceeds this value (since it is a lower bound of the optimal value). | 1e30|
| `eps_prox` | Regularization parameter used for proximal-point iterations (0 means that no proximal-point iterations are performed) | 0|
| `eta_prox` | Tolerance that determines if a fix-point has been reached during proximal-point iterations | 1e-6|
| `rho_soft` | Weight used for soft constraints (higher enables more violations) | 1e-3|
| `rel_subopt` | Allowed relative suboptimality in branch and bound | 0 |
| `abs_subopt` | Allowed absolute suboptimality in branch and bound | 0 |


## Exit flags 

|Value|Status |
|:-:|:-------|
|2|Soft optimal|
|1|Optimal |
|-1|Infeasible|
|-2|Cycling detected|
|-3|Unbounded problem|
|-4|Iteration limit reached|
|-5|Nonconvex problem|
|-6|Initial working set overdetermined|

## Constraint classification
The type of a constraint is classified through an integer value (called sense), where the bits in this integer encode different properties: 

|Value| Classification|Description
|:-:|:-------|:--|
|1|Active| The constraint should be imposed as an equality at the start|
|2|Lower| Determine which of the lower and upper bound is imposed when active| 
|4|Immutable| Disallow the constraint to be activated/deactivated|
|8|Soft| The constraint should be softened | 
|16|Binary| Either the upper or lower bound should hold with equality|

These flags can be combined through addition. For example, if we want to encode equality constraints in our problem we would mark these as active and immutable, that is, with sense 1+4=5.
