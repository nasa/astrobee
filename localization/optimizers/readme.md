\page optimizer 

# Package Overview
The optimizer package provides a base class optimizer that is used by the graph optimizer class. Optimizers perform optimization using GTSAM factors and values, for more information on these see the GTSAM package here: https://github.com/borglab/gtsam. Optimizers also provided covariance matrices for values given their keys. Two implementations are provided - the nonlinear optimizer and isam2 optimizer.

## Nonlinear Optimizer
The nonlinear optimizer performs Levenberg-Marquardt nonlinear optimization.

## ISAM2 Optimizer
The ISAM2 optimizer performs optimization using ISAM2. Note this is still in development and not yet tested.

