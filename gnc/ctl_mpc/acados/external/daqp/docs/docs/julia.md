---
layout: page
title: Julia 
permalink: /start/julia
nav: 3 
parent: Interfaces 
grand_parent: Getting started 
math: mathjax3
---


## Setting up the problem
In Julia we define the problem as 
```julia
# Define the problem
H = [1.0 0; 0 1];
f = [1.0 ;1];
A = [1.0 1; 1 -1];
bupper = [1.0 ; 2; 3; 4];
blower = [-1.0; -2; -3; -4];
sense = zeros(Cint,4);
```
`sense` determines the type of the constraints (more details are given [here](/daqp/parameters/#constraint-classification)).

Note: When $$b_u$$ and $$b_l$$ has more elements than the number of rows in $$A$$, the first elements in $$b_u$$ and $$b_l$$ are interpreted as simple bounds. 

## Calling DAQP
There are two ways of calling DAQP in Julia. The first way is through a quadprog call: 
```julia
x,fval,exitflag,info = DAQP.quadprog(H,f,A,bupper,blower,sense);
```
This will solve the problem with default settings. A more flexible interface is also offered, where we first setup the problem and then solve it 
```julia
d = DAQP.Model();
DAQP.setup(d,H,f,A,bupper,blower,sense);
x,fval,exitflag,info = DAQP.solve(d);
```
This allows us to reuse internal matrix factorization if we want to solve a perturbed problem. 

## Changing settings
If we, for example, want to change the maximum number of iterations to 2000 we can do so by
```julia
DAQP.settings(d; Dict(:iter_limit =>2000))
```

A full list of available settings is provided [here](/daqp/parameters/#settings)

## Using DAQP in JuMP
DAQP can also be interfaced to [JuMP](https://jump.dev/). The following code sets up and solves the problem considered above

```julia
using DAQP
using JuMP

## Setup problem
model = Model(DAQP.Optimizer)
@variable(model, -1<= x1 <=1)
@variable(model, -2<= x2 <=2)
@objective(model, Min, 0.5*(x1^2+x2^2)+x1+x2)
@constraint(model, c1, -3 <= x1 + x2 <= 3)
@constraint(model, c2, -4 <= x1 - x2 <= 4)

## Solve problem
optimize!(model)
```
