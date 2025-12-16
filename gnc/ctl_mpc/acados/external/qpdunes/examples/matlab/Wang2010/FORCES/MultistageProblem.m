function stages = MultistageProblem(N)
% Default initialization for a multistage problem.
%
%    STAGES = MULTISTAGEPROBLEM(N) returns an array of structs defining a 
%    multistage problem of with N stages:
%
%        min   sum_{i=1}^N z_i'*H*z_i + f_i'*z_i
%       {z_i}
% 
%     subject to:              
%                  lb_i <= z_i <= ub_i (bound constraints),      i=1,...,N
%                  Aineq_i*z_i <= b_i  (affine inequalities),    i=1,...,N
%                  z_i'*Q_{i,j}*z_i + g_{i,j}'*z_i <= r_{i,j}^2, i=1,...,N,
%                                      (quadratic inequalities)  j=1,...q
%
%    Both H and Q are supposed to be positive definite.
%
%    The ith stage of the problem is accessed by STAGES(i), and has the
%    following fields:
%
% stages(i)
%       .dims.n - length of stage variable
%            .m - number of affine equalities (only for stages 1..N-1)
%            .l - vector in (1,n) indicating the number of lower bounds
%            .u - vector in (1,n) indicating the number of upper bounds
%            .p - number of linear inequalities g'*z <= h
%            .q - number of quadratic constraints z'*M*z + g'*z <= r
%
%       .cost.H - Hessian of cost
%            .f - linear part of cost
%
%         .eq.C - cell array containing the equality constraint matrices Ci
%            .D - cell array containing the equality constraint matrices Di
%            .c - matrix containing the RHS of equality constraints ci
%
%    .ineq.b.lb - cell array with lower bounds of stage variables
%           .lbidx - cell array indicating to which stage variables the
%                    lower bounds belong to
%           .ub - cell array with upper bounds of stage variables
%           .ubidx - cell array indicating to which stage variables the
%                    upper bounds belong to
%            
%     .ineq.p.A - cell array containing the polytopic inequalities A*z <= b
%           p.b - matrix, RHS of polytopic inequalities
%  
%     .ineq.q.P - cell array containing the Hessians of quadratic
%                 inequalities z'*Q*z + g'*z <= r
%            .G - linear part of quadratic inequalities ( N x q matrix )
%            .r - RHS of quadratic inequalities ( q x 1 vector )
%
% See also NEWPARAM NEWOUTPUT FORCES_LICENSE

for i = 1:N
    
    % dimensions
    stages(i).dims.n = [];
    stages(i).dims.r = [];
    stages(i).dims.l = [];
    stages(i).dims.u = [];
    stages(i).dims.p = [];
    stages(i).dims.q = [];
    
    % cost
    stages(i).cost.H = [];
    stages(i).cost.f = [];
    
    % equality constraints
    stages(i).eq.C = [];
    stages(i).eq.c = [];
    stages(i).eq.D = [];
    
    % inequality constraints - bounds
    stages(i).ineq.b.lb = [];
    stages(i).ineq.b.lbidx = [];
    stages(i).ineq.b.ub = [];
    stages(i).ineq.b.ubidx = [];
    
    % inequality constraints - polytopic constraints
    stages(i).ineq.p.A = [];
    stages(i).ineq.p.b = [];
    
    % inequality constraints - quadratic constraints
    stages(i).ineq.q.Q = [];
    stages(i).ineq.q.G = [];
    stages(i).ineq.q.r = [];
    
end
