%
% Copyright 2019 Gianluca Frison, Dimitris Kouzoupis, Robin Verschueren,
% Andrea Zanelli, Niels van Duijkeren, Jonathan Frey, Tommaso Sartor,
% Branimir Novoselnik, Rien Quirynen, Rezart Qelibari, Dang Doan,
% Jonas Koenemann, Yutao Chen, Tobias Schöls, Jonas Schlagenhauf, Moritz Diehl
%
% This file is part of acados.
%
% The 2-Clause BSD License
%
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
%
% 1. Redistributions of source code must retain the above copyright notice,
% this list of conditions and the following disclaimer.
%
% 2. Redistributions in binary form must reproduce the above copyright notice,
% this list of conditions and the following disclaimer in the documentation
% and/or other materials provided with the distribution.
%
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
% POSSIBILITY OF SUCH DAMAGE.;
%
%   Author: Jonathan Frey: jonathanpaulfrey(at)gmail.com

function gnsf = reformulate_with_LOS( model, gnsf, print_info)

%% Description:
% This function takes an intitial transcription of the implicit ODE model
% "model" into "gnsf" and reformulates "gnsf" with a linear output system
% (LOS), containing as many states of the model as possible.
% Therefore it might be that the state vector and the implicit function
% vector have to be reordered. This reordered model is part of the output,
% namely reordered_model.

%% import CasADi and load models

import casadi.*

% symbolics
x = gnsf.x;
xdot = gnsf.xdot;
u = gnsf.u;
z = gnsf.z;

% dimensions
nx  = gnsf.nx;
nz  = gnsf.nz;

% get model matrices
A  = gnsf.A;
B  = gnsf.B;
C  = gnsf.C;
E  = gnsf.E;
c  = gnsf.c;

A_LO = gnsf.A_LO;

y = gnsf.y;

phi_old = gnsf.phi_expr;

if print_info
disp(' ');
disp('=================================================================');
disp(' ');
disp('================    Detect Linear Output System   ===============');
disp(' ');
disp('=================================================================');
disp(' ');
end

%% build initial I_x1 and I_x2_candidates
% I_x1: all components of x for which either xii or xdot_ii enters y;
% I_LOS_candidates: the remaining components

I_nsf_components = [];
I_LOS_candidates = [];

if gnsf.ny > 0
    for ii = 1:nx
        if or(y.which_depends(x(ii)), y.which_depends(xdot(ii)))
            % i.e. xii or xiidot are part of y, and enter phi_expr
            if print_info
                disp(['xii is part of x1, ii = ', num2str(ii)]);
            end
            I_nsf_components = union(I_nsf_components, ii);
        else
            % i.e. neither xii nor xiidot are part of y, i.e. enter phi_expr
            I_LOS_candidates = union(I_LOS_candidates, ii);
        end
    end
    if print_info
        disp(' ');
    end
    for ii = 1:nz
        if y.which_depends(z(ii))
            % i.e. xii or xiidot are part of y, and enter phi_expr
            if print_info
                disp(['zii is part of z1, ii = ', num2str(ii)]);
            end
            I_nsf_components = union(I_nsf_components, ii + nx);
        else
            % i.e. neither xii nor xiidot are part of y, i.e. enter phi_expr
            I_LOS_candidates = union(I_LOS_candidates, ii + nx);
        end
    end
else
    I_LOS_candidates = 1:(nx+nz);
end

if print_info
disp(' ');
end

new_nsf_components = I_nsf_components;
I_nsf_eq = [];
unsorted_dyn = 1:nx + nz;
xdot_z = [xdot; z];

%% determine components of Linear Output System
% determine maximal index set I_x2
% such that the components x(I_x2) can be written as a LOS
Eq_map = [];
while true
    %% find equations corresponding to new_nsf_components
    for ii = new_nsf_components
        current_var = xdot_z(ii);
        var_name = current_var.name;

        I_eq = intersect(find(E(:,ii)), unsorted_dyn);
        if length(I_eq) == 1
            i_eq = I_eq;
        elseif length(I_eq) > 1 % x_ii_dot occurs in more than 1 eq linearly
            % find the equation with least linear dependencies on
            % I_LOS_cancidates
            number_of_eq = 1;
            candidate_dependencies = zeros(length(I_eq), 1);
            I_x2_candidates = intersect(I_LOS_candidates, 1:nx);
            for eq = I_eq
                depending_candidates = union( find(E(eq, I_LOS_candidates)), ...
                    find(A(eq, I_x2_candidates)));
                candidate_dependencies(number_of_eq) = ...
                  + length(depending_candidates);
                number_of_eq = number_of_eq + 1;
            end
            [~, number_of_eq] = min(candidate_dependencies);
            i_eq = I_eq(number_of_eq);
        else %% x_ii_dot does not occur linearly in any of the unsorted dynamics
            for j = unsorted_dyn
                phi_eq_j = gnsf.phi_expr(find(C(j,:)));
                if phi_eq_j.which_depends(xdot_z(ii))
                    I_eq = union(I_eq, j);
                end
            end
            if isempty(I_eq)
                I_eq = unsorted_dyn;
            end
            % find the equation with least linear dependencies on
            % I_LOS_cancidates
            number_of_eq = 1;
            candidate_dependencies = zeros(length(I_eq), 1);
            I_x2_candidates = intersect(I_LOS_candidates, 1:nx);
            for eq = I_eq
                depending_candidates = union( find(E(eq, I_LOS_candidates)), ...
                    find(A(eq, I_x2_candidates)));
                candidate_dependencies(number_of_eq) = ...
                  + length(depending_candidates);
                number_of_eq = number_of_eq + 1;
            end
            [~, number_of_eq] = min(candidate_dependencies);
            i_eq = I_eq(number_of_eq);
            %% add 1 * [xdot,z](ii) to both sides of i_eq
            if print_info
                disp(['adding 1 * ', var_name, ' to both sides of equation ',...
                    num2str( i_eq ) , '.']);
            end
            gnsf.E(i_eq, ii) = 1;
            i_phi = find(gnsf.C(i_eq,:));
            if isempty(i_phi)
                i_phi = length(gnsf.phi_expr) + 1;
                gnsf.C( i_eq, i_phi) = 1; % add column to C with 1 entry
                gnsf.phi_expr = [ gnsf.phi_expr; 0];
            end
            gnsf.phi_expr(i_phi) = gnsf.phi_expr(i_phi) + ...
                gnsf.E(i_eq, ii) / gnsf.C(i_eq, i_phi) * xdot_z(ii);
        end
        if print_info
            disp(['detected equation ', num2str( i_eq ),...
                ' to correspond to variable ', var_name]);
        end
        I_nsf_eq = union(I_nsf_eq, i_eq);
        % remove i_eq from unsorted_dyn
        unsorted_dyn = setdiff(unsorted_dyn, i_eq);
        Eq_map = [Eq_map, [ii; i_eq]];
    end

    %% add components to I_x1
    for eq = I_nsf_eq
        I_linear_dependence = find(E(eq,:));
        I_linear_dependence = union( find(A(eq,:)), I_linear_dependence);
        I_nsf_components = union(I_linear_dependence, I_nsf_components);
        I_nsf_components = I_nsf_components(:)'; % ensure row vector for octave
    end
    %
    new_nsf_components = intersect(I_LOS_candidates, I_nsf_components);
    if isempty( new_nsf_components )
        break;
    end
    % remove new_nsf_components from candidates
    I_LOS_candidates = setdiff( I_LOS_candidates, new_nsf_components );
end

if ~isempty(Eq_map)
    [~, new_eq_order] = sort(Eq_map(1,:));
    I_nsf_eq = Eq_map(2, new_eq_order );
else
    I_nsf_eq = [];
end

I_LOS_components = I_LOS_candidates;
I_LOS_eq = setdiff( 1:nx+nz, I_nsf_eq );

I_x1 = intersect(I_nsf_components, 1:nx);
I_z1 = intersect(I_nsf_components, nx+1:nx+nz);
I_z1 = I_z1 - nx;

I_x2 = intersect(I_LOS_components, 1:nx);
I_z2 = intersect(I_LOS_components, nx+1:nx+nz);
I_z2 = I_z2 - nx;

%% permute x, xdot

if isempty(I_x1)
    x1 = [];
    x1dot = [];
else
    x1 = x(I_x1);
    x1dot = xdot(I_x1);
end

if isempty(I_x2)
    x2 = [];
    x2dot = [];
else
    x2 = x(I_x2);
    x2dot = xdot(I_x2);
end

if isempty(I_z1)
    z1 = [];
else
    z1 = z(I_z1);
end
if isempty(I_z2)
    z2 = [];
else
    z2 = z(I_z2);
end

gnsf.xdot = [x1dot; x2dot];
gnsf.x = [x1; x2];
gnsf.z = [z1; z2];

gnsf.nx1 = size(x1,1);
gnsf.nx2 = size(x2,1);
gnsf.nz1 = size(z1,1);
gnsf.nz2 = size(z2,1);

% store permutations
gnsf.idx_perm_x = [I_x1, I_x2];
gnsf.ipiv_x = idx_perm_to_ipiv(gnsf.idx_perm_x);
gnsf.idx_perm_z = [I_z1, I_z2];
gnsf.ipiv_z = idx_perm_to_ipiv(gnsf.idx_perm_z);
gnsf.idx_perm_f = [I_nsf_eq, I_LOS_eq];
gnsf.ipiv_f = idx_perm_to_ipiv(gnsf.idx_perm_f);

f_LO = SX.sym('f_LO',0,0);

%% rewrite I_LOS_eq as LOS
if gnsf.n_out == 0
    C_phi = zeros(gnsf.nx+gnsf.nz,1);
else
    C_phi = C * phi_old;
end

if gnsf.nx1 == 0
    Ax1 = zeros(gnsf.nx+gnsf.nz,1);
else
    Ax1 = A(:,I_x1) * x1;
end

if gnsf.nx1 + gnsf.nz1 == 0
    lhs_nsf = zeros(gnsf.nx+gnsf.nz,1);
else
    lhs_nsf = E(:,I_nsf_components) * [x1; z1];
end

n_LO = length(I_LOS_eq);
B_LO = zeros(n_LO, gnsf.nu);
E_LO = zeros(n_LO);
c_LO = zeros(n_LO, 1);

for eq = I_LOS_eq
    i_LO = find( I_LOS_eq == eq );
    f_LO = vertcat(f_LO, ...
            Ax1(eq) + C_phi(eq)...
            - lhs_nsf(eq)) ;
    E_LO(i_LO, :) = E(eq, I_LOS_components);
    A_LO(i_LO, :) = A(eq, I_x2);
    c_LO(i_LO, :) = c(eq);
    B_LO(i_LO, :) = B(eq, :);
end

if any(size(f_LO) == 0)
    f_LO = SX.zeros(gnsf.nx2 + gnsf.nz2,1);
end

f_LO = f_LO.simplify();
gnsf.A_LO = A_LO;
gnsf.E_LO = E_LO;
gnsf.B_LO = B_LO;
gnsf.c_LO = c_LO;
gnsf.f_lo_expr = f_LO;

%% remove I_LOS_eq from NSF type system
gnsf.A = gnsf.A(I_nsf_eq, I_x1);
gnsf.B = gnsf.B(I_nsf_eq, :);
gnsf.C = gnsf.C(I_nsf_eq, :);
gnsf.E = gnsf.E(I_nsf_eq, I_nsf_components);
gnsf.c = gnsf.c(I_nsf_eq, :);


%% reduce phi, C
I_nonzero = [];
for ii = 1:size(gnsf.C, 2) % n_colums of C
    if ~all(gnsf.C(:,ii) == 0) % if column ~= 0
        I_nonzero = union(I_nonzero, ii);
    end
end

gnsf.C = gnsf.C(:,I_nonzero);
gnsf.phi_expr = gnsf.phi_expr(I_nonzero);

gnsf = determine_input_nonlinearity_function( gnsf );
check_reformulation(model, gnsf, print_info);


gnsf.nontrivial_f_LO = 0;
if ~gnsf.f_lo_expr.is_empty()
    for ii = 1:length(gnsf.f_lo_expr)
        fii = gnsf.f_lo_expr(ii);
        if ~fii.is_zero
            gnsf.nontrivial_f_LO = 1;
        end
    end
    if ~gnsf.nontrivial_f_LO && print_info
        disp('f_LO is fully trivial (== 0)');
    end
end
check_reformulation(model, gnsf, print_info);

if print_info
    disp('');
    disp('---------------------------------------------------------------------------------');
    disp('------------- Success: Linear Output System (LOS) detected ----------------------');
    disp('---------------------------------------------------------------------------------');
    disp('');
    disp(['==>>  moved  ', num2str(gnsf.nx2), ' differential states and ',...
        num2str(gnsf.nz2),' algebraic variables to the Linear Output System']);
    disp(['==>>  recuced output dimension of phi from  ',...
        num2str(length(phi_old)), ' to ', num2str(length(gnsf.phi_expr))]);
    disp(' ');
    disp('Matrices defining the LOS read as');
    disp(' ');
    disp('E_LO =');
    disp(gnsf.E_LO);
    disp('A_LO =');
    disp(gnsf.A_LO);
    disp('B_LO =');
    disp(gnsf.B_LO);
    disp('c_LO =');
    disp(gnsf.c_LO);
end

end


