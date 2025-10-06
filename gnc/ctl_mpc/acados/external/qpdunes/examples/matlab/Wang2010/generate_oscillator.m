%%
%function [ output_args ] = generate_oscillator( input_args )
%GENERATE_OSCILLATOR Sets up system and control matrices for the oscillator
%published in Wang2010
%   
clc
% Spring constant
k = 10;
% Spring length
l = 0.4;

% Mass
m = 1;

L = 5 * l;


Ac = zeros(8, 8);

% Identity
Ac(1 : 4, 5 : 8) = eye(4);

%
Ac(5, 1 : 4) = [-2 * k / m, k / m, 0 , 0];
Ac(6, 1 : 4) = [k / m,  -2*k/m, k/m, 0];
Ac(7, 1 : 4) = [0, k / m, -2 * k /m, k / m];
Ac(8, 1 : 4) = [0, 0, k /m, -2 * k / m];

Bc = zeros(8, 3);
Bc(5, :) = [-1/m, 1 / m, 0];
Bc(6, :) = [0, 0, 1 / m];
Bc(7, :) = [0, -1/m, 0];
Bc(8, :) = [0, 0, -1/m];

Cc = eye(size(Ac));
Dc = zeros(size(Ac,1),size(Bc,2));

Ts = 0.1;
[A,B,C,D] = c2dm(Ac,Bc,Cc,Dc,Ts);

% Weight of states 
Q = eye(8) * 0.1;
% Weight of controls
R = eye(3) * 0.01;
% Initial value
xinit = [0.1 0.2, 0.0 0.0, 0.0, 0.0, 0.0, 0.0 ]';

lbu = ones(3, 1) * (-0.2);
ubu = -lbu;

lbx = -inf(8, 1);
ubx = -lbx;

%  save oscillator-MPC.mat A B Q R xinit lbu ubu lbx ubx

%spy(A)

%end

