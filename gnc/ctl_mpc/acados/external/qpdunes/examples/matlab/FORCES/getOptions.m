function opts = getOptions(varargin)
% Returns options structure for generating code with FORCES.
% 
%    OPTS = GETOPTIONS returns a default options struct.
%
%    OPTS = GETOPTIONS(NAME) returns a default options struct with the 
%    solver named NAME.
%
% See also GENERATECODE FORCES_LICENSE

% default name
if (nargin == 1 && isa(varargin{1},'char') )
    opts.name = varargin{1};
else
    opts.name = 'FORCES';
end

% maximum number of iterations
opts.maxit = 1000;

% default line search options
opts.linesearch.factor_aff = 0.8;
opts.linesearch.factor_cc = 0.9;
opts.linesearch.minstep = 1e-6;
opts.linesearch.maxstep = 0.9995;

% default accuracy / convergence options
opts.accuracy.mu = 1e-6;
opts.accuracy.ineq = 1e-6;
opts.accuracy.eq = 1e-6;
opts.accuracy.rdgap = 1e-3;

% printlevel
opts.printlevel = 2;
