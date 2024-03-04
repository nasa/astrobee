function par = newParam(name, maps2stage, maps2data)
% Shortcut code to define a parameter of the FORCES solver.
% 
%    PAR = NEWPARAM(NAME, MAPS2STAGE, MAPS2DATA) returns a struct that
%    defines a valid parameter to be used with the FORCES code generator,
%    where NAME is a label of the parameter, MAPS2STAGE defines to which
%    stage variable the output is mapped, and MAPS2DATA defines the
%    matrix or vector that this parameter substitutes.
%
%    Example: to have the affine equality vector "stages(1).eq.c" as a  
%             parameter (often used in MPC), call 
%                    
%             par = newParameter('myparameter', 1, 'eq.c');
%
% See also FORCES_LICENSE

par.name = name;
par.maps2stage = maps2stage;
par.maps2data = maps2data;
