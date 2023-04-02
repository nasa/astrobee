function outvar = newOutput(name, fromStage, idxWithinStage)
% Shortcut code to define an output of the FORCES solver.
% 
%    OUTVAR = NEWOUTPUT(NAME, FROMSTAGE, IDXWITHINSTAGE) returns a struct
%    that defines a valid output to be used with the FORCES code generator,
%    where NAME is a label of the output, FROMSTAGE defines from which
%    stage variable the output is retrieved, and IDXWITHINSTAGE defines the
%    indices within that stage variable.
%
%    Example: to have variables 5 to 8 from stage variable 11 as outputs of
%             the generated solver, call
%                    
%             output = newOutput('myoutput', 11, 5:8);
%
% See also FORCES_LICENSE

outvar.name = name;
outvar.fromStage = fromStage;
outvar.idxWithinStage = idxWithinStage;
