function outvars = getAllOutputs(stages)
% shortcut code to generate list of outputs such that all variables are
% included

% default is to output all variables
for i = 1:length(stages)    
    outvars(i).name = sprintf('y%02d',i);    
    outvars(i).fromStage = i;
    outvars(i).idxWithinStage = 1:stages(i).dims.n;
end
