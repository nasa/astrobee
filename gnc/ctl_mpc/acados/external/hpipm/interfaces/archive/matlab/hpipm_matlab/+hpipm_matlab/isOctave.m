function retval = isOctave
  persistent cacheval;  % speeds up repeated calls

  if isempty (cacheval)
    cacheval = (exist ("OCTAVE_VERSION", "builtin") > 0);
  end

  retval = cacheval;
end