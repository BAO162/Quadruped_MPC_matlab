function [iteration,phase]=setIterations(nIterations,currentIteration,IterationsBetweenMpc)
  iteration = floor(mod((currentIteration / IterationsBetweenMpc) ,nIterations));
  phase = mod(currentIteration ,(IterationsBetweenMpc * nIterations)) / (IterationsBetweenMpc * nIterations);
end
