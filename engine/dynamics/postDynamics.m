%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% postDynamics.m
%
% This method is called AFTER a solution is found, but BEFORE kinematic
% updates are applied.  The dynamics solution is stored in sim.z
% Additionally, if dynamics formulation information is required, it is
% available in the struct sim.dynamics
% 

function postDynamics( sim )

  %BenderJointCorrection( sim ); 
  
  
  JointCorrection( sim ); 
  
end


