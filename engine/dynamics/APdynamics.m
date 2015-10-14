%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% mLCPdynamics.m
%
% Constructs matrices A and b, representing the mixed linear 
% complementarity problem (mLCP) given the current contacts.  

function mLCPdynamics( sim )
  M = sim.dynamics.M;             % Mass-inertia matrix
  Gn = sim.dynamics.Gn;   
  if sim.FRICTION
     Gf = sim.dynamics.Gf;
     U = sim.dynamics.U;
     E = sim.dynamics.E; 
  end
  NU = sim.dynamics.NU;           % Vector of velocities, including rotational
  FX = sim.dynamics.FX;           % Vector of external forces, including rotational
  PSI = sim.dynamics.PSI;         % Vector of gap distances 
  h = sim.h;                      % Simulation time-step 
  
  nc = length(sim.Contacts); 
  nb = length(sim.activeBodies);
  nj = length(sim.Joints);          % Number of joints
  nd = sim.num_fricdirs; 
  
  % Construct A and b
  % With friction
  if sim.FRICTION
      A = [ -M                Gn                       Gf   zeros(6*nb,nc+nj)  % Note: first line is negated
            Gn'               zeros(nc+5*nj,nc*(2+nd)+7*nj)
            Gf'               zeros(nd*nc+nj,(1+nd)*nc+6*nj)        E
            zeros(nc+nj,6*nb) U                       -E'   zeros(nc+nj) ];   

      b = [ M*NU + FX*h                                                
            PSI * 0         % TODO: do this right...
            zeros((nd+1)*nc+nj,1) ];
  % No friction
  else
      A = [ -M    Gn                       
            Gn'   zeros(nc+5*nj) ]; 

      b = [ M*NU + FX*h                                 
            PSI * 0        % TODO: do this right...
            sim.dynamics.joint_bn ];
  end
 
  % Store formulation, to be passed to solver.
  sim.dynamics.A = A;
  sim.dynamics.b = b;
end 




