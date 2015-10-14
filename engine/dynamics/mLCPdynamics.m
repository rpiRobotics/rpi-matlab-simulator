%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% mLCPdynamics.m
%
% Constructs matrices A and b, representing the mixed linear 
% complementarity problem (mLCP) given the current contacts.  

function sim = mLCPdynamics( sim )

  M = sim.dynamics.M;             % Mass-inertia matrix
  Gn = sim.dynamics.Gn;   
  Gb = sim.dynamics.Gb; 
  if sim.FRICTION
     Gf = sim.dynamics.Gf;
     U = sim.dynamics.U;
     E = sim.dynamics.E; 
  end
  NU = sim.dynamics.NU;           % Vector of velocities, including rotational
  FX = sim.dynamics.FX;           % Vector of external forces, including rotational
  PSI = sim.dynamics.PSI;         % Vector of gap distances 
  h = sim.h;                      % Simulation time-step 
  
  nc = length(sim.contacts); 
  ns = nc; 
  nb = sim.num_activeBodies;
  %nj = length(sim.joints);        % Number of joints
  njc = sim.num_jointConstraints; 
  nd = sim.num_fricdirs; 
  
  % Construct A and b
  % With friction
  if sim.FRICTION && ~isempty(Gf)
      A = [ -M               Gb                 Gn           Gf          zeros(6*nb,nc)   
            Gb'              zeros(njc,njc+nc*(2+nd))
            Gn'              zeros(nc,njc)      zeros(nc,nc*(2+nd))
            Gf'              zeros(nd*nc,njc)   zeros(nd*nc,(1+nd)*nc)   E
            zeros(nc,6*nb)   zeros(nc,njc)      U           -E'          zeros(nc) ];   

      b = [ M*NU + FX*h        
            sim.dynamics.joint_bn 
            PSI / h
            zeros((nd+1)*nc,1) ];
  % No friction
  else
      A = [ -M    Gb    Gn   
            Gb'   zeros(njc,njc+nc)
            Gn'   zeros( nc,njc+nc) ]; 

      b = [ M*NU + FX*h      
            sim.dynamics.joint_bn 
            PSI / h                 ];
  end


% Solve the MCP  
  problem_size = size(A,1);
  z0 = zeros(problem_size,1);       
  big=10^20;    
  u = big*ones(problem_size,1);
  
  if sim.FRICTION && ~isempty(Gf)
      l = [-big*ones(6*nb+njc,1);
           zeros((2+nd)*nc+(ns-nc),1)];
  else
      l = [-big*ones(6*nb + njc,1)
       zeros(nc+(ns-nc),1)];
  end

  newNU = pathlcp(A,b,l,u,z0);
  sim.newNU = newNU;
end 




