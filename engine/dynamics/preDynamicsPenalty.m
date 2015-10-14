%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% preDynamics.m
%
% Generates submatrices for penalty formulation 
%   M   - Mass-inertia
%   Gn  - Contact normal wrench
%   Gf  - Contact friction wrench
%   Gb  - Bilateral constraint wrench
%   E   - Mask matrix for friction
%   U   - Coefficients of friction
% 
%   NU  - Velocities, including rotational           These 3 are vectors
%   FX  - External forces, including rotational
%   PSI - Gap distances 

function sim = preDynamicsPenalty( sim )

  % We need to add joint bodies to the set of activeBodies.  Further, by
  % doing this first, we ensure that the mass-inertia matrix contains joint
  % bodies first.  This is useful in postDynamics when applying joint corrections.  
  nj = length(sim.joints); 
  for j=1:nj
     sim = sim_activateBodies( sim, sim.joints(j).body1id, sim.joints(j).body2id );
  end
  njc = sim.num_jointConstraints; 

  %% Useful vars 
  nb = 0;    % Number of bodies 
  for i=1:sim.num_bodies
    if (sim.bodies(i).dynamic)
      nb = nb + 1;
    end
  end
  nc = length(sim.contacts);    % Number of contacts
  

  %% Init submatrices 
  M = sparse(6*nb);
  Gn = sparse(6*nb,nc);
  if sim.FRICTION
      Gf = sparse(6*nb,nc);  
      U = sparse(nc);
  end
  Gb = sparse(6*nb,njc);
  NU = zeros(6*nb,1);   % Velocity, including angular   
  FX = sparse(6*nb,1);   % External force (not impulse!) 
  PSI = zeros(nc,1);    % Gap distance per contact, psi_n  
  sim.num_subContacts = 0; 
  

  %% Calculate submatrices
  
  % M, NU, and FX
  for b=1:length(sim.bodies)
     B = sim.bodies(b);
     if B.dynamic
        j = B.bodyDynamicID;
        M(6*j-5:6*j,6*j-5:6*j) = body_massInertiaMatrix(B);  
        NU(6*j-5:6*j) = B.nu;                  % NU
        FX(6*j-5:6*j) = B.Fext;                % FX
     end
  end

  % Gn, E, U, and Gf
  for cID=1:nc
      
    C = sim.contacts(cID);
    sim.num_subContacts = sim.num_subContacts + length(C.psi_n);    % TODO: count the number of subcontacts
    
    PSI(cID) = C.psi_n(1);                                          % PSI

    % For every contact, there are two bodies which enter Gn and Gf 
    % (unless contact was with a static body). 
    
    % Body 1
    B1 = sim.bodies(C.body1_id);
    if B1.dynamic
        r1 = C.p1' - B1.u;
        body1id = B1.bodyDynamicID;
        Gn_i1 = [-C.normal(1,:)'; cross3(r1,-C.normal(1,:)')];
        Gn(6*body1id-5:6*body1id,cID) = Gn_i1;
    end
    % Body 2
    B2 = sim.bodies(C.body2_id); 
    if sim.bodies(C.body2_id).dynamic
        r2 = (C.p1+C.normal*C.psi_n)' - B2.u;
        body2id = B2.bodyDynamicID;
        Gn_i2 = [C.normal(1,:)'; cross3(r2,C.normal(1,:)')];
        Gn(6*body2id-5:6*body2id,cID) = Gn_i2;
    end
    
    if sim.FRICTION
        % U
        U(cID,cID) = 0.5*B1.mu * B2.mu;     
        % Gf
        % TODO: Select the initial tangent opposing motion
        randTan = arbitraryTangent(C.normal(1,:)); 
        d = cross3(C.normal(1,:), randTan);    % Friction direction d
        if B1.dynamic
          Gf(6*body1id-5:6*body1id, 2*(cID-1)+1) = [randTan; cross3(r1,randTan)];
          Gf(6*body1id-5:6*body1id, 2*(cID-1)+2) = [d; cross3(r1,d)];
        end
        if B2.dynamic
          Gf(6*body2id-5:6*body2id, 2*(cID-1)+1) = [randTan; cross3(r2,randTan)];
          Gf(6*body2id-5:6*body2id, 2*(cID-1)+2) = [d; cross3(r2,d)];
        end
    end
  end
  
  %% Joint Dynamics 
  joint_bn = zeros(njc, 1);
    
  for j=1:nj
     sim = updateJoint( sim, j );
     Jnt = sim.joints(j); 
     b1id = sim.bodies(Jnt.body1id).bodyDynamicID;
     b2id = sim.bodies(Jnt.body2id).bodyDynamicID;
     constIndex = Jnt.constraintIndex; 

     [G1c G2c] = joint_Jacobians(sim,j);
     [C Cdot] = joint_constraintError(sim,j);   % TODO: This is probably not necessary.
     %[C Cdot] = Jnt.constraintError();
     %bj = 0.0*C/sim.h + Cdot;    
     bj = 0.9*Cdot;  % Maybe add a coefficient 0<=C<=1 for stability.
     
     if sim.bodies(Jnt.body1id).dynamic
        Gb(6*b1id-5:6*b1id, constIndex) = G1c; 
        %if sim.FRICTION
            %Gf(6*b1id-5:6*b1id, nd*nc+j) = G1f;   % TODO: not yet implemented
        %end
     end
     if sim.bodies(Jnt.body2id).dynamic
        Gb(6*b2id-5:6*b2id, constIndex) = G2c;  % TODO: fix indices 
        %if sim.FRICTION
            %Gf(6*b2id-5:6*b2id, nd*nc+j) = G2f; % TODO: this is wrong now
        %end
     end
     
     if sim.FRICTION
         % TODO: joint friction
     end
     
     % Store joint constraint errors for vector b
     joint_bn(constIndex,1) = bj;  

  end
  sim.dynamics.joint_bn = joint_bn; 
  
  
  %% Store values in the struct sim.dynamics  
  sim.dynamics.M = M;        % It seems that assignment here is faster than
  sim.dynamics.Gn = Gn;      % referencing the struct multiple times above.
  sim.dynamics.Gb = Gb; 
  if sim.FRICTION
      sim.dynamics.Gf = Gf;
      sim.dynamics.U = U;
  end
  sim.dynamics.NU = NU;
  sim.dynamics.FX = FX;
  sim.dynamics.PSI = PSI; 
  
end

