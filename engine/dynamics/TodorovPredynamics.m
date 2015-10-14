%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Right now still use the preDynamic.m


% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% preDynamics.m
%
% Generates submatrices that are common for dynamics formulations including
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
function sim = TodorovPredynamics( sim )
  % We need to add joint bodies to the set of activeBodies.  Further, by
  % doing this first, we ensure that the mass-inertia matrix contains joint
  % bodies first.  This is useful in postDynamics when applying joint corrections.  
  nj = length(sim.joints); 
  for j=1:nj
     sim = sim_activateBodies( sim, sim.joints(j).body1id, sim.joints(j).body2id );
  end
  njc = sim.num_jointConstraints; 

  %% Useful vars 
  nb = sim.num_activeBodies;    % Number of bodies with contacts
  nc = length(sim.contacts);    % Number of contacts
  % nd = sim.num_fricdirs;        % Number of directions in discrete friction "cone"
  nd = 2;
  %% Init submatrices 
  Mass = zeros(6*nb);
  Gn = zeros(6*nb,nc);
  if sim.FRICTION
      Gf = zeros(6*nb,nd*nc);  
      U = zeros(nc);
      E = zeros(nd*nc,nc);  % TODO: joint friction
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
     if B.active && B.dynamic
        cID = B.bodyContactID;
        Mass(6*cID-5:6*cID,6*cID-5:6*cID) = body_massInertiaMatrix(B);  
        NU(6*cID-5:6*cID) = B.nu;                  % NU
        FX(6*cID-5:6*cID) = B.Fext;                % FX
     end
  end
  
  M = zeros(6, nb);  % THE size is 6 by nb
  for i = 1 : nb
      M(:, i) = diag(Mass(i*6-5:i*6, i*6-5:i*6));
  end
  M = M(:);
  
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
        r1 = C.p1' - B1.u + PSI(cID) * (-C.normal(1, :))';
        body1id = B1.bodyContactID;
        Gn_i1 = [-C.normal(1,:)'; cross3(r1,-C.normal(1,:)')];
        Gn(6*body1id-5:6*body1id,cID) = Gn_i1;
    end
    % Body 2
    B2 = sim.bodies(C.body2_id); 
    if sim.bodies(C.body2_id).dynamic
        %r2 = (C.p1+C.normal*C.psi_n)' - B2.u;
        r2 = C.p1' - B2.u + PSI(cID) * (-C.normal(1, :))';
        body2id = B2.bodyContactID;
        Gn_i2 = [C.normal(1,:)'; cross3(r2,C.normal(1,:)')];
        Gn(6*body2id-5:6*body2id,cID) = Gn_i2;
    end
    
    if sim.FRICTION
        % E 
        E(nd*cID-(nd-1):nd*cID,cID) = ones(nd,1);   
        % U
        U(cID,cID) = 0.5*B1.mu * B2.mu;     
        % Gf
        % TODO: Select the initial tangent opposing motion
        randTan = arbitraryTangent(C.normal(1,:)); 
        if B1.dynamic
            d = [randTan,  cross(-C.normal(1, :)', randTan)];
            Gf(6*body1id-5:6*body1id, 2*cID-1:2*cID) = [d; cross(r1, d(:, 1)), cross(r1, d(:, 2))];
        end
        if sim.bodies(C.body2_id).dynamic
            d = [randTan, cross(C.normal(1, :)', randTan)];
            Gf(6*body2id-5:6*body2id, 2*cID-1:2*cID) = [d; cross(r2, d(:, 1)), cross(r2, d(:, 2))];
        end
%         for j=1:nd
%             d = rot(C.normal(1,:),((j-1)/nd)*(2*pi)) * randTan;    % Friction direction d
%             if B1.dynamic
%                 Gf(6*body1id-5:6*body1id, nd*(cID-1)+j) = [d; cross3(r1,d)];
%             end
%             if B2.dynamic
%                 Gf(6*body2id-5:6*body2id, nd*(cID-1)+j) = [d; cross3(r2,d)];
%             end
%         end
    end
  end
  
  % Construct G
  G = zeros(6*nb, 3*nc);
  for i = 1 : nc
    G(:, (1+nd)*(i-1)+1:(1+nd)*i) = [Gn(:, i), Gf(:, nd*(i-1)+1:nd*i)];
  end
  
  %% Joint Dynamics 
  joint_bn = zeros(njc, 1);
    
  for j=1:nj
     sim = updateJoint( sim, j );
     Jnt = sim.joints(j); 
     b1id = sim.bodies(Jnt.body1id).bodyContactID;
     b2id = sim.bodies(Jnt.body2id).bodyContactID;
     constIndex = Jnt.constraintIndex; 

     [G1c G2c] = joint_Jacobians(sim,j);
     [C Cdot] = joint_constraintError(sim,j);   % TODO: This is probably not necessary.
     %[C Cdot] = Jnt.constraintError();
     %bj = 0.0*C/sim.h + Cdot;    
     bj = Cdot;  % Maybe add a coefficient 0<=C<=1 for stability.
     
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
      sim.dynamics.E = E;
      sim.dynamics.G = G;
  end
  sim.dynamics.NU = NU;
  sim.dynamics.FX = FX;
  sim.dynamics.PSI = PSI; 
end
 
