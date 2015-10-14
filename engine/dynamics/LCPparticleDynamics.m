%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% LCPdynamics.m
%
% Constructs matrices A and b, representing the MCP of the current time
% step, solves the MCP, and returns that solution z.  
%
% NOTE!: Whereas MCPdynamics returns the solution of the MCP for parsing,
% LCPdynamics returns the updated velocities NUnew.  The reason being that
% the LCP solution requires calculations such as inverse of the mass matrix
% M, Gn and Gf, which are already calculated in this function, making it
% computationally convenient to begin parsing the solution here.  
function LCPparticleDynamics( obj )

  %% Useful vars 
  num_bodies = length(obj.activeBodies);
  num_contacts = length(obj.Contacts); 
  fricdirs = 5;   % Important: Number of directions in discrete friction "cone"

  %% Init submatrices 
  M = zeros(3*num_bodies);
  Gn = zeros(3*num_bodies,num_contacts);
  
  if (strcmp(obj.SOLVER, 'fixed_point'))
      Gf = zeros(3*num_bodies, 2*num_contacts);
  else
      Gf = zeros(3*num_bodies,fricdirs*num_contacts);
  end
  U = eye(num_contacts);
  E = zeros(fricdirs*num_contacts,num_contacts);  
  %b = zeros(2*num_bodies + num_bodies*fricdirs,1);  % We assign b a value below.  
  NU = zeros(3*num_bodies,1);   % Velocity, including angular
  %NUnew = zeros(3*num_bodies,1);% Return variable; the updated velocities
  FX = zeros(3*num_bodies,1);   % External force (not impulse!)    Fext should be updated to be Pext
  PSI = zeros(num_contacts,1);  % Distance per contact, psi_n  

  %% Formulate submatrices
  % M
  for i=1:num_bodies
     body = obj.P{obj.activeBodies(i)};                % (activeBodies should never contain static bodies)
     if body.static > 0, continue; end                 % Don't incld static bds (this shouldn't happen)
     M(3*i-2:3*i,3*i-2:3*i) = body.mass*eye(3);        % 3x3 Mass matrix
  end
   
  % E
  for i=1:num_contacts
    E(fricdirs*i-(fricdirs-1):fricdirs*i,i) = ones(fricdirs,1);
  end

  % Gn, U, and b
  for i=1:num_contacts
    C = obj.Contacts(i);
    cID = C.id;

    % For every contact, there are two bodies which need Gn and Gf (unless
    % contact was with a static object).  
    body1id = obj.P{C.body_1}.BodyIndex;
    body2id = obj.P{C.body_2}.BodyIndex;
    
    % Gn
    Gn_i1 = [-C.normal];
    Gn_i2 = [ C.normal];
    % The following update should not include static bodies.
    if obj.P{C.body_1}.static == 0
        Gn(3*body1id-2:3*body1id,cID) = Gn_i1;
    end
    if obj.P{C.body_2}.static == 0
        Gn(3*body2id-2:3*body2id,cID) = Gn_i2;
    end
    
    
    % Gf
    if (strcmp(obj.SOLVER, 'fixed_point'))
        Gf_i1 = zeros(3,2);
        Gf_i2 = zeros(3,2);
        Gf_i1(1:3, 1) = arbitraryTangent(-C.normal);
        Gf_i2(1:3, 1) = arbitraryTangent(C.normal);
        Gf_i1(1:3, 2) = cross(-C.normal, Gf_i1(1:3, 1));
        Gf_i2(1:3, 2) = cross(-C.normal, Gf_i2(1:3, 1));
        
        %Gf_i1(4:6, 1:2) = [cross(r1, Gf_i1(1:3, 1)), cross(r1, Gf_i1(1:3,2))];  Inertia part of friction  
        %Gf_i2(4:6, 1:2) = [cross(r2, Gf_i2(1:3.,1)), cross(r2, Gf_i2(1:3, 2))];
        
        % The following update should not include static bodies.
        if obj.P{C.body_1}.static == 0
            Gf(3*body1id-2:3*body1id,2*cID-1:2*cID) = Gf_i1;
        end
        
        if obj.P{C.body_2}.static == 0
            Gf(3*body2id-2:3*body2id,2*cID-1:2*cID) = Gf_i2;
        end
        
    else
        for j=1:fricdirs
            d = rot(C.normal,((j-1)/fricdirs)*(2*pi)) * C.tangent;    % Friction direction d_j
            Gf_i1(:,j) = d;
            Gf_i2(:,j) = d;
        end
        
        % The following update should not include static bodies.
        if obj.P{C.body_1}.static == 0
            Gf(3*body1id-2:3*body1id,fricdirs*cID-(fricdirs-1):fricdirs*cID) = Gf_i1;
        end
        
        if obj.P{C.body_2}.static == 0
            Gf(3*body2id-2:3*body2id,fricdirs*cID-(fricdirs-1):fricdirs*cID) = Gf_i2;
        end
    end
    
    
    % With the MCP, this is where we would fill in a portion of b.  
    % The following will make redundant assignments to NU & FX, but it's 
    % for the best right now!  (Since we need PSI, and we're already looping)
    % Body 1
    if obj.P{C.body_1}.static == 0
        NU(3*body1id-2:3*body1id) = obj.P{C.body_1}.nu;
        FX(3*body1id-2:3*body1id) = obj.P{C.body_1}.Fext; 
    end
    % Body 2
    if obj.P{C.body_2}.static == 0
        NU(3*body2id-2:3*body2id) = obj.P{C.body_2}.nu;     % Per body
        FX(3*body2id-2:3*body2id) = obj.P{C.body_2}.Fext;   % Per body
    end
    % U
    U(cID,cID) = obj.P{C.body_2}.mu;  % Just take the particle's mu %obj.P{C.body_1}.mu * obj.P{C.body_2}.mu; 
    PSI(cID) = C.psi_n;                                 % Per contact!
  end

  %% Construct A and b as LCP 
  MinvGn = M\Gn;
  MinvGf = M\Gf; 
  MinvPext = M \ (FX * obj.h);
  if (strcmp(obj.SOLVER, 'fixed_point'))
  else
      
      A = [Gn'*MinvGn   Gn'*MinvGf  zeros(num_contacts)
          Gf'*MinvGn   Gf'*MinvGf  E
          U            -E'         zeros(num_contacts)];
      
      b = [ Gn'*(NU + M\FX*obj.h) + PSI/obj.h;    % FX*h could be replaced if we stored Pext instead of Fext
          Gf'*(NU + M\FX*obj.h);
          zeros(num_contacts,1) ];
      mm = length(b);
      %   Asparse = sparse(A);
      %   bsparse = sparse(b);
      %   save(['matrix_A_' num2str(mm) '.mat'], 'Asparse');
      %   save(['vector_b_' num2str(mm) '.mat'], 'bsparse');
      
      %       for k = 1:mm
      %           if A(k, k) == 0
      %               A(k, k) = A(k, k) + 1e-3;
      %           end
      %       end
      
      
%        [mm, ~] = size(A);
%        [U, S, V] = svd(A);
%        topk = round(mm * 0.6);
%        A_new = U(:, 1:topk) * S(1:topk, 1:topk) * V(:, 1:topk)';
       
      z0 = zeros(size(A,1),1);
      max_iter = 100;  % Maximum number of iterations
  end
  % Solve LCP  
  switch obj.SOLVER 
      case 'Lemke' 
          [z, iter, err] = lemke(A,b,z0); 
      case 'interior_point'
          [z,err] = ip_lcp(A,b,z0,100); % Currently, if using this, make sure warnings are off
      case 'fischer_newton'
          tol_rel = 0.0001;
          tol_abs = 10*eps; 
          [z err iter flag convergence msg] = fischer_newton(A, b, z0, max_iter, tol_rel, tol_abs, 'random',false);
      case 'minmap_newton'
          tol_rel = 0.0001;
          tol_abs = 10*eps;  
          [z err iter flag convergence msg] = minmap_newton(A, b, z0, max_iter, tol_rel, tol_abs, false);
      case 'multilevel'
          tol_rel = 0.0001;
          tol_abs = 10*eps;  
          [z, err, iter, flag, conv, m, t0] = multilevel(A, b, z0, 2, max_iter, tol_rel, tol_abs, false);
      case 'pgs'
          tol_rel = 0.0001;
          tol_abs = 10*eps;  
          [z err iter flag convergence msg] = pgs(A, b, z0, max_iter, tol_rel, tol_abs, false);
      case 'psor' 
          tol_rel = 0.0001;
          tol_abs = 10*eps; 
          [z err iter flag convergence msg] = psor(A, b, z0, 1.4, max_iter, tol_rel, tol_abs, false);
      case 'fixed_point'
          [pn_ellp1, pf_ellp1, error] = fixed_point(M, Gn, MinvPext, h, PSI, NU, num_contacts);
  end
  
  if (strcmp(obj.SOLVER, 'fixed_point'))
      %fprintf('The error is: %d\n', error);
      NUnew = NU + MinvGn * pn_ellp1 + MinvGf * pf_ellp1 + MinvPext;
      info = [];
      %disp('This is fixed_point method ');
  else
      if err ~= 0
          if size(err) == 1
              disp(['LCP Error: ' num2str(err)]);
          else
              disp('LCP Error');
          end
      end
      % Calculate updated velocities. Quaternion updates occur in Simulation.m
      % The impulse in the normal direction per active body.
      Pn = z(1:num_contacts);
      % The impulse in the friction directions per active body.
      Pf = z(num_contacts+1:num_contacts + fricdirs*num_contacts);
      % Calculate new velocites
      NUnew = NU + MinvGn*Pn + MinvGf*Pf + M\FX*obj.h;
      
      if strcmp(obj.SOLVER,'Lemke')
          solver_iteration = iter;
          solver_error = z'*(A*z + b);
          problem_size = mm;
          info = [solver_iteration; solver_error; problem_size];
      else
          info = [];
      end
  end
  
  obj.z = NUnew;
  obj.info = info; 
  
end 



