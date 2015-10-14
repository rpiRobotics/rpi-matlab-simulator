%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% LCPdynamics.m
%
% Constructs matrices A and b, representing the LCP of the current time step.

function [sim] = NoSlip( sim )

   global z;
   nb = sim.num_activeBodies; 
   nc = length(sim.contacts); 
   njc = sim.num_jointConstraints;
   nd = sim.num_fricdirs; 

    M = sim.dynamics.M;
   Gn = sim.dynamics.Gn;
   Gb = sim.dynamics.Gb; 
   if sim.FRICTION
        Gf = sim.dynamics.Gf;
        U = sim.dynamics.U;
   end
   NU = sim.dynamics.NU;
   FX = sim.dynamics.FX;
   PSI = sim.dynamics.PSI;
   psi_b = sim.dynamics.joint_bn; 
    h = sim.h;

   % search for no contacts
   if (nc == 0)   
     X = [M -Gb; Gb' zeros(njc)];
     q = [-M*NU - h*FX; zeros(njc,1)];
     u = X \ q;
     newNU = u(1:length(NU));
     return;
   end

   act = [];
   iM = inv(M);
   for i=1:nc
     % compute the candidate matrix for first friction direction
     cact = [act i*2-1];
     ncact = length(cact);
     X = [Gb Gf(:, cact)];
     Y = X'*iM*X;
     [R, p] = chol(full(Y - eye(size(Y))*1e-6));
     if (p == 0)
       act = cact;
     end

     % compute the candidate matrix for second friction direction
     cact = [act i*2];
     ncact = length(cact);
     X = [Gb Gf(:, cact)];
     Y = X'*iM*X;
     [R, p] = chol(full(Y - eye(size(Y))*1e-6));
     if (p == 0)
       act = cact;
     end
   end 

   % Mv^+ - Gb*lambda - Gn*cn - Gf*cf = Mv + h*fx
   % Gb'*v^+ = 0
   % Gf'*v^+ = 0
   % Gn'*v^+ + NU >= 0
   % | M  -Gb  -Gf  -Gn | | v^+    | + | -Mv - h*fx |
   % | Gb'              | | lambda |   | 0          |
   % | Gf'              | | cf     |   | 0          |
   % | Gn'              | | cn     |   | -PSI       |
   % From Cottle, p. 29, mixed LCP:
   % Au + Cv + a = 0
   % Du + Bv + b >= 0
   % yields LCP:
   % q = b - D*inv(A)*a
   % M = B - D*inv(A)*C
   % and u is -inv(A)*(a + C*v)
   nact = length(act);
   b = -PSI;
   ngc = size(M,1);
   A = full([M -Gb -Gf(:,act); Gb' zeros(njc,njc+nact); Gf(:,act)' zeros(nact,njc+nact)]);
   a = [-M*NU - h*FX; zeros(njc+nact,1)];
   D = [Gn' zeros(nc,njc+nact)];
   C = -D';

   if 1 % if this is set to 0, modified PPM method will not be used 
     N = D;
     v = -A \ a;
     [z, pivots, errcode] = pivot(A, N, v, PSI);

     % check the solution
     if (errcode == 0)
       DiA = D / A;
       q = b - DiA*a;
       M = -DiA*C;
       w = M*z + q;
       if (min(z) < -1e-8 || min(w) < -1e-8 || abs(z'*w) > 1e-8)
         errcode = -1;
       end
     end
  end

  if (0 || errcode < 0)
     % This is the full LCP
     DiA = D / A;
     q = b - DiA*a;
     M = -DiA*C;

     % setup QP options
     options = optimset('Algorithm', 'Active-set');

     % attempt to solve the QP instead
     if (length(z) ~= length(q))
       z = zeros(length(q), 1);
     end

     % check for zero q
     [z, fval, exitflag] = quadprog(full(M), q*0.5, -full(M), q, [], [], zeros(length(q),1), [], z, options);
     if (exitflag == -2)
       [z, fval, exitflag] = linprog([zeros(length(q),1); 1], -[full(M) ones(size(M,1),1)], q, [], [], zeros(length(q)+1,1), []);
       z = z(1:end-1);
    end
  end

   % determine new velocities
   u = -A \ (a + C*z);
   newNU = u(1:length(NU));
   sim.newNU = newNU;

end



