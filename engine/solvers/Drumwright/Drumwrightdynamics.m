%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% LCPdynamics.m
%
% Constructs matrices A and b, representing the LCP of the current time step.

function newNU = Drumwrightdynamics( sim )

   function [f, g] = quad(x, Q, c)
     f = x'*(Q*x*0.5 + c);
     g = Q*x + c;
   end

   function [c, ceq] = nonlcon(x, U)
     n = size(U,1);
     c = zeros(n,1);

     for i=1:n
       c(i) = x(n+i*2-1).^2 - x(n+i*2).^2 - U(i,i).^2*x(i).^2;
     end

     ceq = [];
   end

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

   % setup the options for the optimization
%   options = optimset('Algorithm', 'interior-point-convex');
   options = optimset();

   % prepare to solve frictionless LCP
   a = [-M*NU - h*FX; zeros(njc,1)];
   A = [M -Gb; Gb' zeros(njc)];
   D = [Gn' zeros(size(Gn,2), njc)];
   C = -D';

   % look for infinite friction LCP
   if (sim.INFINITE_FRICTION)
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
     b = -PSI;
     ngc = size(M,1);
     nact = nc;
     A = full([M -Gb -Gf; Gb' zeros(njc,njc+nc*2); Gf' zeros(nc*2,njc+nc*2)]);
     while (rank(A) < size(A,1))
       nact = nact - 1;
       A = full([M -Gb -Gf(:,1:nact*2); Gb' zeros(njc,njc+nact*2); Gf(:,1:nact*2)' zeros(nact*2,njc+nact*2)]);
     end
     a = [-M*NU - h*FX; zeros(njc+nact*2,1)];
     D = [Gn' zeros(nc,njc+nact*2)];
     C = -D';
     DiA = D / A;
     q = b - DiA*a;
     M = -DiA*C;
     z = lemke(M, q);
     w = M*z + q;
     epsilon = 1e-15;
     while (min(w) < -1e-5 || min(z) < -1e-5)
       z = lemke(M+eye(size(M))*epsilon, q);
       w = M*z + q;
       epsilon = epsilon * 10;
       if (epsilon > 1e-4)
         error('Epsilon too large in regularization');
       end
     end
     u = -A \ (a + C*z);
     newNU = u(1:length(NU));
max(abs(Gf'*newNU))
     return;
   end

   % solve the frictionless LCP first
   DiA = D / A;
   qq = -DiA*a;
   MM = -DiA*C;
   z = lemke(MM, qq);
%   options = optimset('Algorithm', 'interior-point-convex', 'MaxIter', 1000);
%   z = quadprog(MM, qq*0.5, -MM, qq, zeros(0,length(qq)), [], zeros(length(qq),1), [], zeros(length(qq), 1), options);

   % determine new velocity and lambda
   u = -A \ (a + C*z);
   newNU = u(1:length(NU));
   lambda = u(length(NU)+1:end);

   % compute contact velocities
    % determine the new velocity along the contact normal
   kappa = sum(Gn' * newNU);
max(abs(Gf'*newNU))

   % setup the new quadratic and linear objectives
   X = M \ [Gn Gf Gb]; 
   G = [Gn'; Gf'; Gb'] * X;
   c = [Gn' * (NU + M\FX*h) + PSI/h; Gf' * (NU+M\FX*h); zeros(njc,1)];

   % setup the non-interpenetration and kappa constraints (A >= b)
%   Aineq = [Gn'*X; -ones(1,size(Gn,2))*Gn'*X];
%   bineq = [-Gn'*(NU + FX*h) - PSI/h; -kappa];
   Aineq = [Gn'*X];
   bineq = [-Gn'*(NU + M\FX*h) + PSI/h];

   % setup the joint constraints
   Aeq = Gb'*X;
   beq = zeros(njc,1);

   % setup the lower variable bounds
   lb = [zeros(nc,1); ones(nc*2,1)*-inf; ones(njc,1)*-inf];

   % setup the options for the optimization
   if (nc > 200)
     G = full((G+G')/2);
     options = optimset('Algorithm', 'active-set', 'GradObj', 'on', 'MaxFunEvals', 100000, 'TolCon', 1e-6);
   else
     options = optimset('Algorithm', 'interior-point', 'GradObj', 'on', 'MaxFunEvals', 100000, 'TolCon', 1e-6);
   end

   zinit = [z; zeros(nc*2,1); lambda];

   % solve the quadratically constrained QP
%   z = quadprog(G,c, -full(Aineq), -bineq, -full(Aeq), -beq, lb, []); 

   z = fmincon(@(x) quad(x,G,c), zinit, -Aineq, -bineq, -Aeq, -beq, lb, [], @(x) nonlcon(x,U), options); 

   % determine newNU
   newNU = NU + M\FX*h + X * z;
newNU 
min(Gn'*newNU)
max(abs(Gf'*newNU))

end



