function [newNU, sim] = TodorovLCPdynamics( sim )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% LCPdynamics.m
%
% Constructs matrices A and b, representing the LCP of the current time step.
    M = sim.dynamics.M;
   Gn = sim.dynamics.Gn;
   if sim.FRICTION
        Gf = sim.dynamics.Gf;
        U = sim.dynamics.U;
        E = sim.dynamics.E;
        G = sim.dynamics.G;
   end
    
   NU = sim.dynamics.NU;
   FX = sim.dynamics.FX;
  PSI = sim.dynamics.PSI;
   nc = length(sim.contacts);
   nb = length(M) / 6;
    h = sim.h;
  %% Construct A and b as LCP
  if sim.FRICTION
      %% Construct the matrix used for Todorov solver
      xc  = sim.dynamics.xc;
      GMi = G' .* (ones(3*nc, 1) * (1 ./M'));
      Todo_A = GMi * G;
      C = M .* NU + FX * h;   % this is bias in the original code 
      %MinvC = M\C;
      %v0 = G' * MinvC;
      v0 = GMi * C;
      if length(xc) ~= 3*nc
          %xc = [];
          xc = zeros(1, 3*nc);
      end
      mu = diag(U);
      [m, n] = size(mu);
      if m > n && n == 1 
          mu = mu';
      end
      %bnd = repeatEle(-PSI/h);  
      %bnd = - PSI / h;
      %bnd = bnd';  % (1 by nc)
      bnd = zeros(1, nc);
      % Solve with Todorov solver
      [x, res, L, it, flag, ~] = cminimize(xc, Todo_A, v0, mu, bnd);
      % REMEMBER to update on xc
      sim.dynamics.xc = x;
      %display( strcat( 'Residual: ', num2str(res) , '  within   :' ,  num2str(it) , '  iterations;   ', '  with flag:   ',  num2str(flag),...
      %    '  Contacts Num     : ', num2str(nc)));
      
      % PARSE the results from cminimize and then calculate the update
      fc = ceval(x, mu, bnd);
      
      impulse = G * fc(:); % get the generalized impulse in world frame
      v1 = (C + impulse) ./ M;
      v1 = reshape(v1, [6, nb]);
      newNU = v1;
      newNU = newNU(:);
  else  % Same but without Gf
      %% Construct the matrix used for Todorov solver
      xc  = sim.dynamics.xc;
      GnMi = Gn' .* (ones(3*nc, 1) * (1 ./ M'));
      Todo_A = GnMi * Gn;
      C = M .* NU + FX * h;   % this is bias in the original code 
      %MinvC = M\C;
      v0 = GnMi * C;
      mu = diag(U);
      bnd = -PSI / h;
      % Solve with Todorov solver
      [x, res, L, it, flag, ~] = cminimize(xc, Todo_A, v0, mu, bnd);
      % REMEMBER to update on xc
      sim.dynamics.xc = x;
      nc = length(x);   % no friction
      % fprintf('The residual is: %f, the iteration is: %d, the flag is: %d \n', res, it, flag);
   
      % PARSE the results from cminimize and then calculate the update
      fc = ceval(x, mu, 0);
      
      impulse = Gn * fc(:);     % get the generalized impulse in world frame
      v1 = (C + impulse) ./ M;
      v1 = reshape(v1, [6, nb]);
      newNU = v1;
      newNU = newNU(:);
  end
  
end

function [bnd] = repeatEle(PSI)
  rowPSI = repmat(PSI', 1, 3);
  matrixPSI = reshape(rowPSI, length(PSI), 3)';
  bnd = matrixPSI(:);
end
