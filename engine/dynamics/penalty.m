function a = penalty( sim )

  % setup kp and kv
  kp = 100;
  kv = 1;

  % get the necessary data
  M = sim.dynamics.M;
  Gn = sim.dynamics.Gn;
  Gf = sim.dynamics.Gf;
  PSI = sim.dynamics.PSI;
  NU = sim.dynamics.NU;
  U = sim.dynamics.U;

  % determine the normal velocities at the contact points
  xdn = Gn'*NU;

  % compute the corrective forces
  cn = (-kp*PSI - kv*xdn);
  f = Gn*cn;
 
  % compute the friction forces
  xdt = Gf'*NU;
  D = zeros(size(Gf,2),size(Gn,2));
  for i=1:size(Gn,2)
    d = [xdt(i*2-1) xdt(i*2)];
    if norm(d) > 0
      d = d/norm(d);
      D(i*2-1,i) = -d(1);
      D(i*2,i) = -d(2);
    end    
  end
  ff = Gf*D*U*abs(cn); 

  % compute the accelerations
  a = M \ (f + ff + sim.dynamics.FX);

