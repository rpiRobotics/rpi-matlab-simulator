function sim = ten_balls_sim()
% Duplicate the example in Todorov's paper, ten balls dropping randomly. 
Nb = 10;   % number of balls
Np = 5;    % number of planes
Color2Use = hsv(Nb);

%sim = Simulator(0.005);
sim = Simulator(0.005);
sim.MAX_STEP = 200;
%sim.H_dynamics = @mLCPdynamics;
%sim.H_solver = @pathsolver; 
sim.H_dynamics = @TodorovLCPdynamics;

P1 = Body_plane([0; 0; -1], [0; 0; 1]);   % bottom
P2 = Body_plane([0; 1; 0], [0; -1; 0]);   % left
P3 = Body_plane([0; -1; 0], [0; 1; 0]);   % right 
P4 = Body_plane([-1; 0; 0], [1; 0; 0]);   % front 
P5 = Body_plane([1; 0; 0], [-1; 0; 0]);   % back
P1.visible = 0; P2.visible = 0; P3.visible = 0; P4.visible = 0; P5.visible =0;
sim = sim_addBody(sim, [P1, P2, P3, P4, P5]);

  R = 0.1 + 0.05 * rand(1, Nb); % radius
  P =  zeros(3, Nb);  % position
  V = randn(3, Nb);      % velocities
  W = randn(3, Nb);      % angular velocities
  V(3, :) = V(3, :) + 1;   % z-direction velocity + 1

Dense = 500;
m = 4/3*pi*R.^3 * Dense;

for i = 1 : Nb
    s = Body_sphere(m(i), R(i));
    s.color = Color2Use(i, :);
    s.num_sphere_verts = 20;
    s.nu = [V(:, i); W(:, i)];
    s.Fext = m(i)*[0; 0; -9.8; 0; 0; 0];
    checkSphere = true;
    while checkSphere
        checkSphere = false;
        % randomize the initial position
        x = -0.8  + 1.6 * rand(1);
        y = -0.8  + 1.6 * rand(1);
        z = -0.8 + 1.6 * rand(1);
        Pnow = [x; y; z];
        % check position against all other spheres
        for j = 1 : i-1
            if norm(P(j) - Pnow) - R(j) - R(i) < 1e-6
                checkSphere = true;
            end
        end
    end
    P(:, i) = [x ; y ;  z];
    s.u  = [x; y ; z];
    sim = sim_addBody(sim, s);
end

%  Run the simulator! 
sim = sim_run_Todorov(sim);
sim = sim_run(sim);
end

