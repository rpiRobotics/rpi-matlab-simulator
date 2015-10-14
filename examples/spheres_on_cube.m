%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
%
% Note: With so many bodies (and the epsilons in the collision detection
% large) the problem size grows very large so you may see may warnings
% from the Lemke solver in LCPdynamics.  

function sim = spheres_on_cube(varargin)

    % Initialize simulator
    sim = Simulator();
    sim.h = 0.01;
    sim.drawContacts = true;
    %sim.MAX_STEP = 250; 
    %sim.draw = false;
    %sim.H_dynamics = @LCPdynamics; 
    sim.H_dynamics = @mLCPdynamics; 

    % A cube
    c = scale_mesh( mesh_cube(), 2 );
        c.u = [0; 0; -1]; 
        c.dynamic = false;
        c.quat = qt([1 1 0],.2); 
        c.color = [.7 .7 .7];

    % Make a stack of cubes, N = the number of layers
    if nargin > 0
        N = varargin{1};
    else
        N = 1;
    end
    bodies = c; 
    mass = 0.1;
    radius = 0.1;
    for n = 1:N
      for row = -0.5 : 4.5*radius : 0.5
        for col = -0.5 : 2.5*radius : 0.5
          s = Body_sphere( mass, radius ); 
          s.u = [row; col; n*3*radius];
          s.color = rand(3,1); 
          bodies = [bodies s]; 
        end
      end
    end

    sim = sim_addBody(sim, bodies); % Add bodies to simulator

    % Run the simulator!
    sim = sim_run(sim); 

end

