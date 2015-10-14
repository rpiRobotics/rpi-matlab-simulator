%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
%

function sim = Pachinko( varargin )
    % Initialize simulator
    sim = Simulator(0.01);          % Timestep of t = 0.01 seconds
    sim.drawContacts = true; 
    %sim.drawBoundingBoxes = true; 
    sim.MAX_STEP = 500;
    sim.H_solver = @Lemke;
    %sim.draw = false; 

    %% Create static cylinders
    num_cylinders = 15; 
    dy = 0.8;
    dz = 0.5;
    Y = 0;
    Z = 0;
    change_row = 1;
    row_counter = 1;
    C(num_cylinders) = mesh_cylinder(11,1,0.1,3);
    for i=1:num_cylinders
       c = mesh_cylinder(11,1,0.1,3); 
       c.dynamic = false; 
       c.quat = qt([0;1;0], pi/2); 
       c.quat = qtmultiply( qt([1;0;0],pi/11) , c.quat);
       c.color = [.7 .7 .7];
       c.u = [ 0; Y ; Z ];
       C(i) = c; 

       if change_row == row_counter
          Z = -row_counter * dz;
          Y = - 0.5 * row_counter * dy;
          change_row = 1; 
          row_counter = row_counter + 1;
       else
          change_row = change_row + 1;
          Y = Y + dy; 
       end
    end

    staticBody = mesh_cylinder(7,1,0.2,3);
        staticBody.dynamic = false; 
        staticBody.quat = qt([0;1;0],pi/2); 
        staticBody.color = [.7 .7 .7];

    %% Create dynamic spheres
    if nargin > 0
        num_spheres = varargin{1}; 
    else
        num_spheres = 50;                       % Number of spheres
    end
    S(num_spheres) = Body_sphere(1,1); 
    sphere_mass = 0.2;  
    sphere_radius = 0.1;
    for i=1:num_spheres
       s = Body_sphere(sphere_mass, sphere_radius);
       s.color = rand(3,1); 
       s.u = [ 2*rand*sign(rand-0.5); rand*sign(rand-0.5); 1+2*rand];
       S(i) = s;
    end

    %% Add a plane to catch all the spheres
    P = Body_plane([0;0;-3],[0;0;1]);

    % Gather simulation bodies for the simulator
    bodies = [P C S];

    %% Add bodies to simulator and run
    sim = sim_addBody( sim, bodies );
    sim = sim_run( sim );
end



