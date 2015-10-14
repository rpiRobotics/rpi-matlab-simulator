%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
%

% Initialize simulator
sim = Simulator();
sim.MAX_STEP = 300;
%sim.DRAW = false;
sim.drawContacts = true;
sim.drawBoundingBoxes = true; 
sim.H_dynamics = @LCPdynamics; 
%sim.H_dynamics = @mLCPdynamics; 

% Create some bodies
b1 = scale_mesh(mesh_cube(), 2);
    b1.u = [0.1; 0.2; -1];
    b1.dynamic = false; 
    b1.quat = qt(rand(3,1),0.3); 
b2 = mesh_cube();
    b2.u = [0; 0; 1]; 
    b2.Fext(3) = -9.8 * b2.mass;
    b2.color = rand(1,3);
b3 = scale_mesh(mesh_icosahedron(),0.5);
    b3.u = [0.3; 0.07; 2];
    b3.Fext(3) = -9.8 * b3.mass;
    b3.color = rand(1,3); 
    %b3.quat = qt([0;0;1],pi/4-.01);
    b3.quat = qt(rand(3,1),pi/4);


% Add bodies to simulator
sim = sim_addBody(sim, [b1 b2 b3]);
sim.H_collision_detection = @BulletCD; 


% Run the simulator!
sim = sim_run(sim); 










