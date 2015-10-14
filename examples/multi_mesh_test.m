%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
%
% An example script

function sim = multi_mesh_test

    % Initialize simulator
    sim = Simulator();
    sim.h = 0.015;
    sim.MAX_STEP = 500;
    sim.drawContacts = true;
    sim.drawBoundingBoxes = true; 
    sim.H_dynamics = @LCPdynamics; 
    sim.H_solver = @Lemke; 

    % Create some bodies
    b1 = scale_mesh(mesh_cube(), 4);
        b1.u = [0.1; 0.2; -2];
        b1.dynamic = false; 
        b1.quat = qt(rand(3,1),0.3); 
        b1.color = [.3 .3 .3];
    b2 = mesh_tetrahedron();
        b2.u = [2; 0; 1]; 
        b2.color = rand(1,3);
    b3 = scale_mesh(mesh_icosahedron(),0.5);
        b3.u = [0.3; 0.07; 3];
        b3.color = rand(1,3); 
        b3.quat = qt(rand(3,1),pi/4);
    b4 = scale_mesh(mesh_dodecahedron(),0.8);
        b4.u = [-.3; -.1; 1.5];
        b4.quat = qt(rand(3,1),pi/4); 
    b5 = scale_mesh(mesh_octahedron(),1.2);
        b5.u = [0; 2; 2];


    % Add bodies to simulator
    sim = sim_addBody(sim, [b1 b2 b3 b4 b5]);


    % Run the simulator!
    sim = sim_run(sim); 

end





