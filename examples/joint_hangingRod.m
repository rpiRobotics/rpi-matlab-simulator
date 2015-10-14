%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
%

function sim = joint_hangingRod()

    % Initialize simulator
    sim = Simulator(0.005);      % Timestep of t = 0.01 seconds
    sim.drawJoints = true;     
    sim.H_dynamics = @mLCPdynamics;
    sim.userFunction = @plotPVA; 

    % Create an invisible static body
    staticBody = mesh_cylinder(7,1,0.2,1);
        staticBody.dynamic = false; 
        staticBody.visible = false;   % Don't bother showing the static body

    % Create a hanging body
    angle = pi/5; 
    hangingBody = mesh_cylinder(7,1,0.1,0.5);
        hangingBody.u = [-0.25*sin(angle); 0; 0.25-0.25*cos(angle)];
        hangingBody.quat = qt([0;1;0], angle);

    % Gather simulation bodies and add to simulator
    bodies = [staticBody hangingBody];
    sim = sim_addBody( sim, bodies );

    % Create joint
    sim = sim_addJoint(sim, 1, 2, [0;0;0.25], [0;1;0], 'spherical');

    % Run simulation
    sim = sim_run( sim );

end



