%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
%

function sim = cart ()

    sim = Simulator(.01);
    sim.H_dynamics = @mLCPdynamics;
    %sim.drawContacts = true;
    %sim.drawJoints = true; 
    %sim.draw = false; 
    %sim.MAX_STEP = 200;
    %sim.record = true; 
    
    % Ground
    ground = Body_plane([0; 0; 0],[0; .17; 1]);
        ground.color = [.7 .5 .5];
        ground.mu = 1;
    
    % Chassis
    chassis = mesh_rectangularBlock(1,3,0.25);
        chassis.u = [0; 0; 1];
        chassis.color = [.3 .6 .5];
        
    % Wheels
    wheel = mesh_cylinder(20,1,0.4,0.2);  wheel.facealpha = 1; 
    wheel.quat = qt([0 1 0],pi/2);
    w1 = wheel;
    w2 = wheel;
    w3 = wheel;
    w4 = wheel;
    w1.u = [ .65;  1.3; .75];  % w1.Fext(4) = -.2;
    w2.u = [-.65;  1.3; .75];  % w2.Fext(4) = -.2;
    w3.u = [ .65; -1.3; .75];
    w4.u = [-.65; -1.3; .75];
    
    % Add bodies to simulator
    sim = sim_addBody(sim, [ground chassis w1 w2 w3 w4]);
    
    % Create joints
     sim = sim_addJoint( sim, 2, 3, w1.u, [1;0;0], 'revolute');
     sim = sim_addJoint( sim, 2, 4, w2.u, [1;0;0], 'revolute');
     sim = sim_addJoint( sim, 2, 5, w3.u, [1;0;0], 'revolute');
     sim = sim_addJoint( sim, 2, 6, w4.u, [1;0;0], 'revolute');
    
    % Run the simulator
    sim = sim_run( sim );

end



