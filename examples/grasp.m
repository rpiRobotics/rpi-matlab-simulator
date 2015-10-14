function [sim, ke] = grasp( nboxes, maxIter )

    % the kinetic energy of the box with respect to the grippers 
    ke = [];

    %% A user function for controlling the wheel torque to achieve a certain speed.
    % userFunctions get called at the beginning of every timestep.  
    function sim = applyForceFunction( sim )
        F = 10*nboxes;
        kv = 0;
        avelD = 1;

        % compute predynamics
        sim = preDynamics(sim);

        % apply forces pushing in
        fx = [F 0 0]; 
        sim.bodies(2).Fext(1:3) = qtrotate(sim.bodies(1).quat,-fx');          
        sim.bodies(3).Fext(1:3) = qtrotate(sim.bodies(1).quat,fx');          

        % get the relative velocities of the box with respect to both grippers
        M = sim.dynamics.M;
        R = [sim.dynamics.Gn sim.dynamics.Gb];
        A = R'*M*R;
        b = R'*sim.dynamics.NU;

        % update the ke
        ke = [ke; 0.5*dot(b, A*b)]; 
    end


    %% Our main function that creates a chassis with four wheels 
    sim = Simulator(.01);
    sim.MAX_STEP = maxIter;
    sim.userFunction = @applyForceFunction;                 % Tell sim to use our userFunction
    sim.H_dynamics = @LCPdynamics;
    sim.H_dynamics = @NoSlip;  % comment this line out to use S-T again
    sim.H_solver = @pgs;
    sim.H_solver = @Lemke;
    sim.draw = false;
    %sim.drawContacts = true;
    sim.FRICTION = true; 
    sim.num_fricdirs = 4; 
    sim.drawJoints = false; 

    % ground
    ground = Body_plane([0; 0; -1.5], [0;0;1]);
      ground.color = [0 0 0];
      ground.dynamic = false;
      ground.visible = false;   
 
    % object 
    for i=1:nboxes
        boxes(i) = mesh_rectangularBlock(1, 1, 1);
        boxes(i).u = [-nboxes/2+(i-1)+0.5; 0; 0];
        boxes(i).color = [.3 .6 .5];
        boxes(i).mu = 10^20;
    end

    % apply a random velocity to the box
    box.nu = randn(6,1);
        
    % setup manipulator 
    gripper1 = mesh_cube();  
    gripper2 = mesh_cube();
    gripper1.u = [-nboxes/2+(nboxes-1)+1.5; 0; 0];
    gripper2.u = [-nboxes/2-0.5; 0; 0];
    gripper1.mu = 10^20;
    gripper2.mu = 10^20;  
    gripper1.color = [1 0 0];
    gripper2.color = [1 0 0];
    w1.u = [ 1.5;  0; 0];  
    w2.u = [-1.5;  0; 0]; 
    
    % Add bodies to simulator
    sim = sim_addBody(sim, [ground gripper1 gripper2 boxes]);
    
    % Create joints
     sim = sim_addJoint( sim, 1, 2, w1.u, [1;0;0], 'prismatic');
     sim = sim_addJoint( sim, 1, 3, w2.u, [1;0;0], 'prismatic');
    
    % Run the simulator
%    sim = sim_run_Drumwright( sim );
    sim = sim_run( sim );

end

