

    %% Our main function that creates a chassis with four wheels 
    sim = Simulator(.01);
    sim.H_dynamics = @penalty;
    %sim.drawContacts = true;
    sim.FRICTION = true; 
    sim.MAX_STEP = 100;
    sim.penalty_integrator = @ode23s;
    
    % Ground
    ground = Body_plane([0; 0; -.2],[0.0; 0.0; 1]); 
        ground.color = [.7 .5 .5];
        ground.mu = 1;
    ground.dynamic = false;
    
    b2 = mesh_cube();
    b2.u = [0; 0; .3];
    b2.mu = 0.1; 
    b2.color = rand(1,3);
    b2.nu(3) = -10;
    b2.nu(6) = 1;
    
    % Add bodies to simulator
    sim = sim_addBody(sim, [ground b2]);
    
    % Run the simulator
    [sim, timings] = sim_run_penalty( sim );


