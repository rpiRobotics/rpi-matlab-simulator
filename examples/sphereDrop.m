

function sim = sphereDrop( )

    sim = Simulator(); 
    sim.userFunction = @plotEnergy; 
    sim.H_solver = @Lemke;
    sim.MAX_STEP = 600;

    P = Body_plane([0;0;0],[0;0;1]);
    S = Body_sphere(1,1);
    S.u = [0; 0; 10];
    
    sim = sim_addBody(sim, [P S]);
    sim = sim_run(sim); 

end

