% TEST 
function sim = Todorov_sphere_on_sphere()

    % Initialize simulator
    sim = Simulator(0.01);
    %sim.drawContacts = true;
    sim.MAX_STEP = 500; 
    sim.H_dynamics = @TodorovLCPdynamics; 
    %sim.H_dynamics = @mLCPdynamics; 

    % A sphere
    s1 = Body_sphere(1,2);
        s1.mass = 20;
        s1.u = [0; 0; -1]; 
        s1.color = [.7 .7 .7];
        s1.num_sphere_verts = 25; 

    s2 = Body_sphere(0.25,0.25);
        s2.u = [-0.05; 0.05; 1.25];
        s2.num_sphere_verts = 15;

    P = Body_plane([0;0;-3],[0;0;1]); 
        
    sim = sim_addBody(sim, [P s1 s2]);    % Add bodies to simulator

    % Run the simulator!
    sim = sim_run_Todorov(sim); 

end