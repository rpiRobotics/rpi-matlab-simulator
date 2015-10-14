%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
%

function sim = sliding_block ()

    sim = Simulator(.01);
    sim.MAX_STEP = 500;
    sim.H_dynamics = @mLCPdynamics;
    sim.num_fricdirs = 8;
    %sim.FRICTION = false; 
    %sim.userFunction = @plotEnergy; 
    
    % Ramp
    incline = pi/12; 
    ramp = mesh_rectangularBlock(1,2,0.1); 
        ramp.dynamic = false; 
        ramp.color = [.7 .5 .5];
        ramp.mu = 1;
        ramp.u = [0; 0; -0.05/cos(incline)]; 
        ramp.quat = qt([1 0 0],incline);
    
    % Block
    block = mesh_rectangularBlock(0.1,0.2,0.05);
        block.color = [.3 .6 .5];
        block.quat = ramp.quat; 
        block.u = [0; 0; 0.025/cos(incline)];  qtrotate(block.quat,[0;1;0]); 
        block.u = block.u + 0.5*qtrotate(block.quat,[0;1;0]);
    
    % Add bodies to simulator
    sim = sim_addBody(sim, [ramp block]);
    
    % Run the simulator
    sim = sim_run( sim );

end



