%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
%
% Runs a simulator 

function sim = sim_run( sim )

    % Initialize graphics 
    sim = sim_draw_init( sim );
    if ~sim.draw, disp('Simulating without GUI, make sure you have set sim.MAX_STEP to a reasonable value.'); end
    
    % Turn "on" gravity
    if sim.gravity, sim = sim_setGravity( sim ); end
    
    % Remove to run without pause
    pause(); 

    %% Simulation loop 
    tic;
    sim.START_TIME = tic; 
    for timestep = 1:sim.MAX_STEP 
        tic;                                % Start step timer
        sim.time = sim.time + sim.h;
        sim.step = sim.step + 1;
        
        % Run user-defined function
        if ~isempty(sim.userFunction), sim = feval( sim.userFunction, sim ); end

        % Generate a contact set, and identify participating dynamic bodies 
        if ~isempty( sim.H_collision_detection )
            sim = feval( sim.H_collision_detection, sim ); 
        end

        % Formulate dynamics and solve for new body velocities
        sim.newNU = [];
        if ~isempty(sim.contacts) || sim.num_jointConstraints > 0  
            sim = preDynamics( sim ); 
            sim = feval(sim.H_dynamics, sim);
        end
         
        % Apply results from solution, as well as update bodies not in contact.   
        sim = body_updateAllBodies(sim);
        
        % Correct joint errors
        if sim.jointCorrection, sim = joint_correction( sim ); end
        
        % Update title and graphics
        if sim.draw
            figure(sim.figure);
            title(['Timestep: ' num2str(sim.step)]);
            if sim.drawContacts, sim = sim_drawContacts(sim); end
            if sim.drawJoints,   sim = sim_drawJoints(sim);   end
            axis equal;
            drawnow; 
        else
            disp(['Step ' num2str(sim.step) ' took ' num2str(toc) ' seconds']);
        end
        
        % Record data for playback 
        if sim.record, sim = sim_record(sim); end 
        
    end % End simulation loop
    sim.TOTAL_TIME = toc(sim.START_TIME); 

    disp(['Simulation finished!  Total time: ' num2str(sim.TOTAL_TIME) ' seconds.']);
    
end % End sim_run()












