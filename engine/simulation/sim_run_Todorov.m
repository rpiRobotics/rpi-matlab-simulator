function sim = sim_run_Todorov( sim )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
%
% Runs a simulator 

    % Initialize graphics 
    sim = sim_draw_init( sim );
    if ~sim.draw, disp('Simulating without GUI, make sure you have set sim.MAX_STEP to a reasonable value.'); end
    
    % Turn "on" gravity
    if sim.gravity, sim = sim_setGravity( sim ); end
    
    % Currently, the LCP formulation doesn't handle bilateral constraints,
    % so check this for user's sanity.
    if ~isempty(sim.joints) && ~isequal(sim.H_dynamics, @mLCPdynamics)
       disp(' *** So sorry, but joints are only supported when using @mLCPdynamics. ***');
       error('This can be fixed by setting sim.dynamics=@mLCPdynamics before running.');
    end
    
    pause(1); 

    %% Simulation loop 
    tic;
    sim.START_TIME = tic; 
     
    for timestep = 1:sim.MAX_STEP 
        tic;                                % Start step timer
        sim.time = sim.time + sim.h;
        sim.step = sim.step + 1;

        % Generate a contact set, and identify participating dynamic bodies 
        sim = feval( sim.H_collision_detection, sim ); 

        newNU = [];
        % Formulate dynamics and solve for new body velocities
        if ~isempty(sim.contacts) || sim.num_jointConstraints > 0   
            % 
            sim = TodorovPredynamics( sim); 
            % For the very first, make xc EMPTY
            if ~isfield(sim.dynamics, 'xc')
                sim.dynamics.xc = [];
            end
            % Modification to keep track of the changing xc
            [newNU, sim] = feval( sim.H_dynamics, sim );  
            %newNU = feval( sim.H_solver, sim );
        end
        sim.newNU = newNU; 
         
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
            %axis equal;
            drawnow; 
        else
            disp(['Step ' num2str(sim.step) ' took ' num2str(toc) ' seconds']);
        end
        
        % Record data for playback 
        if sim.record, sim = sim_record(sim); end 
        
        % Run user-defined function
        if ~isempty(sim.userFunction), sim = feval( sim.userFunction, sim ); end

    end % End simulation loop
    sim.TOTAL_TIME = toc(sim.START_TIME); 

    disp(['Simulation finished!  Total time: ' num2str(sim.TOTAL_TIME) ' seconds.']);
    
end % End sim_run_Todorov()

 
