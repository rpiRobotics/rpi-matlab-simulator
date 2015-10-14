%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
%
% Runs a simulator 

function [sim, times] = sim_run_penalty( sim )

    % computes the ODE for the penalty method
    function dxdt = ode(t, x, sim)

      % update bodies from x
      sim = body_updateAllFromX(sim,x);

       % Run user-defined function
      if ~isempty(sim.userFunction), sim = feval( sim.userFunction, sim ); end

      % Generate a contact set, and identify participating dynamic bodies 
      if ~isempty( sim.H_collision_detection )
          sim = feval( sim.H_collision_detection, sim ); 
      end

      % Formulate dynamics and determine new accelerations 
      sim = preDynamicsPenalty( sim ); 
      a = feval( sim.H_dynamics, sim);

      % TODO: computes forces to correct joint errors
%      if sim.jointCorrection, sim = joint_correction( sim ); end

      % transform nu to qdot 
      qdot = body_VelocityToQdotAll(sim, x);

      % set dxdt as qdot and a
      dxdt = [qdot'; a];

    end

    % Initialize graphics 
    sim = sim_draw_init( sim );
    if ~sim.draw, disp('Simulating without GUI, make sure you have set sim.MAX_STEP to a reasonable value.'); end
    
    % Turn "on" gravity
    if sim.gravity, sim = sim_setGravity( sim ); end
    
    pause(); 

    % get initial position and velocity
    q = body_getStates(sim);
    v = body_getVelocities(sim);
    qv = [q v];

    %% Simulation loop 
    times = [];
    tic;
    sim.START_TIME = tic; 
    for timestep = 1:sim.MAX_STEP 
        tic;                                % Start step timer

        % integrate
        [tout, qv] = sim.penalty_integrator(@(t,x)ode(t,x,sim), [sim.time sim.time+sim.h], qv);
        qv = qv(end,:);

        % update the bodies and draw them
        sim = body_updateAllFromX(sim,qv);
        if (sim.draw)
          for i=1:sim.num_bodies
            if (sim.bodies(i).dynamic)
              body_draw(sim.bodies(i));
            end
          end
        end

        % update time and step
        sim.time = sim.time + sim.h;
        sim.step = sim.step + 1;
        times = [times; toc];

        % Update title and graphics
        if sim.draw
            figure(sim.figure);
            title(['Timestep: ' num2str(sim.step)]);
            if sim.drawContacts, sim = sim_drawContacts(sim); end
            if sim.drawJoints,   sim = sim_drawJoints(sim);   end
            %axis equal;
            drawnow; 
        else
            disp(['Step ' num2str(sim.step) ' took ' num2str(times(end)) ' seconds']);
        end
        
        % Record data for playback 
        if sim.record, sim = sim_record(sim); end 
        
    end % End simulation loop
    sim.TOTAL_TIME = toc(sim.START_TIME); 

    disp(['Simulation finished!  Total time: ' num2str(sim.TOTAL_TIME) ' seconds.']);
    
end % End sim_run()












