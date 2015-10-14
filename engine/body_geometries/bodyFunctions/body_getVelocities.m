

% This function iterates over all simulation bodies and gets their velocities 

function x = body_GetVelocities( sim )

    % Apply results from solution, as well as update bodies not in contact.   
    for i = 1:length(sim.bodies)     
        body = sim.bodies(i);  
        if ~body.dynamic, continue; end;    % Don't update static bodies. 
        j = body.bodyDynamicID;
        x(6*j-5:6*j) = body.nu;
    end
end

