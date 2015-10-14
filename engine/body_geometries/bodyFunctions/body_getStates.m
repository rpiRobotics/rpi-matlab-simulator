

% This function iterates over all simulation bodies and gets their states 

function x = body_GetStates( sim )

    % Apply results from solution, as well as update bodies not in contact.   
    for i = 1:length(sim.bodies)     
        body = sim.bodies(i);  
        if ~body.dynamic, continue; end;    % Don't update static bodies. 
        j = body.bodyDynamicID;
        x(7*j-6:7*j) = [body.u; body.quat];
    end
end

