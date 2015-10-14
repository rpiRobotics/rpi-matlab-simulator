

% This function iterates over all simulation bodies and converts their
% spatial velocities to time-derivatives of quaternions 

function qdot = body_VelocityToQdotAll( sim, x)

    % determine q and v
    nb = size(x,1)/13;
    q = x(1:nb*7);
    v = x(nb*7+1:end);

    % Apply results from solution, as well as update bodies not in contact.   
    for i = 1:length(sim.bodies)     
        body = sim.bodies(i);  
        if ~body.dynamic, continue; end;    % Don't update static bodies. 
        j = body.bodyDynamicID;
        qdot(7*j-6:7*j) = body_VelocityToQdot(body, q(7*j-6:7*j), v(6*j-5:6*j) );
    end
end

