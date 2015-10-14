

% This function iterates over all simulation bodies and updates their 
% position as well as type-specific properties like vertex positions for 
% mesh bodies.  

function sim = body_updateAllFromX( sim, x)

    % determine q and v
    if (size(x,2) > 1)
      x = x';
    end
    nb = size(x,1)/13;
    q = x(1:nb*7);
    v = x(nb*7+1:end);

    % Apply results from solution, as well as update bodies not in contact.   
    for i = 1:length(sim.bodies)     
        body = sim.bodies(i);  
        if ~body.dynamic, continue; end;    % Don't update static bodies. 

        j = body.bodyDynamicID;
        sim.bodies(i) = body_updateFromX(body, q(7*j-6:7*j), v(6*j-5:6*j) );

        % Do type-dependent updates
        if strcmp(body.type,'mesh')
           sim.bodies(i) = body_updateMesh(sim.bodies(i)); 
        %elseif strcmp(body.type,'sphere')
        end

    end
end

