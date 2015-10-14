

% This function iterates over all simulation bodies and updates their 
% position as well as type-specific properties like vertex positions for 
% mesh bodies.  

function sim = body_updateAllBodies( sim )

    % Apply results from solution, as well as update bodies not in contact.   
    for i = 1:length(sim.bodies)     
        body = sim.bodies(i);  
        if ~body.dynamic, continue; end;    % Don't update static bodies. 

        bodyCID = body.bodyContactID; 
        if body.active  % Body had >= 1 constraint
            sim.bodies(i) = body_updatePosition(body, sim.h, sim.newNU(6*bodyCID-5:6*bodyCID) );
        else            % Body had no constraints (just apply external forces)
            sim.bodies(i) = body_updatePosition(body, sim.h);
        end

        % Do type-dependent updates
        if strcmp(body.type,'mesh')
           sim.bodies(i) = body_updateMesh(sim.bodies(i)); 
        %elseif strcmp(body.type,'sphere')
        end
        
        % Update graphics
        if sim.draw
            body_draw(sim.bodies(i)); 
        end
    end

end

