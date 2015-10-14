%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
%
% Given a sim struct and two body IDs b1id and b2id, 
% "activates" the bodies and updates sim. 

function sim = sim_activateBodies( sim, b1id, b2id )
    if ~sim.bodies(b1id).active 
        sim.bodies(b1id).active = true;
        if sim.bodies(b1id).dynamic
            sim.bodies(b1id).bodyContactID = sim.num_activeBodies + 1;
            sim.num_activeBodies = sim.num_activeBodies + 1;
        end
    end
    if ~sim.bodies(b2id).active 
        sim.bodies(b2id).active = true;
        if sim.bodies(b2id).dynamic
            sim.bodies(b2id).bodyContactID = sim.num_activeBodies + 1;
            sim.num_activeBodies = sim.num_activeBodies + 1;
        end
    end
end


